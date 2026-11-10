//! JPEG 像素解码器（Baseline Sequential，即 SOF0）
//!
//! 目标：把 `file.rs` 解析出来的“段列表”还原成像素，最终写成 PPM P6 文件。
//!
//! ```text
//!  JPEG 文件                           本模块做的事
//!  ┌──────────────┐
//!  │ SOI / APP0   │  忽略
//!  │ DQT          │ ──► 量化表 Q(u,v)                 （file.rs 已解好，直接取用）
//!  │ SOF0         │ ──► 图像宽高 / 分量 / 采样因子     （file.rs 已解好）
//!  │ DHT          │ ──► 建 Huffman 规范码表 min_code/max_code/value_ptr
//!  │ SOS + 熵数据 │ ──► ① 位流解 Huffman ② 反量化 ③ IDCT ④ 电平搬移
//!  │ EOI          │  结束
//!  └──────────────┘
//!                         ↓ 每个分量得到一张“样本平面” (Y, Cb, Cr)
//!                    ⑤ 上采样（4:2:0 → 4:4:4）
//!                    ⑥ YCbCr → RGB
//!                    ⑦ 写 PPM P6
//! ```
//!
//! 数学约定（贯穿全文）：
//! * 一块 (block) = 8×8 个样本，下标 `x` 为列（水平），`y` 为行（垂直）。
//! * DCT 系数 `F(u,v)`：`u` 是水平频率，`v` 是垂直频率，按“自然顺序”存成
//!   `F[v*8 + u]`（`file.rs` 里 DQT 已经把 zig-zag 还原成自然顺序了）。
//! * 样本值在 JPEG 内部是“已减去 128 的有符号数”，输出前要加回 128（电平搬移）。

use std::format;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::jpeg::file::{
    FrameHeader, HuffmanTable, JpegFile, JpegSegment, QuantizationTable, ScanHeader,
    ZIGZAG_TO_NATURAL,
};

// ---------------------------------------------------------------------------
// 数据结构
// ---------------------------------------------------------------------------

/// 解码后的一张图像。
///
/// 前 6 个字段沿用你原来的设计；`max_h / max_v` 是新增的，
/// 作用只有一个：上采样时要算“放大倍数”，即 `hmax / h`（见 `to_rgb`）。
pub struct JpegImage {
    pub width: usize,
    pub height: usize,
    pub components: Vec<JpegComponent>,
    pub quant_tables: Vec<QuantizationTable>,
    pub huffman_tables: Vec<HuffmanTable>,

    pub restart_interval: usize,

    /// 所有分量中最大的水平采样因子（4:2:0 时为 2）
    pub max_h: u8,
    /// 所有分量中最大的垂直采样因子（4:2:0 时为 2）
    pub max_v: u8,
}

/// 单个分量（Y / Cb / Cr）的元信息 + 解码后的样本平面。
pub struct JpegComponent {
    pub id: u8,
    pub h_sampling: u8,
    pub v_sampling: u8,
    pub quant_table_id: u8,

    // ------- 以下为解码阶段新增 -------
    /// 本分量的真实样本宽度 = ⌈图像宽 × h / hmax⌉
    /// （4:2:0 时色度分量只有亮度的一半宽，所以是 1920 而不是 3840）
    pub width: usize,
    /// 本分量的真实样本高度 = ⌈图像高 × v / vmax⌉
    pub height: usize,
    /// 平面横向块数（含为了凑整 MCU 而在右侧补的块）
    pub blocks_x: usize,
    /// 平面纵向块数（含为了凑整 MCU 而在下方补的块）
    pub blocks_y: usize,
    /// 平面像素宽 = blocks_x × 8（补过 8 的整数倍，方便直接按块写入）
    pub plane_w: usize,
    /// 平面像素高 = blocks_y × 8
    pub plane_h: usize,
    /// 解出来的样本，长度 = plane_w × plane_h，逐行存放
    pub samples: Vec<u8>,
}

// ---------------------------------------------------------------------------
// Huffman 规范码解码表
// ---------------------------------------------------------------------------

/// Huffman 解码表（“规范码 / canonical code”形式）。
///
/// JPEG 不把每个符号的码字直接写进文件，只写每个码长上有多少个码字
/// （`counts[0..16]`，对应码长 1..16）以及按码字递增顺序排列的符号表
/// （`symbols`）。解码器按下面三条公式离线展开即可，查表时 O(1)：
///
/// ```text
/// 码长 l 的最小码字：  min_code[l]  = (min_code[l-1] + counts[l-1]) << 1
/// 码长 l 的最大码字：  max_code[l]  = min_code[l] + counts[l] - 1
/// 该码长第一符号下标： value_ptr[l] = value_ptr[l-1] + counts[l-1]
/// ```
///
/// 下标 0 空置不用，这样 `min_code[1]` 就是长度为 1 的码的起点，
/// 写起来不用到处 `-1`。
#[derive(Debug)]
pub struct HuffmanDecoder {
    pub min_code: [i32; 17],
    pub max_code: [i32; 17],
    pub value_ptr: [usize; 17],
    pub symbols: Vec<u8>,
}

impl HuffmanDecoder {
    /// 由 DHT 段解析出来的原始表 (BITS/HUFFVAL) 展开成可直接查表的三个数组。
    ///
    /// 逐步推导（以 `counts = [0,1,0,...]` 为例）：
    ///
    /// ```text
    /// 初始：code = 0, k = 0
    /// l = 1: counts[0] = 0 → min_code[1]=0, value_ptr[1]=0
    ///        code = 0 + 0 = 0，max_code[1] = -1（区间为空）
    ///        code <<= 1 → 0
    /// l = 2: counts[1] = 1 → min_code[2]=0, value_ptr[2]=0
    ///        code = 0 + 1 = 1，max_code[2] = 0   ← 长度为 2 的码只有 "00"
    ///        code <<= 1 → 2
    /// l = 3: min_code[3] = 2 → 下一条码从 "010" 开始，符合规范码定义
    /// ```
    ///
    /// 规范码的约束：同长度码字从 min_code 连续递增，且短码在前（数值上更小），
    /// 所以“先比长度、再比数值”就能唯一确定一个码字，这正是解码循环的做法。
    pub fn from_table(t: &HuffmanTable) -> Self {
        let mut min_code: [i32; 17] = [0; 17];
        let mut max_code: [i32; 17] = [-1; 17];
        let mut value_ptr: [usize; 17] = [0; 17];

        for l in 1..=16usize {
            min_code[l] = (max_code[l - 1] + 1) << 1;
            max_code[l] = min_code[l] as i32 + t.counts[l - 1] as i32 - 1;
            value_ptr[l] = value_ptr[l - 1] + if l == 1 { 0 } else { t.counts[l - 2] as usize };
        }
        Self { min_code, max_code, value_ptr, symbols: t.symbols.clone() }
    }
}

// ---------------------------------------------------------------------------
// 位读取器
// ---------------------------------------------------------------------------

/// 从“已去掉 0xFF00 填充、已按 RST 切开”的字节串里按 MSB-first 读位。
///
/// 为什么要先去掉填充：JPEG 的熵编码数据是纯字节流，但字节 `0xFF`
/// 会和标记 (marker) 冲突，所以编码器遇到数据里的 `0xFF` 会补一个 `0x00`。
/// 解码时若读到 `FF 00`，真实数据只有一个 `FF`。
/// 本结构不处理这件事——它在 `split_entropy` 里一次性做完了，
/// 于是这里的 `bit()` 可以保持“傻白甜”的逐位读取，方便对照公式。
struct BitReader<'a> {
    data: &'a [u8],
    /// 下一个待取字节的下标
    byte_pos: usize,
    /// 当前字节里还剩几位没消费
    bit_cnt: u32,
    /// 当前正在消费的字节
    cur: u8,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            byte_pos: 0,
            bit_cnt: 0,
            cur: 0,
        }
    }

    /// 读 1 位。高位在前（MSB first），这是 JPEG 的规定。
    ///
    /// 取法：先把目标位“推到最高位再右移 7 位”，
    /// `(cur >> (bit_cnt - 1)) & 1`，其中 `bit_cnt` 从 8 递减到 1。
    #[inline]
    fn bit(&mut self) -> u32 {
        if self.bit_cnt == 0 {
            self.cur = if self.byte_pos < self.data.len() {
                self.data[self.byte_pos]
            } else {
                0
            };
            self.byte_pos += 1;
            self.bit_cnt = 8;
        }
        self.bit_cnt -= 1;
        (self.cur as u32 >> self.bit_cnt) & 1
    }

    /// 连续读 `n` 位并拼成一个无符号整数：
    /// `V = Σ_{i=0..n-1} b_i · 2^(n-1-i)`（先读到的位权最高）。
    #[inline]
    fn bits(&mut self, n: u32) -> u32 {
        let mut v = 0u32;
        for _ in 0..n {
            v = (v << 1) | self.bit();
        }
        v
    }

    /// 按 Huffman 表解出一个符号。
    ///
    /// 做法是逐位“走树”，但用数组代替真正的树：
    ///
    /// ```text
    /// code = 0
    /// for l = 1..=16:
    ///     code = (code << 1) | 下一位
    ///     if code <= max_code[l]:            ← 说明长度 l 的码字落在本区间
    ///         return symbols[value_ptr[l] + (code - min_code[l])]
    /// ```
    ///
    /// 为什么 `code <= max_code[l]` 就够？因为若长度 l 上码字区间是空的，
    /// `max_code[l] = min_code[l] - 1`，而此刻 `code >= min_code[l]` 恒成立
    /// （规范码保证短码数值小），条件自然为假。
    #[inline]
    fn huff(&mut self, t: &HuffmanDecoder) -> Result<u8, String> {
        let mut code: i32 = 0;
        for l in 1..=16 {
            code <<= 1;
            code |= self.bit() as i32;
            if code <= t.max_code[l] {
                let place = t.value_ptr[l] + code as usize - t.min_code[l] as usize;
                return t
                    .symbols
                    .get(place)
                    .copied()
                    .ok_or_else(|| { format!("Index out of range: len = {l}, index = {place}") });
            }
        }
        Err("Invalid Huffman code: We haven't found the correct code even after 16 bits".to_string())
    }

    /// 读 `s` 位并做“符号扩展”，得到 DC 差值或 AC 系数的真实值。
    ///
    /// JPEG 用“类别 + 幅值”表示有符号数（ITU T.81 F.2.2.1）：
    /// 正数直接写成 `s` 位二进制；负数写成它的“反码形式”，即
    /// `Vt = V + 2^s - 1`，所以解码时要反过来减。
    ///
    /// ```text
    /// 若 Vt < 2^(s-1)  →  V = Vt - 2^s + 1   （负数）
    /// 若 Vt >= 2^(s-1) →  V = Vt             （正数）
    /// ```
    ///
    /// 例：s = 3，区间 [-7, -4] ∪ [4, 7]；
    /// 收到 `010(2)` → 2 < 4 → 2 - 8 + 1 = -5 ✓
    /// 收到 `101(5)` → 5 >= 4 → 5 ✓
    #[inline]
    fn receive_extend(&mut self, s: u32) -> i32 {
        if s == 0 {
            return 0; // 零值不占位，类别 0 表示“差值为 0”
        }
        let vt = self.bits(s) as i32;
        let threshold = 1i32 << (s - 1);
        if vt < threshold {
            vt - (1 << s) + 1
        } else {
            vt
        }
    }
}

// ---------------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------------

impl JpegImage {
    /// 从 `file.rs` 解析出来的段列表，一路解码到“每个分量都有样本平面”。
    ///
    /// 关键点：多段 DQT / DHT 会合并；一个文件可以有多个 SOS（非交错扫描），
    /// 每个 SOS 后面紧跟一段 `EntropyData`。
    pub fn decode(file: JpegFile) -> Result<Self, String> {
        let mut frame: Option<FrameHeader> = None;
        let mut quant_tables: Vec<QuantizationTable> = Vec::new();
        let mut huffman_tables: Vec<HuffmanTable> = Vec::new();
        let mut restart_interval = 0usize;
        let mut scans: Vec<(ScanHeader, Vec<u8>)> = Vec::new();

        // 顺序扫描段列表，把需要的东西各归各位
        let mut segs = file.segments.into_iter();
        while let Some(seg) = segs.next() {
            match seg {
                JpegSegment::Sof(f) => frame = Some(f),
                JpegSegment::Dqt(t) => quant_tables.extend(t),
                JpegSegment::Dht(t) => huffman_tables.extend(t),
                JpegSegment::Dri(r) => restart_interval = r.interval as usize,
                JpegSegment::Sos(s) => {
                    // SOS 之后紧跟的就是本扫描的熵编码数据
                    let data = match segs.next() {
                        Some(JpegSegment::EntropyData(d)) => d,
                        _ => Vec::new(),
                    };
                    scans.push((s, data));
                }
                _ => {}
            }
        }

        let frame = frame.ok_or_else(|| "缺少 SOF 段，无法解码".to_string())?;
        if frame.marker != 0xc0 {
            return Err(format!(
                "当前只实现了 Baseline(SOF0) 解码，本文件帧类型是 0xFF{:02X}（SOF{}）",
                frame.marker,
                frame.marker & 0x0f
            ));
        }
        if frame.precision != 8 {
            return Err(format!("只支持 8bit 精度，本文件为 {}bit", frame.precision));
        }

        let width = frame.width as usize;
        let height = frame.height as usize;
        if width == 0 || height == 0 {
            return Err("图像宽或高为 0".to_string());
        }

        // hmax / vmax：所有分量采样因子的最大值，例如 4:2:0 → 2,2
        let hmax = frame.components.iter().map(|c| c.h_sampling).max().unwrap_or(1);
        let vmax = frame.components.iter().map(|c| c.v_sampling).max().unwrap_or(1);
        if hmax == 0 || vmax == 0 {
            return Err("采样因子为 0，文件损坏".to_string());
        }

        // MCU（最小编码单元）网格：一个 MCU 覆盖 8*hmax × 8*vmax 个像素
        let mcus_x = ceil_div(width, 8 * hmax as usize);
        let mcus_y = ceil_div(height, 8 * vmax as usize);

        // 为每个分量分配样本平面。注意平面尺寸要“向上补到 8 的整数倍”，
        // 因为熵数据里 MCU 是整块编码的，右/下边缘可能超出图像边界。
        let mut components = Vec::with_capacity(frame.components.len());
        for fc in frame.components {
            let h = fc.h_sampling as usize;
            let v = fc.v_sampling as usize;
            if h == 0 || v == 0 || h > 4 || v > 4 {
                return Err(format!("非法采样因子 {}x{}", h, v));
            }

            // 分量样本尺寸 = ⌈图像尺寸 × 本分量采样因子 / 最大采样因子⌉
            let cw = ceil_div(width * h, hmax as usize);
            let ch = ceil_div(height * v, vmax as usize);

            // 交错扫描时，本分量在每个 MCU 里占 h×v 块
            let blocks_x = mcus_x * h;
            let blocks_y = mcus_y * v;
            let plane_w = blocks_x * 8;
            let plane_h = blocks_y * 8;

            components.push(JpegComponent {
                id: fc.id,
                h_sampling: fc.h_sampling,
                v_sampling: fc.v_sampling,
                quant_table_id: fc.quant_table_id,
                width: cw,
                height: ch,
                blocks_x,
                blocks_y,
                plane_w,
                plane_h,
                samples: vec![0u8; plane_w * plane_h],
            });
        }

        let mut img = Self {
            width,
            height,
            components,
            quant_tables,
            huffman_tables,
            restart_interval,
            max_h: hmax,
            max_v: vmax,
        };

        img.decode_scans(&scans)?;
        Ok(img)
    }

    /// 依次解码每个扫描（SOS）。
    fn decode_scans(&mut self, scans: &[(ScanHeader, Vec<u8>)]) -> Result<(), String> {
        // 先建 Huffman 解码表。索引用 class*4 + id：
        // class 0 = DC 表，class 1 = AC 表，id 0..3（来自 DHT 的高/低半字节）
        let mut decoders: Vec<Option<HuffmanDecoder>> = (0..8).map(|_| None).collect();
        for t in &self.huffman_tables {
            if t.table_class > 1 || t.table_id > 3 {
                continue;
            }
            decoders[t.table_class as usize * 4 + t.table_id as usize] =
                Some(HuffmanDecoder::from_table(t));
        }

        // IDCT 的余弦基表只跟 (x,u) 有关，整幅图共用一份
        let basis = build_idct_basis();

        for (scan, entropy) in scans {
            decode_scan(
                &mut self.components,
                &decoders,
                &self.quant_tables,
                self.restart_interval,
                self.width,
                self.height,
                scan,
                entropy,
                &basis,
            )?;
        }
        Ok(())
    }

    /// 把已经解码好的各分量平面合成 RGB。
    ///
    /// 两步：
    /// 1. **上采样**：色度分量分辨率比亮度低，取最近邻放大
    ///    `sx = x · h / hmax`，`sy = y · v / vmax`（整数除法即向下取整）。
    /// 2. **色彩转换**（JFIF / ITU-R BT.601 的 YCbCr → RGB）：
    ///
    /// ```text
    /// R = Y + 1.402  · (Cr - 128)
    /// G = Y - 0.344136·(Cb - 128) - 0.714136·(Cr - 128)
    /// B = Y + 1.772  · (Cb - 128)
    /// ```
    ///
    /// 其中 `Cr - 128`、`Cb - 128` 是“去偏置”，因为色度以 128 表示“无色”。
    pub fn to_rgb(&self) -> Result<Vec<u8>, String> {
        let hmax = self.max_h.max(1) as usize;
        let vmax = self.max_v.max(1) as usize;
        let ncomp = self.components.len();

        if ncomp != 1 && ncomp != 3 {
            return Err(format!("暂不支持 {ncomp} 个分量的色彩空间"));
        }

        let mut rgb = vec![0u8; self.width * self.height * 3];
        for y in 0..self.height {
            for x in 0..self.width {
                let o = (y * self.width + x) * 3;

                if ncomp == 1 {
                    // 灰度图：Y 直接就是 R=G=B
                    let g = self.sample(0, x, y, hmax, vmax);
                    rgb[o] = g;
                    rgb[o + 1] = g;
                    rgb[o + 2] = g;
                } else {
                    let yy = self.sample(0, x, y, hmax, vmax) as f32;
                    let cb = self.sample(1, x, y, hmax, vmax) as f32 - 128.0;
                    let cr = self.sample(2, x, y, hmax, vmax) as f32 - 128.0;

                    let r = yy + 1.402 * cr;
                    let g = yy - 0.344_136 * cb - 0.714_136 * cr;
                    let b = yy + 1.772 * cb;

                    // 浮点结果四舍五入后夹到 [0,255]（越界是色度上采样 + 量化误差的常态）
                    rgb[o] = r.round().clamp(0.0, 255.0) as u8;
                    rgb[o + 1] = g.round().clamp(0.0, 255.0) as u8;
                    rgb[o + 2] = b.round().clamp(0.0, 255.0) as u8;
                }
            }
        }
        Ok(rgb)
    }

    /// 取第 `ci` 个分量在 (x, y) 处的样本，内部完成最近邻上采样。
    #[inline]
    fn sample(&self, ci: usize, x: usize, y: usize, hmax: usize, vmax: usize) -> u8 {
        let c = &self.components[ci];
        let sx = (x * c.h_sampling as usize / hmax).min(c.width.saturating_sub(1));
        let sy = (y * c.v_sampling as usize / vmax).min(c.height.saturating_sub(1));
        c.samples[sy * c.plane_w + sx]
    }

    /// 写成 PPM P6（二进制 RGB）。
    ///
    /// 文件结构：
    /// ```text
    /// P6\n
    /// <宽> <高>\n
    /// 255\n
    /// <R G B R G B ...>   每个像素 3 字节，逐行、从上到下
    /// ```
    /// 前 4 行是 ASCII 头，之后是裸字节；`255` 表示每通道最大值，
    /// 所以 3 字节正好对应 0..255。
    pub fn write_ppm<P: AsRef<Path>>(&self, path: P) -> Result<(), String> {
        let rgb = self.to_rgb()?;
        let file = File::create(path.as_ref())
            .map_err(|e| format!("创建 {} 失败: {e}", path.as_ref().display()))?;
        let mut w = BufWriter::new(file);

        write!(w, "P6\n{} {}\n255\n", self.width, self.height)
            .map_err(|e| format!("写 PPM 头失败: {e}"))?;
        w.write_all(&rgb).map_err(|e| format!("写像素失败: {e}"))?;
        w.flush().map_err(|e| format!("flush 失败: {e}"))?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// 扫描解码（熵解码 + 反量化 + IDCT）
// ---------------------------------------------------------------------------

/// 解码一个扫描（一个 SOS 段 + 它后面的熵编码数据）。
///
/// 参数说明：
/// * `comps`    —— 全部分量（可能只用到其中几个，取决于 SOS 里写了谁）
/// * `huff`     —— Huffman 解码表，下标 `class*4 + id`
/// * `qts`      —— 量化表
/// * `scan`     —— SOS 头：哪些分量、用哪张 DC/AC 表
/// * `entropy`  —— 原始熵数据（含 0xFF00 填充和 RST 标记）
#[allow(clippy::too_many_arguments)]
fn decode_scan(
    comps: &mut [JpegComponent],
    huff: &[Option<HuffmanDecoder>],
    qts: &[QuantizationTable],
    restart_interval: usize,
    img_w: usize,
    img_h: usize,
    scan: &ScanHeader,
    entropy: &[u8],
    basis: &[[f32; 8]; 8],
) -> Result<(), String> {
    let ns = scan.components.len();
    if ns == 0 {
        return Err("SOS 段没有分量".to_string());
    }

    // SOS 里的分量 id → components 下标
    let mut scomp: Vec<usize> = Vec::with_capacity(ns);
    for sc in &scan.components {
        let idx = comps
            .iter()
            .position(|c| c.id == sc.component_id)
            .ok_or_else(|| format!("SOS 引用了不存在的分量 id={}", sc.component_id))?;
        scomp.push(idx);
    }

    let hmax = comps.iter().map(|c| c.h_sampling as usize).max().unwrap_or(1);
    let vmax = comps.iter().map(|c| c.v_sampling as usize).max().unwrap_or(1);

    // 交错 vs 非交错：
    // * ns > 1：一个 MCU 里装 h×v 个某分量的块，MCU 网格按整幅图算
    // * ns == 1：一个 MCU 就是一个块，网格退化成该分量自己的块网格（ceil(宽/8) × ceil(高/8)）
    let interleaved = ns > 1;
    let (grid_x, grid_y) = if interleaved {
        (ceil_div(img_w, 8 * hmax), ceil_div(img_h, 8 * vmax))
    } else {
        let c = &comps[scomp[0]];
        (ceil_div(c.width, 8), ceil_div(c.height, 8))
    };

    // 预先整理“本扫描要用到的分量”，把量化表 / Huffman 表 / 每 MCU 块数
    // 一次查好，避免在内层循环里反复查找。
    struct ScanComp<'a> {
        ci: usize,        // 在 components 中的下标
        bpm_x: usize,     // 一个 MCU 内横向块数
        bpm_y: usize,     // 一个 MCU 内纵向块数
        dc: &'a HuffmanDecoder,
        ac: &'a HuffmanDecoder,
        qt: [u16; 64],    // 量化表（自然顺序，已拷贝，避免借用冲突）
    }

    let mut plan: Vec<ScanComp> = Vec::with_capacity(ns);
    for si in 0..ns {
        let ci = scomp[si];
        let sc = &scan.components[si];

        let (bpm_x, bpm_y) = if interleaved {
            (
                comps[ci].h_sampling as usize,
                comps[ci].v_sampling as usize,
            )
        } else {
            (1, 1)
        };

        let qt = qts
            .iter()
            .find(|q| q.id == comps[ci].quant_table_id)
            .map(|q| q.values)
            .ok_or_else(|| format!("缺少量化表 id={}", comps[ci].quant_table_id))?;

        // Huffman 表：DC 在 class 0，AC 在 class 1
        let dc = huff
            .get(sc.dc_table_id as usize)
            .and_then(|o| o.as_ref())
            .ok_or_else(|| format!("缺少 DC Huffman 表 id={}", sc.dc_table_id))?;
        let ac = huff
            .get(4 + sc.ac_table_id as usize)
            .and_then(|o| o.as_ref())
            .ok_or_else(|| format!("缺少 AC Huffman 表 id={}", sc.ac_table_id))?;

        plan.push(ScanComp { ci, bpm_x, bpm_y, dc, ac, qt });
    }

    // 熵数据按 RST 切段；无 RST 时只有 1 段
    let chunks = split_entropy(entropy);
    let mut br = BitReader::new(chunks[0].as_slice());
    let mut chunk_idx = 0usize;

    // DC 预测值：每个分量各自维护一个，遇到 RST 或新扫描时清零
    let mut dc_pred = vec![0i32; comps.len()];

    let mut mcu = 0usize;
    for my in 0..grid_y {
        for mx in 0..grid_x {
            // 重启间隔：每解码 interval 个 MCU，编码器插入一个 RSTn 标记，
            // 同时 DC 预测值归零（这样丢一个 MCU 的损坏不会传染到后面）
            if restart_interval > 0 && mcu > 0 && mcu % restart_interval == 0 {
                chunk_idx += 1;
                if chunk_idx < chunks.len() {
                    br = BitReader::new(chunks[chunk_idx].as_slice());
                }
                dc_pred.iter_mut().for_each(|p| *p = 0);
            }
            mcu += 1;

            for p in &plan {
                for v in 0..p.bpm_y {
                    for h in 0..p.bpm_x {
                        let bx = mx * p.bpm_x + h;
                        let by = my * p.bpm_y + v;

                        decode_block(
                            &mut comps[p.ci],
                            p.dc,
                            p.ac,
                            &p.qt,
                            &mut br,
                            &mut dc_pred[p.ci],
                            bx,
                            by,
                            basis,
                        )?;
                    }
                }
            }
        }
    }

    Ok(())
}

/// 解码一个 8×8 块：Huffman 熵解码 → 反量化 → IDCT → 写回样本平面。
///
/// 熵编码里的系数按 **zig-zag** 顺序排列（低频在前，沿着“之”字形走），
/// 原因是低频系数大、高频多为 0，这样连续出现 0 的概率高，用 `ZRL` 就能一次跳过 16 个零。
/// `ZIGZAG_TO_NATURAL[k]` 就是“zig-zag 第 k 位 → 自然顺序下标”的映射表。
fn decode_block(
    comp: &mut JpegComponent,
    dc_tbl: &HuffmanDecoder,
    ac_tbl: &HuffmanDecoder,
    qt: &[u16; 64],
    br: &mut BitReader,
    pred: &mut i32,
    bx: usize,
    by: usize,
    basis: &[[f32; 8]; 8],
) -> Result<(), String> {
    let mut coef = [0f32; 64];

    // ---------- ① DC 系数（差分编码） ----------
    // JPEG 不对 DC 本身编码，而是对“与上一块 DC 的差”编码：
    //     DC_k = DC_{k-1} + DIFF
    // 第一个块的 DC_{k-1} 取 0（或 128*8，见下文 IDCT 里的电平搬移）。
    let s = br.huff(dc_tbl)? as u32;
    let diff = br.receive_extend(s);
    *pred += diff;

    // 反量化：F(0,0) = DC × Q(0,0)
    coef[0] = *pred as f32 * qt[0] as f32;

    // ---------- ② AC 系数（游程编码） ----------
    // 每个符号一个字节 (R, S)：高 4 位 R = 前面有多少个 0，低 4 位 S = 幅值占几位
    //   S == 0 且 R == 0  → EOB，本块剩下的系数全是 0，结束
    //   S == 0 且 R == 15 → ZRL，跳过 16 个 0（即 0xF0）
    //   S != 0            → 先跳 R 个 0，再读 S 位幅值放到该位置
    let mut k = 1usize; // 从 zig-zag 第 1 位开始（第 0 位是 DC）
    while k < 64 {
        let rs = br.huff(ac_tbl)?;
        let r = (rs >> 4) as usize;
        let s = (rs & 0x0f) as u32;

        if s == 0 {
            if r == 15 {
                k += 16; // ZRL：16 个零
                continue;
            }
            break; // EOB：后面全零
        }

        k += r;
        if k > 63 {
            break; // 数据异常，直接收尾
        }

        let v = br.receive_extend(s);
        let n = ZIGZAG_TO_NATURAL[k]; // zig-zag → 自然顺序
        // 反量化：F(u,v) = 量化系数 × Q(u,v)
        coef[n] = v as f32 * qt[n] as f32;
        k += 1;
    }

    // ---------- ③ IDCT + 电平搬移 ----------
    let mut block = [0u8; 64];
    idct_8x8(&coef, basis, &mut block);

    // ---------- ④ 写回样本平面 ----------
    for y in 0..8 {
        let dst = (by * 8 + y) * comp.plane_w + bx * 8;
        if dst + 8 <= comp.samples.len() {
            comp.samples[dst..dst + 8].copy_from_slice(&block[y * 8..y * 8 + 8]);
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// IDCT
// ---------------------------------------------------------------------------

/// 预计算 IDCT 的余弦基表。
///
/// 二维 IDCT 的定义（ITU T.81 A.3.2）：
///
/// ```text
///           1    7   7
/// f(x,y) = ─── · Σ   Σ   C(u)·C(v)·F(u,v)·cos((2x+1)uπ/16)·cos((2y+1)vπ/16)
///           4   u=0 v=0
///
/// C(0) = 1/√2 ,  C(u) = 1 (u > 0)
/// ```
///
/// 直接算一个块要 64×64 次乘法。利用“可分离性”可以拆成两次一维变换，
/// 把代价降到 2×8×8×8 = 1024 次：
///
/// ```text
/// 第一步（每行 v 做水平变换）：  t(v,x) = Σ_u F(v,u) · basis[x][u]
/// 第二步（每列 x 做垂直变换）：  f(x,y) = Σ_v t(v,x) · basis[y][v]
///
/// basis[x][u] = C(u)/2 · cos((2x+1)uπ/16)
/// ```
///
/// 两个 1/2 相乘正好是公式里的 1/4，所以 `basis` 里带 `C(u)/2`。
fn build_idct_basis() -> [[f32; 8]; 8] {
    let mut b = [[0f32; 8]; 8];
    for x in 0..8usize {
        for u in 0..8usize {
            let cu = if u == 0 {
                std::f32::consts::FRAC_1_SQRT_2 // 1/√2 ≈ 0.70710678
            } else {
                1.0
            };
            let angle = (2 * x + 1) as f32 * u as f32 * std::f32::consts::PI / 16.0;
            b[x][u] = 0.5 * cu * angle.cos();
        }
    }
    b
}

/// 对一块 8×8 的（已反量化的）DCT 系数做二维 IDCT，输出 0..255 的样本。
///
/// 两个细节：
/// 1. **电平搬移**：JPEG 编码前把每个样本减了 128（为了 DCT 后 DC 更集中）。
///    所以 IDCT 的结果要 `+128` 再取整夹紧：`s = clamp(round(f(x,y) + 128), 0, 255)`。
/// 2. **全零行捷径**：某一行的 AC 全为 0 时，
///    `t(v,x) = F(v,0)·basis[x][0]` 与 x 无关，只算一次即可。
///    实拍图片里这种行非常多（尤其 Cb/Cr），是廉价的加速。
fn idct_8x8(coef: &[f32; 64], basis: &[[f32; 8]; 8], out: &mut [u8; 64]) {
    // ---- 第一步：行变换（水平方向，对每个 v 的 8 个 u 求和）----
    let mut tmp = [[0f32; 8]; 8];
    for v in 0..8usize {
        let row = &coef[v * 8..v * 8 + 8];
        if row[1..].iter().all(|&c| c == 0.0) {
            // 只剩 DC 项：t(v,x) = F(v,0)·C(0)/2，C(0)/2 = 1/(2√2)
            let t = row[0] * basis[0][0];
            tmp[v] = [t; 8];
            continue;
        }
        for x in 0..8usize {
            let mut s = 0f32;
            for u in 0..8usize {
                s += row[u] * basis[x][u];
            }
            tmp[v][x] = s;
        }
    }

    // ---- 第二步：列变换（垂直方向，对每个 x 的 8 个 v 求和）----
    for x in 0..8usize {
        for y in 0..8usize {
            let mut s = 0f32;
            for v in 0..8usize {
                s += tmp[v][x] * basis[y][v];
            }
            // 电平搬移 + 四舍五入 + 夹紧
            out[y * 8 + x] = (s + 128.0).round().clamp(0.0, 255.0) as u8;
        }
    }
}

// ---------------------------------------------------------------------------
// 熵数据预处理
// ---------------------------------------------------------------------------

/// 把原始熵数据拆成“无填充、按 RST 分段”的干净字节流。
///
/// 需处理两种情况（见 ITU T.81 B.1.1.5 / B.2.1）：
/// * **字节填充**：数据中真正的 `0xFF` 会被写成 `FF 00`，读时还原成 `FF`
/// * **重启标记**：`FF D0`..`FF D7` 不是数据，而是“这里可以重新同步”的分隔符，
///   每遇到一个就切开，后面解码时对应一次 DC 预测重置
///
/// 返回值至少有 1 段（哪怕整段为空）。
fn split_entropy(entropy: &[u8]) -> Vec<Vec<u8>> {
    let mut chunks: Vec<Vec<u8>> = Vec::new();
    let mut cur: Vec<u8> = Vec::new();
    let mut i = 0usize;

    while i < entropy.len() {
        if entropy[i] != 0xff {
            cur.push(entropy[i]);
            i += 1;
            continue;
        }

        match entropy.get(i + 1) {
            Some(&0x00) => {
                // 填充字节：真实数据是一个 0xFF
                cur.push(0xff);
                i += 2;
            }
            Some(&m) if (0xd0..=0xd7).contains(&m) => {
                // RSTn：当前段结束
                chunks.push(std::mem::take(&mut cur));
                i += 2;
            }
            Some(_) => {
                // 理论上解析阶段已经切开，这里只做容错
                i += 1;
            }
            None => i += 1,
        }
    }

    chunks.push(cur);
    chunks
}

/// 向上取整除法：⌈a / b⌉ = (a + b - 1) / b（整数除法向下取整）。
#[inline]
fn ceil_div(a: usize, b: usize) -> usize {
    (a + b - 1) / b
}

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------

#[test]
fn test_decode_sample() {
    use crate::jpeg::file::JpegRaw;

    let data = std::fs::read("./bin/20210617054007202.jpg").unwrap();
    let mut raw = JpegRaw::new(&data);
    let jpeg = raw.parse();

    let img = JpegImage::decode(jpeg).expect("解码失败");
    assert_eq!((img.width, img.height), (3840, 2400));
    assert_eq!(img.components.len(), 3);
    // 4:2:0：亮度 3840×2400，色度 1920×1200
    assert_eq!(img.components[0].width, 3840);
    assert_eq!(img.components[1].width, 1920);

    let rgb = img.to_rgb().unwrap();
    assert_eq!(rgb.len(), 3840 * 2400 * 3);
    println!("首像素 RGB = {:?}", &rgb[0..3]);
}
