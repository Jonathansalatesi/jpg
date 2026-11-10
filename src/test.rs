mod example {
    use std::{collections::HashMap, println, unreachable, usize, vec};
    use crate::test::{encode::{extend, series}, jpg_huffman::Coder};
    use super::jpg_huffman;
    use crate::jpeg::file::ZIGZAG_TO_NATURAL;

    const QUANTIZATION: [[f64; 8]; 8] = [
        [16.0, 11.0, 10.0, 16.0, 24.0, 40.0, 51.0, 61.0],
        [12.0, 12.0, 14.0, 19.0, 26.0, 58.0, 60.0, 55.0],
        [14.0, 13.0, 16.0, 24.0, 40.0, 57.0, 69.0, 56.0],
        [14.0, 17.0, 22.0, 29.0, 51.0, 87.0, 80.0, 62.0],
        [18.0, 22.0, 37.0, 56.0, 68.0, 109.0, 103.0, 77.0],
        [24.0, 35.0, 55.0, 64.0, 81.0, 104.0, 113.0, 92.0],
        [49.0, 64.0, 78.0, 87.0, 103.0, 121.0, 120.0, 101.0],
        [72.0, 92.0, 95.0, 98.0, 112.0, 100.0, 103.0, 99.0]
    ];
    const WIDTH: usize = 1920;
    const HEIGHT: usize = 1080;

    fn photo() -> Vec<Vec<[u8; 1920]>> {
        let mut figure = vec![];

        let r = vec![[0xff; 1920]; 1080];
        let mut g = vec![[0xff; 1920]; 1080];
        let mut b = vec![[0xff; 1920]; 1080];
        for i in 0..1080 {
            for j in 0..1920 {
                if i * i + j * j <= 500 * 500 {
                    g[i][j] = 0;
                    b[i][j] = 0;
                }
            }
        }
        figure.push(r);
        figure.push(g);
        figure.push(b);
        figure
    }

    fn y_cb_cr(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
        // Y  =  0.299·R + 0.587·G + 0.114·B
        // Cb = -0.168736·R - 0.331264·G + 0.5·B + 128
        // Cr =  0.5·R - 0.418688·G - 0.081312·B + 128
        let r = r as f64;
        let g = g as f64;
        let b = b as f64;

        let y = 0.299 * r + 0.587 * g + 0.114 * b;
        let cb = -0.168736 * r - 0.331264 * g + 0.5 * b + 128.;
        let cr = 0.5 * r - 0.418688 * g - 0.081312 * b + 128.;
        (y as u8, cb as u8, cr as u8)
    }

    pub fn serialize() -> Jpeg {
        let fig = photo();
        let mut ycs = vec![vec![[0_u8; WIDTH]; HEIGHT]; 3];
        for i in 0..HEIGHT {
            for j in 0..WIDTH {
                let (y, cb, cr) = y_cb_cr(fig[0][i][j], fig[1][i][j], fig[2][i][j]);
                ycs[0][i][j] = y;
                ycs[1][i][j] = cb;
                ycs[2][i][j] = cr;
            }
        }

        let (y_mat, cb_mat, cr_mat) = (&ycs[0], &ycs[1], &ycs[2]);
        let mut cbs = vec![[0_u8; WIDTH / 2]; HEIGHT / 2];
        let mut crs = vec![[0_u8; WIDTH / 2]; HEIGHT / 2];
        for i in 0..HEIGHT / 2 {
            for j in 0..WIDTH / 2 {
                cbs[i][j] = cb_mat[i * 2][j * 2];
                crs[i][j] = cr_mat[i * 2][j * 2];
            }
        }

        let mut y_cubes = vec![vec![[[0_u8; 8]; 8]; WIDTH.div_ceil(16) * 2]; HEIGHT.div_ceil(16) * 2];
        let mut cb_cubes = vec![vec![[[0_u8; 8]; 8]; (WIDTH / 2).div_ceil(8)]; (HEIGHT / 2).div_ceil(8)];
        let mut cr_cubes = vec![vec![[[0_u8; 8]; 8]; (WIDTH / 2).div_ceil(8)]; (HEIGHT / 2).div_ceil(8)];
        for i in 0..y_cubes.len() {
            for j in 0..y_cubes[0].len() {
                for ii in 0..8 {
                    for jj in 0..8 {
                        let y = (i * 8 + ii).min(HEIGHT - 1);
                        let x = (j * 8 + jj).min(WIDTH - 1);
                        y_cubes[i][j][ii][jj] = y_mat[y][x];
                    }
                }
            }
        }

        for i in 0..cb_cubes.len() {
            for j in 0..cb_cubes[0].len() {
                for ii in 0..8 {
                    for jj in 0..8 {
                        let y = (i * 8 + ii).min(HEIGHT / 2 - 1);
                        let x = (j * 8 + jj).min(WIDTH / 2 - 1);
                        cb_cubes[i][j][ii][jj] = cbs[y][x];
                        cr_cubes[i][j][ii][jj] = crs[y][x];
                    }
                }
            }
        }

        let mut y_series = vec![vec![vec![]; WIDTH.div_ceil(16) * 2]; HEIGHT.div_ceil(16) * 2];
        let mut cb_series = vec![vec![vec![]; (WIDTH / 2).div_ceil(8)]; (HEIGHT / 2).div_ceil(8)];
        let mut cr_series = vec![vec![vec![]; (WIDTH / 2).div_ceil(8)]; (HEIGHT / 2).div_ceil(8)];
        for i in 0..y_cubes.len() {
            for j in 0..y_cubes[0].len() {
                y_series[i][j] = series(&y_cubes[i][j], &QUANTIZATION);
            }
        }

        for i in 0..cb_cubes.len() {
            for j in 0..cb_cubes[0].len() {
                cb_series[i][j] = series(&cb_cubes[i][j], &QUANTIZATION);
                cr_series[i][j] = series(&cr_cubes[i][j], &QUANTIZATION);
            }
        }

        let (mut last_dc, mut tmp_dc) = (0, 0);
        for i in 0..y_cubes.len() {
            for j in 0..y_cubes[0].len() {
                tmp_dc = y_series[i][j][0].1;
                let (v, s) = extend(tmp_dc - last_dc);
                y_series[i][j][0].0 = s as u8;
                y_series[i][j][0].1 = v as i64;
                last_dc = tmp_dc;
            }
        }

        let (mut last_dc, mut tmp_dc) = (0, 0);
        for i in 0..cb_cubes.len() {
            for j in 0..cb_cubes[0].len() {
                tmp_dc = cb_series[i][j][0].1;
                let (v, s) = extend(tmp_dc - last_dc);
                cb_series[i][j][0].0 = s as u8;
                cb_series[i][j][0].1 = v as i64;
                last_dc = tmp_dc;
            }
        }

        let (mut last_dc, mut tmp_dc) = (0, 0);
        for i in 0..cb_cubes.len() {
            for j in 0..cb_cubes[0].len() {
                tmp_dc = cr_series[i][j][0].1;
                let (v, s) = extend(tmp_dc - last_dc);
                cr_series[i][j][0].0 = s as u8;
                cr_series[i][j][0].1 = v as i64;
                last_dc = tmp_dc;
            }
        }

        let mut dch: HashMap<u8, u64> = HashMap::new();
        let mut ach: HashMap<u8, u64> = HashMap::new();
        for i in 0..y_cubes.len() {
            for j in 0..y_cubes[0].len() {
                for (idx, &elem) in y_series[i][j].iter().enumerate() {
                    if idx == 0 {
                        dch.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    } else {
                        ach.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    }
                }
            }
        }

        for i in 0..cb_cubes.len() {
            for j in 0..cb_cubes[0].len() {
                for (idx, &elem) in cb_series[i][j].iter().enumerate() {
                    if idx == 0 {
                        dch.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    } else {
                        ach.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    }
                }
            }
        }

        for i in 0..cr_cubes.len() {
            for j in 0..cr_cubes[0].len() {
                for (idx, &elem) in cr_series[i][j].iter().enumerate() {
                    if idx == 0 {
                        dch.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    } else {
                        ach.entry(elem.0).and_modify(|counter| *counter += 1).or_insert(1);
                    }
                }
            }
        }

        let mut dc_symbols = dch.iter().map(|v| (*v.0, *v.1)).collect::<Vec<(u8, u64)>>();
        let mut ac_symbols = ach.iter().map(|v| (*v.0, *v.1)).collect::<Vec<(u8, u64)>>();
        dc_symbols.push((0, 0));
        ac_symbols.push((0, 0));
        let (mut dc_marks, dc_coder, mut dc_counts) = {
            let counts = jpg_huffman::compress(&dc_symbols);
            dc_symbols.sort_by_key(|(_, b)| std::cmp::Reverse(*b));
            let marks: Vec<u8> = dc_symbols.iter().map(|(a, _)| *a).collect();
            let mut coder = jpg_huffman::Coder::new();
            coder.create(&counts);

            (marks, coder, counts)
        };
        dc_marks.pop();
        for i in (0..dc_counts.len()).rev() {
            if dc_counts[i] > 0 {
                dc_counts[i] -= 1;
                break;
            }
        }

        let (mut ac_marks, ac_coder, mut ac_counts) = {
            let counts = jpg_huffman::compress(&ac_symbols);
            ac_symbols.sort_by_key(|(_, b)| std::cmp::Reverse(*b));
            let marks: Vec<u8> = ac_symbols.iter().map(|(a, _)| *a).collect();
            let mut coder = jpg_huffman::Coder::new();
            coder.create(&counts);

            (marks, coder, counts)
        };
        ac_marks.pop();
        for i in (0..ac_counts.len()).rev() {
            if ac_counts[i] > 0 {
                ac_counts[i] -= 1;
                break;
            }
        }

        let mut binary = Binary::new();
        for i in 0..cr_cubes.len() {
            for j in 0..cr_cubes[0].len() {
                let offs = [[0, 0], [0, 1], [1, 0], [1, 1]];
                for [di, dj] in offs {
                    binary.entropy(&y_series[i * 2 + di][j * 2 + dj], &ac_marks, &ac_coder, &dc_marks, &dc_coder);
                }
                binary.entropy(&cb_series[i][j], &ac_marks, &ac_coder, &dc_marks, &dc_coder);
                binary.entropy(&cr_series[i][j], &ac_marks, &ac_coder, &dc_marks, &dc_coder);
            }
        }
        binary.finished();
        let ent = binary.filter();
        
        let mut jpeg = Jpeg::new();
        jpeg.dqt();
        jpeg.sof0();
        jpeg.dht(&ac_marks, &ac_counts[1..=16], &dc_marks, &dc_counts[1..=16]);
        jpeg.sos();
        jpeg.extend(&ent);
        jpeg.end();

        jpeg
    }

    fn search_for_code(symbol: u8, marks: &[u8], coder: &Coder) -> (i64, usize) {
        for i in 0..marks.len() {
            if symbol == marks[i] {
                for j in 0..coder.ptr_addr.len() {
                    if i < coder.ptr_addr[j] {
                        return (coder.min_code[j - 1] + i as i64 - coder.ptr_addr[j - 1] as i64, j - 1);
                    }
                }
                let inx = coder.min_code.len() - 1;
                return (coder.min_code[inx] + i as i64 - coder.ptr_addr[inx] as i64, inx);
            }
        }
        unreachable!()
    }

    #[derive(Debug)]
    struct Binary {
        data: Vec<u8>,
        pos: usize,         // i-th u8
        offset: usize,      // i-th bit in a u8, 0-7
    }

    impl Binary {
        pub fn new() -> Self {
            Self { data: vec![0], pos: 0, offset: 0 }
        }

        pub fn bit(&mut self, b: i64) {
            if b == 0 {
                self.data[self.pos] &= !(1 << (7 - self.offset));
            } else {
                self.data[self.pos] |= 1 << (7 - self.offset);
            }
            self.offset += 1;
            if self.offset >= 8 {
                self.offset = 0;
                self.pos += 1;
                while self.data.len() < self.pos + 1 {
                    self.data.push(0xff);
                }
            }
        }

        pub fn bits(&mut self, b: i64, s: usize) {
            for i in (0..s).rev() {
                self.bit(b & (1 << i));
            }
        }

        pub fn entropy(
            &mut self,
            data: &[(u8, i64)],
            ac_marks: &[u8],
            ac_coder: &Coder,
            dc_marks: &[u8],
            dc_coder: &Coder
        ) {
            for (idx, &elem) in data.iter().enumerate() {
                if idx == 0 {
                    let (v, s) = search_for_code(elem.0, dc_marks, dc_coder);
                    self.bits(v, s);
                    self.bits(elem.1, elem.0 as usize);
                } else {
                    let (v, s) = search_for_code(elem.0, ac_marks, ac_coder);
                    self.bits(v, s);
                    if elem.0 & 0x0f > 0 {
                        self.bits(elem.1, (elem.0 & 0x0f) as usize);
                    }
                }
            }
        }

        pub fn finished(&mut self) {
            if self.offset > 0 {
                self.data[self.pos] |= (1 << (8 - self.offset)) - 1;
                self.offset = 0;
                self.pos += 1;
            }
        }

        pub fn filter(&self) -> Vec<u8> {
            let mut stuffed = Vec::with_capacity(self.data.len());
            for &b in &self.data {
                stuffed.push(b);
                if b == 0xff {
                    stuffed.push(0x00);
                }
            }
            stuffed
        }
    }

    // SOI (FFD8)
    // APP0 (可选)
    // DQT (FFDB)   ← 量化表
    // SOF0 (FFC0)  ← 宽高、采样因子、分量
    // DHT (FFC4)   ← BITS + HUFFVAL（DC 和 AC 各一套）
    // SOS (FFDA)   ← 扫描头
    // [熵编码数据]  ← 你现在输出的
    // EOI (FFD9)
    pub struct Jpeg {
        pub data: Vec<u8>,
    }

    impl Jpeg {
        pub fn new() -> Self {
            Self { data: vec![0xff, 0xd8] }
        }

        // FF DB           marker
        // Lq (2 字节)     段长度 = 2 + 1 + 64 = 67
        // Pq/Tq (1 字节)  高4位 Pq（精度），低4位 Tq（表ID）
        // Qk (64 字节)    量化表，按 zigzag 顺序
        pub fn dqt(&mut self) {
            let id = 0u8;
            let precision = 0u8;

            self.data.push(0xff);
            self.data.push(0xdb);
            self.data.push(0x00);
            self.data.push(67);                        // 2 + 1 + 64
            self.data.push((precision << 4) | id);     // Pq/Tq 合成 1 字节

            for i in 0..64 {
                let idx = ZIGZAG_TO_NATURAL[i];
                let r = idx / 8;
                let c = idx % 8;
                self.data.push(QUANTIZATION[r][c] as u8);
            }
        }

        // FF C0           marker
        // Lf (2 字节)     段长度 = 2 + 1 + 2 + 2 + 1 + 3*Nf = 17（Nf=3 时）
        // P (1 字节)      精度
        // Y (2 字节)      高度
        // X (2 字节)      宽度
        // Nf (1 字节)     分量数 ← 你漏了
        // 每个分量 3 字节：C (1) + HV (1) + Tq (1)
        pub fn sof0(&mut self) {
            self.data.push(0xff);
            self.data.push(0xc0);
            self.data.push(0x00);
            self.data.push(17);                // 2 + 1 + 2 + 2 + 1 + 9
            self.data.push(8);                 // precision
            self.data.extend(&1080u16.to_be_bytes());
            self.data.extend(&1920u16.to_be_bytes());
            self.data.push(3);                 // Nf = 3

            // Y: H=2, V=2, Tq=0
            self.data.push(1);
            self.data.push((2 << 4) | 2);
            self.data.push(0);

            // Cb: H=1, V=1, Tq=0
            self.data.push(2);
            self.data.push((1 << 4) | 1);
            self.data.push(0);

            // Cr: H=1, V=1, Tq=0
            self.data.push(3);
            self.data.push((1 << 4) | 1);
            self.data.push(0);
        }

        // Tc/Th (1 字节)   高4位 Tc（0=DC, 1=AC），低4位 Th（表ID 0~3）
        // BITS[1..=16] (16 字节)
        // HUFFVAL (N 字节)
        pub fn dht(
            &mut self,
            ac_marks: &[u8],
            ac_counts: &[u64],
            dc_marks: &[u8],
            dc_counts: &[u64]
        ) {
            let length = 2 + 1 + 16 + dc_marks.len() + 1 + 16 + ac_marks.len();
            self.data.push(0xff);
            self.data.push(0xc4);
            self.extend(&(length as u16).to_be_bytes());

            // DC 表: Tc=0, Th=0 → 0x00
            self.data.push(0x00);
            self.extend(&dc_counts.iter().map(|v| *v as u8).collect::<Vec<u8>>());
            self.extend(dc_marks);

            // AC 表: Tc=1, Th=1 → 0x11（注意和 SOS 中的 Ta 一致）
            self.data.push(0x11);
            self.extend(&ac_counts.iter().map(|v| *v as u8).collect::<Vec<u8>>());
            self.extend(ac_marks);
        }

        // FF DA           marker
        // Ls (2 字节)     段长度 = 2 + 1 + 2*Ns + 3
        // Ns (1 字节)     扫描中分量数
        // 每个分量 2 字节：Cs (1) + Td/Ta (1)
        // Ss (1) Se (1) Ah/Al (1)
        pub fn sos(&mut self) {
            let length: u16 = 2 + 1 + 2 * 3 + 3;   // = 12
            self.data.push(0xff);
            self.data.push(0xda);
            self.extend(&length.to_be_bytes());

            self.data.push(3);          // Ns = 3

            // Y:  Cs=1, Td=0, Ta=1
            self.data.push(1);
            self.data.push(0x01);

            // Cb: Cs=2, Td=0, Ta=1
            self.data.push(2);
            self.data.push(0x01);

            // Cr: Cs=3, Td=0, Ta=1
            self.data.push(3);
            self.data.push(0x01);

            self.data.push(0);          // Ss
            self.data.push(63);         // Se
            self.data.push(0);          // Ah/Al
        }

        pub fn end(&mut self) {
            self.data.push(0xff);
            self.data.push(0xd9);
        }

        fn extend(&mut self, arr: &[u8]) {
            self.data.extend(arr);
        }
    }
}

mod encode {
    use std::f64;
    use crate::jpeg::file::ZIGZAG_TO_NATURAL;

    pub fn dct(f: &[[f64; 8]; 8]) -> [[f64; 8]; 8] {
        let mut F: [[f64; 8]; 8] = [[0.0; 8]; 8];
        for u in 0..8 {
            for v in 0..8 {
                let cu = if u == 0 { 1. / (2f64).sqrt() } else { 1. };
                let cv = if v == 0 { 1. / (2f64).sqrt() } else { 1. };
                let mut s = 0.0;
                for x in 0..8 {
                    for y in 0..8 {
                        let f1 = ((2. * x as f64 + 1.) * u as f64 * f64::consts::PI / 16.).cos();
                        let f2 = ((2. * y as f64 + 1.) * v as f64 * f64::consts::PI / 16.).cos();
                        s += f[x][y] * f1 * f2;
                    }
                }
                F[u][v] = cu * cv * s / 4.;
            }
        }

        F
    }

    pub fn round(a: f64, b: f64) -> i64 {
        let res = a / b;
        if res < 0. {
            let res = -res;
            let i = res as i64;
            if res - i as f64 >= 0.5 {
                return -(i + 1);
            } else {
                return -i;
            }
        } else {
            let i = res as i64;
            if res - i as f64 >= 0.5 {
                return (i + 1);
            } else {
                return i;
            }
        }
    }

    /// s位
    /// 若 Vt < 2^(s-1)  →  V = Vt - 2^s + 1   （负数）
    /// 若 Vt >= 2^(s-1) →  V = Vt             （正数）
    /// -> (v, s)
    pub fn extend(v: i64) -> (i64, u64) {
        if v == 0 {
            return (0, 0);
        }
        let s = (v.abs() as f64).log2() as u64 + 1;
        if v < 0 {
            (v + (1 << s) - 1, s)
        } else {
            (v, s)
        }
    }

    pub fn series(f: &[[u8; 8]; 8], qq: &[[f64; 8]; 8]) -> Vec<(u8, i64)> {
        let mut ff = [[0.; 8]; 8];
        for i in 0..8 {
            for j in 0..8 {
                ff[i][j] = f[i][j] as f64 - 128.;
            }
        }

        let ff: [[f64; 8]; 8] = dct(&ff);
        
        let mut mat = [[0; 8]; 8];
        for i in 0..8 {
            for j in 0..8 {
                mat[i][j] = round(ff[i][j], qq[i][j]);
            }
        }

        let mut mat1d = [0; 64];
        for i in 0..8 {
            for j in 0..8 {
                mat1d[i * 8 + j] = mat[i][j];
            }
        }
        let mut mat_aft = [0; 64];
        for i in 0..64 {
            mat_aft[i] = mat1d[ZIGZAG_TO_NATURAL[i]];
        }

        let mut ac_series = Vec::new();
        ac_series.push((0x0, mat_aft[0]));                         // DC value
        let mut count_zeros: usize = 0;
        for i in 1..64 {
            if mat_aft[i] != 0 {
                while count_zeros >= 16 {
                    ac_series.push((0xF0, -1));   // ZRL
                    count_zeros -= 16;
                }
                let (v, s) = extend(mat_aft[i]);
                let rs = ((count_zeros as u8) << 4) | ((s as u8) & 0x0f);
                ac_series.push((rs, v));
                count_zeros = 0;
            } else {
                count_zeros += 1;
            }
        }
        if count_zeros > 0 {
            ac_series.push((0, 0));
        }
        
        ac_series
    }
}

#[cfg(feature = "essence")]
mod traditional_huffman {
    use std::cmp::Reverse;

    #[derive(Debug)]
    struct HuffmanNode<'a> {
        subs: Vec<HuffmanNode<'a>>,
        symbol: &'a str,
        number: u64,
    }

    impl<'a> HuffmanNode<'a> {
        fn new(info: &(&'a str, u64)) -> Self {
            let (symbol, number) = info;
            Self { subs: Vec::new(), symbol, number: *number }
        }
    }

    pub fn huffman<'a>(arr: &[(&'a str, u64)]) -> Vec<(&'a str, String)> {
        let mut huffman_nodes = Vec::new();
        for item in arr.iter() {
            huffman_nodes.push(HuffmanNode::new(item));
        }
        huffman_nodes.sort_by_key(|huff| Reverse(huff.number));
        while huffman_nodes.len() > 1 {
            let p1 = huffman_nodes.pop().unwrap();
            let p2 = huffman_nodes.pop().unwrap();
            let mut new_node = HuffmanNode {
                subs: Vec::new(),
                symbol: "",
                number: p1.number + p2.number,
            };
            new_node.subs.push(p1);
            new_node.subs.push(p2);
            huffman_nodes.push(new_node);
            huffman_nodes.sort_by_key(|huff| Reverse(huff.number));
        }

        let mut codes = Vec::new();
        fn dfs<'a>(n: &HuffmanNode<'a>, codes: &mut Vec<(&'a str, String)>, mut code: String) {
            if n.subs.len() == 0 {
                codes.push((n.symbol, code));
                return;
            } else {
                code.push('0');
                dfs(&n.subs[0], codes, code.clone());
                code.pop();
                code.push('1');
                dfs(&n.subs[1], codes, code);
            }
        }
        dfs(&huffman_nodes[0], &mut codes, "".to_owned());
        codes
    }
}

mod jpg_huffman {
    use std::{cmp::Reverse, fmt::Debug};

    #[derive(Debug)]
    struct HuffmanNode {
        subs: Vec<HuffmanNode>,
        symbol: usize,
        number: u64,
    }

    impl HuffmanNode {
        fn new(symbol: usize, number: u64) -> Self {
            Self { subs: Vec::new(), symbol, number }
        }
    }

    impl Default for HuffmanNode {
        fn default() -> Self {
            Self { subs: Vec::new(), symbol: usize::MAX, number: 0 }
        }
    }

    pub fn huffman<T>(arr: &[(T, u64)]) -> Vec<(usize, String)> {
        let mut huffman_nodes = Vec::new();
        for (inx, (_, number)) in arr.iter().enumerate() {
            huffman_nodes.push(HuffmanNode::new(inx, *number));
        }
        huffman_nodes.sort_by_key(|huff| Reverse(huff.number));
        while huffman_nodes.len() > 1 {
            let p1 = huffman_nodes.pop().unwrap();
            let p2 = huffman_nodes.pop().unwrap();
            let mut new_node = HuffmanNode {
                subs: Vec::new(),
                symbol: usize::MAX,
                number: p1.number + p2.number,
            };
            new_node.subs.push(p1);
            new_node.subs.push(p2);
            huffman_nodes.push(new_node);
            huffman_nodes.sort_by_key(|huff| Reverse(huff.number));
        }

        let mut codes = Vec::new();
        fn dfs(n: &HuffmanNode, codes: &mut Vec<(usize, String)>, mut code: String) {
            if n.subs.len() == 0 {
                codes.push((n.symbol, code));
                return;
            } else {
                code.push('0');
                dfs(&n.subs[0], codes, code.clone());
                code.pop();
                code.push('1');
                dfs(&n.subs[1], codes, code);
            }
        }
        dfs(&huffman_nodes[0], &mut codes, "".to_owned());
        codes
    }

    pub fn compress<T>(arr: &[(T, u64)]) -> [u64; 33] {
        let mut huff_tree = huffman(arr);
        let mut bits = count_code_length(huff_tree);
        limit_code_length(&mut bits);
        bits
    }

    fn count_code_length(huff0: Vec<(usize, String)>) -> [u64; 33] {
        let mut counts = [0; 33];
        for i in huff0.iter() {
            counts[i.1.len()] += 1;
        }
        counts
    }

    fn limit_code_length(bits: &mut [u64; 33]) {
        for i in (17..=32).rev() {
            while bits[i] > 0 {
                let mut j = i - 2;
                while j > 0 && bits[j] == 0 {
                    j -= 1;
                }
                if j == 0 {
                    break;
                }

                bits[i] -= 2;
                bits[i - 1] += 1;
                bits[j + 1] += 2;
                bits[j] -= 1;
            }
        }
    }

    #[derive(Debug)]
    pub struct Coder {
        pub min_code: [i64; 33],
        pub max_code: [i64; 33],
        pub ptr_addr: [usize; 33],
    }

    impl Coder {
        pub fn new() -> Self {
            Self { min_code: [0; 33], max_code: [-1; 33], ptr_addr: [0; 33] }
        }

        pub fn create(&mut self, bits: &[u64; 33]) {
            for i in 1..=32 {
                self.min_code[i] = (self.max_code[i - 1] + 1) << 1;
                self.max_code[i] = self.min_code[i] + bits[i] as i64 - 1;
                self.ptr_addr[i] = self.ptr_addr[i - 1] + if i == 1 { 0 } else { bits[i - 1] } as usize;
            }
        }

        pub fn show(&self) -> Vec<String> {
            let mut codes = Vec::new();
            for i in 1..=32 {
                for j in self.min_code[i]..=self.max_code[i] {
                    codes.push(show_digits(j, i));
                }
            }
            codes
        }
    }

    fn show_digits(n: i64, l: usize) -> String {
        let mut view = String::new();
        for i in (0..l).rev() {
            let bit = if n & (1 << i) == 0 { '0' } else { '1' };
            view.push(bit);
        }
        view
    }
}

#[cfg(test)]
mod tests {
    use crate::{jpeg::file::ZIGZAG_TO_NATURAL, test::{jpg_huffman::{self, Coder}}};
    use super::encode::*;
    use std::{io::Write, println, vec};

    #[test]
    fn test_px() {
        let mut f = [
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0], 
            [52.0, 55.0, 61.0, 66.0, 70.0, 61.0, 64.0, 73.0]
        ];

        for i in 0..8 {
            for j in 0..8 {
                f[i][j] -= 128.;
            }
        }

        let mut ff = dct(&f);

        let qq = [
            [16.0, 11.0, 10.0, 16.0, 24.0, 40.0, 51.0, 61.0],
            [12.0, 12.0, 14.0, 19.0, 26.0, 58.0, 60.0, 55.0],
            [14.0, 13.0, 16.0, 24.0, 40.0, 57.0, 69.0, 56.0],
            [14.0, 17.0, 22.0, 29.0, 51.0, 87.0, 80.0, 62.0],
            [18.0, 22.0, 37.0, 56.0, 68.0, 109.0, 103.0, 77.0],
            [24.0, 35.0, 55.0, 64.0, 81.0, 104.0, 113.0, 92.0],
            [49.0, 64.0, 78.0, 87.0, 103.0, 121.0, 120.0, 101.0],
            [72.0, 92.0, 95.0, 98.0, 112.0, 100.0, 103.0, 99.0]
        ];
        
        let mut mat = [[0; 8]; 8];
        for i in 0..8 {
            for j in 0..8 {
                mat[i][j] = round(ff[i][j], qq[i][j]);
            }
        }

        let mut mat1d = [0; 64];
        for i in 0..8 {
            for j in 0..8 {
                mat1d[i * 8 + j] = mat[i][j];
            }
        }
        let mut mat_aft = [0; 64];
        for i in 0..64 {
            mat_aft[i] = mat1d[ZIGZAG_TO_NATURAL[i]];
        }
        println!("{:?}", mat_aft);

        let mut ac_series = Vec::new();
        let mut count_zeros: usize = 0;
        for i in 1..64 {
            if mat_aft[i] != 0 {
                let (v, s) = extend(mat_aft[i]);
                let rs = ((count_zeros as u8) << 4) | ((s as u8) & 0x0f);
                ac_series.push((rs, v));
                count_zeros = 0;
            } else {
                count_zeros += 1;
            }
        }
        ac_series.push((0, -1));
        println!("{:?}", ac_series);
    }

    #[test]
    #[cfg(feature = "essence")]
    fn test_traditional_huffman() {
        let original_arr: Vec<(&'static str, u64)> = vec![
            ("A", 5),
            ("B", 9),
            ("C", 12),
            ("D", 13),
            ("E", 16),
            ("F", 45),
        ];
        let huff = crate::test::traditional_huffman::huffman(&original_arr);
        println!("{huff:?}");
    }

    #[test]
    fn test_jpg_huffman() {
        let original_arr: Vec<(&'static str, u64)> = vec![
            ("A", 5),
            ("B", 9),
            ("C", 12),
            ("D", 13),
            ("E", 16),
            ("F", 45),
        ];
        let huff = jpg_huffman::huffman(&original_arr);
        let mut res = Vec::new();
        for i in 0..huff.len() {
            res.push((original_arr[huff[i].0].0, huff[i].1.clone()));
        }
        println!("{res:?}");
    }

    #[test]
    fn test_compress() {
        let mut original_arr: Vec<(&'static str, u64)> = vec![
            ("A", 5),
            ("B", 9),
            ("C", 12),
            ("D", 13),
            ("E", 16),
            ("F", 45),
            ("EOF", 0),
        ];
        let counts = jpg_huffman::compress(&original_arr);
        original_arr.sort_by_key(|(_, b)| std::cmp::Reverse(*b));
        let marks: Vec<&str> = original_arr.iter().map(|(a, _)| *a).collect();
        println!("{:?}", marks);
        println!("{:?}", counts);
    }

    #[test]
    fn test_coder() {
        let mut original_arr: Vec<(&'static str, u64)> = vec![
            ("A", 5),
            ("B", 9),
            ("C", 12),
            ("D", 13),
            ("E", 16),
            ("F", 45),
            ("EOF", 0),
        ];
        let counts = jpg_huffman::compress(&original_arr);
        original_arr.sort_by_key(|(_, b)| std::cmp::Reverse(*b));
        let marks: Vec<&str> = original_arr.iter().map(|(a, _)| *a).collect();

        println!("{counts:?}");
        let mut coder = Coder::new();
        coder.create(&counts);
        // println!("{coder:?}");

        let codes = coder.show();
        println!("{:?}", marks);
        println!("{:?}", codes);
    }

    #[test]
    fn test_example() {
        use super::example;
        let jpeg = example::serialize();
        let mut file = std::fs::File::create("./bin/test.jpg").unwrap();
        file.write_all(&jpeg.data).unwrap();
        file.flush().unwrap();
    }
}
