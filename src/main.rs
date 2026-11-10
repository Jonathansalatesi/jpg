//! JPG → PPM(P6) 命令行工具
//!
//! 用法：
//! ```text
//! cargo run --release -- <输入.jpg> <输出.ppm>
//! cargo run --release              # 默认 bin/20210617054007202.jpg → out/20210617054007202.ppm
//! ```

use std::format;
use std::path::Path;
use std::time::Instant;

use jpg::jpeg::decoder::JpegImage;
use jpg::jpeg::file::JpegRaw;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();

    let input = args
        .get(0)
        .cloned()
        .unwrap_or_else(|| "bin/20210617054007202.jpg".to_string());

    // 没指定输出时，把输入文件名的扩展名换成 .ppm，放到 out/ 下
    let output = args.get(1).cloned().unwrap_or_else(|| {
        let stem = Path::new(&input)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("output");
        format!("out/{stem}.ppm")
    });

    // ---------- 1. 读文件 ----------
    let data = match std::fs::read(&input) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("读取 {input} 失败: {e}");
            std::process::exit(1);
        }
    };
    println!("读入 {} 字节: {input}", data.len());

    // ---------- 2. 分段解析（file.rs 已完成） ----------
    let t0 = Instant::now();
    let mut raw = JpegRaw::new(&data);
    let jpeg = raw.parse();
    println!("分段解析完成: {} 个段, 用时 {:?}", jpeg.segments.len(), t0.elapsed());

    // ---------- 3. 熵解码 + 反量化 + IDCT ----------
    let t1 = Instant::now();
    let img = match JpegImage::decode(jpeg) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("解码失败: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "解码完成: {}x{}, {} 个分量, 重启间隔 {}, 用时 {:?}",
        img.width,
        img.height,
        img.components.len(),
        img.restart_interval,
        t1.elapsed()
    );
    for c in &img.components {
        println!(
            "  分量 id={} 采样 {}x{} 量化表 {} 样本 {}x{}",
            c.id, c.h_sampling, c.v_sampling, c.quant_table_id, c.width, c.height
        );
    }

    // ---------- 4. YCbCr → RGB → PPM ----------
    let t2 = Instant::now();
    if let Some(dir) = Path::new(&output).parent() {
        if !dir.as_os_str().is_empty() {
            let _ = std::fs::create_dir_all(dir);
        }
    }
    if let Err(e) = img.write_ppm(&output) {
        eprintln!("写出 PPM 失败: {e}");
        std::process::exit(1);
    }
    println!(
        "写出 PPM(P6): {output} ({} 字节), 用时 {:?}",
        std::fs::metadata(&output).map(|m| m.len()).unwrap_or(0),
        t2.elapsed()
    );
}
