use std::fs::File;
use std::io::{self, BufReader, Read};
use std::path::Path;

/// 读取 PPM 文件，按通道拆分。
/// 返回 (width, height, r, g, b)：
///   r[y][x] / g[y][x] / b[y][x] 分别是 (x, y) 处像素的红/绿/蓝分量（0..=255）
pub fn read_ppm_channels<P: AsRef<Path>>(
    path: P,
) -> io::Result<(
    usize,
    usize,
    Vec<Vec<u8>>,
    Vec<Vec<u8>>,
    Vec<Vec<u8>>,
)> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    // ---- 头部 ----
    let magic = read_token(&mut reader)?;
    let is_binary = match magic.as_slice() {
        b"P6" => true,
        b"P3" => false,
        _ => return Err(invalid_data("expected P6 or P3")),
    };

    let width = parse_usize(&read_token(&mut reader)?)?;
    let height = parse_usize(&read_token(&mut reader)?)?;
    let maxval = parse_usize(&read_token(&mut reader)?)?;

    if width == 0 || height == 0 {
        return Err(invalid_data("width/height must be positive"));
    }
    if maxval == 0 || maxval > 65535 {
        return Err(invalid_data("maxval must be in 1..=65535"));
    }

    let total = width
        .checked_mul(height)
        .and_then(|n| n.checked_mul(3))
        .ok_or_else(|| invalid_data("dimensions overflow"))?;

    // ---- 读取原始采样到 u16 缓冲（统一位深）----
    let mut raw: Vec<u16> = Vec::with_capacity(total);

    if is_binary {
        let bps = if maxval < 256 { 1 } else { 2 };
        let byte_len = total
            .checked_mul(bps)
            .ok_or_else(|| invalid_data("data size overflow"))?;
        let mut buf = vec![0u8; byte_len];
        reader.read_exact(&mut buf)?;

        if bps == 1 {
            raw.extend(buf.iter().map(|&b| b as u16));
        } else {
            raw.extend(
                buf.chunks_exact(2)
                    .map(|c| u16::from_be_bytes([c[0], c[1]])),
            );
        }
    } else {
        for _ in 0..total {
            let tok = read_token(&mut reader)?;
            let v = parse_usize(&tok)?;
            if v > maxval {
                return Err(invalid_data("sample exceeds maxval"));
            }
            raw.push(v as u16);
        }
    }

    // ---- 按通道拆分为三个二维数组 ----
    let scale = |v: u16| -> u8 {
        if maxval == 255 {
            v as u8
        } else {
            let m = maxval as u32;
            (((v as u32) * 255 + m / 2) / m).min(255) as u8
        }
    };

    // 预分配 height 行，每行 width 个元素
    let mut r = vec![vec![0u8; width]; height];
    let mut g = vec![vec![0u8; width]; height];
    let mut b = vec![vec![0u8; width]; height];

    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) * 3;
            r[y][x] = scale(raw[idx]);
            g[y][x] = scale(raw[idx + 1]);
            b[y][x] = scale(raw[idx + 2]);
        }
    }

    Ok((width, height, r, g, b))
}

// ---------- 头部解析辅助函数（与之前一致） ----------

fn read_token<R: Read>(reader: &mut BufReader<R>) -> io::Result<Vec<u8>> {
    let mut byte = [0u8; 1];
    loop {
        if reader.read(&mut byte)? == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "unexpected EOF in header",
            ));
        }
        match byte[0] {
            b'#' => skip_to_eol(reader)?,
            b if b.is_ascii_whitespace() => continue,
            _ => break,
        }
    }
    let mut token = vec![byte[0]];
    loop {
        if reader.read(&mut byte)? == 0 {
            break;
        }
        match byte[0] {
            b'#' => {
                skip_to_eol(reader)?;
                break;
            }
            b if b.is_ascii_whitespace() => break,
            b => token.push(b),
        }
    }
    Ok(token)
}

fn skip_to_eol<R: Read>(reader: &mut BufReader<R>) -> io::Result<()> {
    let mut byte = [0u8; 1];
    loop {
        if reader.read(&mut byte)? == 0 {
            return Ok(());
        }
        if byte[0] == b'\n' {
            return Ok(());
        }
    }
}

fn parse_usize(tok: &[u8]) -> io::Result<usize> {
    std::str::from_utf8(tok)
        .ok()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| invalid_data("invalid integer"))
}

fn invalid_data(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg)
}