use std::{matches, mem::MaybeUninit, ptr::addr_of_mut};
use crate::mem::{u16_be, divide_u8};

pub const ZIGZAG_TO_NATURAL: [usize; 64] = [
    0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
];

#[derive(Debug)]
enum ParserState {
    Segment,
    Entropy,
    Finished,
}

pub struct JpegRaw<'a> {
    pub data: &'a [u8],
    pub addr: usize,
    pub state: ParserState,
}

#[derive(Debug)]
pub struct JpegFile {
    pub segments: Vec<JpegSegment>,
}

#[repr(C)]
#[derive(Debug)]
pub struct App0Data {
    pub length: u16,
    pub version: [u8; 2],
    pub density_units: u8,  // 0 = no unit, 1 = DPI, 2 = DPCM
    pub x_density: u16,
    pub y_density: u16,
    pub thumbnail: Option<(u8, u8, Vec<u8>)>,   // Width, Height, Pixial data of RGB
}

#[repr(C)]
#[derive(Debug)]
pub struct QuantizationTable {
    pub id: u8,                 // 0-3
    pub precision: u8,          // 0 = 8bits, 1 = 16 bits
    pub values: [u16; 64],      // In natural to store, not zig-zag
}

#[repr(C)]
#[derive(Debug)]
pub struct FrameHeader {
    pub marker: u8,
    pub precision: u8,
    pub height: u16,
    pub width: u16,
    pub components: Vec<FrameComponent>,
}

#[repr(C)]
#[derive(Debug)]
pub struct FrameComponent {
    pub id: u8,         // 1 = Y, 2 = Cb, 3 = Cr
    pub h_sampling: u8, // 水平采样因子
    pub v_sampling: u8, // 垂直采样因子
    pub quant_table_id: u8,
}

#[repr(C)]
#[derive(Debug)]
pub struct HuffmanTable {
    pub table_class: u8, // 0=DC, 1=AC
    pub table_id: u8,
    pub counts: [u8; 16],  // 即 BITS 表
    pub symbols: Vec<u8>,  // 即 HUFFVAL 表
}

#[repr(C)]
#[derive(Debug)]
pub struct ScanHeader {
    pub components: Vec<ScanComponent>,
    pub spectral_selection: (u8, u8, u8), // Ss, Se, Ah/Al, baseline: Ss = 0, Se = 63, Ah/Al = 0
}

#[repr(C)]
#[derive(Debug)]
pub struct ScanComponent {
    pub component_id: u8,
    pub dc_table_id: u8,
    pub ac_table_id: u8,
}

#[repr(C)]
#[derive(Debug)]
pub struct RestartInterval {
    pub interval: u16,
}

#[derive(Debug)]
pub enum JpegSegment {
    // Independent symbol, no data
    Soi,
    Eoi,

    // The segment with parameters, the data can be parsed as specified structure
    App0(App0Data),     // JFIF infos
    // App1(App1Data),          // Exif/XMP等，可原始保存或解析
    Dqt(Vec<QuantizationTable>),  // 量化表
    Sof(FrameHeader),        // 帧头 (SOF0, SOF2等)
    Dht(Vec<HuffmanTable>),       // 哈夫曼表
    Sos(ScanHeader),         // 扫描头
    Dri(RestartInterval),    // 重启间隔
    Rst(u8),
    Com(Vec<u8>),            // 注释

    // 熵编码数据，通常不解析，作为原始字节保留
    EntropyData(Vec<u8>),

    // 兜底：保留未识别的段，便于无损重写
    Unknown { marker: u8, data: Vec<u8> },
}

impl<'a> JpegRaw<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            data: buf,
            addr: 0,
            state: ParserState::Segment,
        }
    }

    fn read_u16_be(&mut self) -> u16 {
        let res = u16_be(&self.data[self.addr..]);
        self.addr += 2;
        res
    }

    fn read_u8(&mut self) -> u8 {
        let res = self.data[self.addr];
        self.addr += 1;
        res
    }

    fn read_u8_pair(&mut self) -> [u8; 2] {
        let res = [self.data[self.addr], self.data[self.addr + 1]];
        self.addr += 2;
        res
    }

    fn read_vec(&mut self, length: usize) -> Vec<u8> {
        let res = Vec::from(&self.data[self.addr..self.addr + length as usize]);
        self.addr += length;
        res
    }

    pub fn parse_segment(&mut self) -> Option<JpegSegment> {
        match self.state {
            ParserState::Segment => {
                let res = match u16_be(&self.data[self.addr..])  {
                    0xffd8 => self.parse_soi(),
                    0xffc0 | 0xffc2 => self.parse_sof(),
                    0xffc4 => self.parse_dht(),
                    0xffdb => self.parse_dqt(),
                    0xffdd => self.parse_dri(),
                    0xffda => self.parse_sos(),
                    0xffd0..=0xffd7 => self.parse_rst(),
                    0xffe0 => self.parse_app0(),
                    0xfffe => self.parse_com(),
                    0xffd9 => self.parse_eoi(),
                    _ => self.parse_unknown(),
                };
                Some(res)
            },
            ParserState::Entropy => Some(self.parse_entropy_data()),
            _ => None,
        }
    }

    fn parse_soi(&mut self) -> JpegSegment {
        self.addr += 2;
        JpegSegment::Soi
    }

    fn parse_eoi(&mut self) -> JpegSegment {
        self.addr += 2;
        self.state = ParserState::Finished;
        JpegSegment::Eoi
    }

    // App0(App0Data),     // JFIF infos
    fn parse_app0(&mut self) -> JpegSegment {
        self.addr += 2;
        let length = self.read_u16_be();
        self.addr += 5;     // Jmp length(2 bytes) + "JFIF\0"(5 bytes)
        
        let mut app0data = MaybeUninit::<App0Data>::uninit();
        let ptr = app0data.as_mut_ptr();
        unsafe {
            addr_of_mut!((*ptr).length).write(length);
            addr_of_mut!((*ptr).version).write(self.read_u8_pair());
            addr_of_mut!((*ptr).density_units).write(self.read_u8());
            addr_of_mut!((*ptr).x_density).write(self.read_u16_be());
            addr_of_mut!((*ptr).y_density).write(self.read_u16_be());
            let x_thumbnail = self.read_u8();
            let y_thumbnail = self.read_u8();
            let thumbnail_len = x_thumbnail as usize * y_thumbnail as usize * 3;
            if thumbnail_len > 0 {
                let thumbnail = self.read_vec(thumbnail_len);
                addr_of_mut!((*ptr).thumbnail).write(Some((x_thumbnail, y_thumbnail, thumbnail)));
            } else {
                addr_of_mut!((*ptr).thumbnail).write(None);
            }
        
            JpegSegment::App0(app0data.assume_init())
        }
    }

    // Dqt(QuantizationTable),  // 量化表
    fn parse_dqt(&mut self) -> JpegSegment {
        self.addr += 2;
        let mut res_seg_len = self.read_u16_be();
        res_seg_len -= 2;

        let mut quantables = Vec::new();
        while res_seg_len > 0 {
            let quantable = self.parse_quantable();
            if quantable.precision == 0 {
                res_seg_len -= 65;
            } else {
                res_seg_len -= 129;
            }
            quantables.push(quantable);
        }
        JpegSegment::Dqt(quantables)
    }

    fn parse_quantable(&mut self) -> QuantizationTable {
        let (pq, tq) = divide_u8(self.read_u8());
        let mut table: [u16; 64] = [0; 64];
        if pq == 0 {
            for i in 0..64 {
                table[ZIGZAG_TO_NATURAL[i]] = self.read_u8() as u16;
            }
        } else {
            for i in 0..64 {
                table[ZIGZAG_TO_NATURAL[i]] = self.read_u16_be();
            }
        }
        QuantizationTable { id: tq, precision: pq, values: table }
    }

    // Sof(FrameHeader),        // 帧头 (SOF0, SOF2等)
    fn parse_sof(&mut self) -> JpegSegment {
        JpegSegment::Sof(self.parse_frame_header())
    }

    fn parse_frame_header(&mut self) -> FrameHeader {
        let marker = self.data[self.addr + 1];
        self.addr += 4;     // marker + segment length
        let precision = self.read_u8();
        let height = self.read_u16_be();
        let width = self.read_u16_be();
        let nf = self.read_u8();
        let mut components = Vec::with_capacity(nf as usize);
        for _ in 0..nf {
            components.push(self.parse_frame_component());
        }
        FrameHeader { marker, precision, height, width, components }
    }

    fn parse_frame_component(&mut self) -> FrameComponent {
        let id = self.read_u8();
        let hi_vi = self.read_u8();
        let (h_sampling, v_sampling) = divide_u8(hi_vi);
        let quant_table_id = self.read_u8();
        FrameComponent { id, h_sampling, v_sampling, quant_table_id }
    }

    // Dht(HuffmanTable),       // 哈夫曼表
    fn parse_dht(&mut self) -> JpegSegment {
        self.addr += 2;
        let length = self.read_u16_be();
        let mut remaining = (length - 2) as usize;

        let mut dhts = Vec::new();
        while remaining > 0 {
            let start = self.addr;
            dhts.push(self.parse_huffman_table());
            let consumed = self.addr - start;
            remaining -= consumed;
        }
        JpegSegment::Dht(dhts)
    }

    fn parse_huffman_table(&mut self) -> HuffmanTable {
        let tc_th = self.read_u8();
        let (table_class, table_id) = divide_u8(tc_th);
        let mut counts = [0; 16];
        for i in 0..16 {
            counts[i] = self.read_u8();
        }
        let symbols_len = counts.iter().map(|&i| i as usize).sum();
        let mut symbols = Vec::with_capacity(symbols_len);
        for _ in 0..symbols_len {
            symbols.push(self.read_u8());
        }
        HuffmanTable { table_class, table_id, counts, symbols }
    }

    // Sos(ScanHeader),         // 扫描头
    fn parse_sos(&mut self) -> JpegSegment {
        self.addr += 4;
        self.state = ParserState::Entropy;
        JpegSegment::Sos(self.parse_scanheader())
    }

    fn parse_scanheader(&mut self) -> ScanHeader {
        let ns = self.read_u8();
        let mut components = Vec::with_capacity(ns as usize);
        for _ in 0..ns {
            components.push(self.parse_scancomponent());
        }
        let spectral_selection = (self.read_u8(), self.read_u8(), self.read_u8());
        ScanHeader { components, spectral_selection }
    }

    fn parse_scancomponent(&mut self) -> ScanComponent {
        let component_id = self.read_u8();
        let td_ta = self.read_u8();
        let (dc_table_id, ac_table_id) = divide_u8(td_ta);
        ScanComponent { component_id, dc_table_id, ac_table_id }
    }

    // Dri(RestartInterval),    // 重启间隔
    fn parse_dri(&mut self) -> JpegSegment {
        self.addr += 4;
        let interval = self.read_u16_be();
        JpegSegment::Dri(RestartInterval { interval })
    }

    // Com(Vec<u8>),            // 注释
    fn parse_com(&mut self) -> JpegSegment {
        self.addr += 2;
        let length = self.read_u16_be();
        let data = self.read_vec((length - 2) as usize);
        JpegSegment::Com(data)
    }

    // EntropyData(Vec<u8>),
    fn parse_entropy_data(&mut self) -> JpegSegment {
        let start = self.addr;
        let data_len = self.data.len();

        while self.addr < data_len {
            if self.data[self.addr] != 0xff {
                self.addr += 1;
                continue;
            }

            let marker_start = self.addr;
            let mut code_pos = marker_start + 1;
            while code_pos < data_len && self.data[code_pos] == 0xff {
                code_pos += 1;
            }

            if code_pos >= data_len {
                self.addr = data_len;
                break;
            }
            let marker = self.data[code_pos];

            match marker {
                0x00 if code_pos == marker_start + 1 => {
                    self.addr = code_pos + 1;
                },
                0xd0..=0xd7 => {
                    self.addr = code_pos + 1;
                }
                _ => {
                    self.addr = marker_start;
                    self.state = ParserState::Segment;
                    break;
                }
            }
        }
        if self.addr >= data_len {
            self.state = ParserState::Finished;
        }
        JpegSegment::EntropyData(
            self.data[start..self.addr].to_vec()
        )
    }

    fn parse_rst(&mut self) -> JpegSegment {
        self.addr += 2;
        JpegSegment::Rst(self.data[self.addr - 1])
    }

    // Unknown { marker: u8, data: Vec<u8> },
    fn parse_unknown(&mut self) -> JpegSegment {
        let marker = self.data[self.addr + 1];
        self.addr += 2;
        if marker == 0x01 {
            return JpegSegment::Unknown { marker, data: Vec::new() };
        }
        let length = self.read_u16_be();
        let data = self.read_vec((length - 2) as usize);
        JpegSegment::Unknown { marker, data }
    }

    pub fn parse(&mut self) -> JpegFile {
        let mut segments = Vec::new();
        while let Some(segment) = self.parse_segment() {
            let finished = matches!(segment, JpegSegment::Eoi);
            segments.push(segment);
            if finished {
                break;
            }
        }
        JpegFile { segments }
    }
}

#[test]
fn test_parser() {
    let file = std::fs::read("./bin/20210617054007202.jpg").unwrap();
    let mut raw = JpegRaw::new(&file);
    let jpeg_file = raw.parse();
    println!("{:?}", jpeg_file);
}