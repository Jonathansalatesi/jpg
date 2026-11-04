use std::{mem::MaybeUninit, ptr::addr_of_mut, todo};

use crate::mem;

pub struct JpegRaw<'a> {
    pub data: &'a [u8],
    pub addr: usize,
}

pub struct JpegFile {
    pub segments: Vec<JpegSegment>,
}

pub enum JpegSegment {
    // Independent symbol, no data
    Soi,
    Eoi,

    // The segment with parameters, the data can be parsed as specified structure
    App0(App0Data),     // JFIF infos
    // App1(App1Data),          // Exif/XMP等，可原始保存或解析
    Dqt(QuantizationTable),  // 量化表
    Sof(FrameHeader),        // 帧头 (SOF0, SOF2等)
    Dht(HuffmanTable),       // 哈夫曼表
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
        }
    }

    pub fn parse_segment(&mut self) -> JpegSegment {
        match self.data[self.addr + 1] {
            0xd8 => self.parse_soi(),
            0xc0 => self.parse_sof(),
            0xc2 => self.parse_sof(),
            0xc4 => self.parse_dht(),
            0xdb => self.parse_dqt(),
            0xdd => self.parse_dri(),
            0xda => self.parse_sos(),
            0xd0..=0xd7 => self.parse_rst(),
            // 0xe0..=0xef => self.parse_app0(),
            0xfe => self.parse_com(),
            0xd9 => self.parse_eoi(),
            _ => self.parse_unknown(),
        }
    }

    fn parse_soi(&mut self) -> JpegSegment {
        self.addr += 2;
        JpegSegment::Soi
    }

    fn parse_eoi(&mut self) -> JpegSegment {
        self.addr += 2;
        JpegSegment::Eoi
    }

    // App0(App0Data),     // JFIF infos
    // fn parse_app0(&mut self) -> JpegSegment {
    //     let 
    // }

    // Dqt(QuantizationTable),  // 量化表
    fn parse_dqt(&mut self) -> JpegSegment {
        self.addr += 2;
        let dstlen = std::mem::size_of::<QuantizationTable>();
        let mut qtt = MaybeUninit::<QuantizationTable>::uninit();
        crate::mem::memcpy(&self.data[self.addr] as *const u8, qtt.as_mut_ptr() as *mut u8, dstlen as u64);
        self.addr += dstlen;
        unsafe {
            JpegSegment::Dqt(qtt.assume_init())
        }
    }

    // Sof(FrameHeader),        // 帧头 (SOF0, SOF2等)
    fn parse_sof(&mut self) -> JpegSegment {
        self.addr += 1;
        let mut frame_header = MaybeUninit::<FrameHeader>::uninit();
        let fh_ptr = frame_header.as_mut_ptr();
        unsafe {
            addr_of_mut!((*fh_ptr).is_progressive).write(if self.data[self.addr] == 0xc0 { false } else { true } );
            self.addr += 1;
            // pub precision: u8,
            addr_of_mut!((*fh_ptr).precision).write(self.data[self.addr]);
            self.addr += 1;
            // pub width: u16,
            addr_of_mut!((*fh_ptr).width).write(u16::from_be_bytes([self.data[self.addr], self.data[self.addr + 1]]));
            self.addr += 2;
            // pub height: u16,
            addr_of_mut!((*fh_ptr).height).write(u16::from_be_bytes([self.data[self.addr], self.data[self.addr + 1]]));
            self.addr += 2;
            // pub components: Vec<FrameComponent>,
            let mut components = Vec::new();
            while self.data[self.addr] != 0xff {
                let mut comp = MaybeUninit::<FrameComponent>::uninit();
                crate::mem::memcpy(&self.data[self.addr] as *const u8, comp.as_mut_ptr() as *mut u8, 4);
                components.push(comp.assume_init());
                self.addr += 4;
            }
            addr_of_mut!((*fh_ptr).components).write(components);
            JpegSegment::Sof(frame_header.assume_init())
        }
    }

    // Dht(HuffmanTable),       // 哈夫曼表
    pub fn parse_dht(&mut self) -> JpegSegment {
        todo!()
    }

    // Sos(ScanHeader),         // 扫描头
    pub fn parse_sos(&mut self) -> JpegSegment {
        todo!()
    }

    // Dri(RestartInterval),    // 重启间隔
    pub fn parse_dri(&mut self) -> JpegSegment {
        todo!()
    }

    // Com(Vec<u8>),            // 注释
    pub fn parse_com(&mut self) -> JpegSegment {
        todo!()
    }

    // EntropyData(Vec<u8>),
    pub fn parse_entropy_data(&mut self) -> JpegSegment {
        todo!()
    }

    pub fn parse_rst(&mut self) -> JpegSegment {
        todo!()
    }

    // Unknown { marker: u8, data: Vec<u8> },
    pub fn parse_unknown(&mut self) -> JpegSegment {
        todo!()
    }
}

#[repr(C)]
pub struct App0Data {
    pub version: (u8, u8),
    pub density_units: u8,
    pub x_density: u16,
    pub y_density: u16,
    pub thumbnail: Option<(u8, u8, Vec<u8>)>,   // Width, Height, Pixial data of RGB
}

#[repr(C)]
pub struct QuantizationTable {
    pub table_id: u8,           // 0-3
    pub precision: u8,          // 0 = 8bits, 1 = 16 bits
    pub values: [u16; 64],      // In natural to store, not zig-zag
}

#[repr(C)]
pub struct FrameHeader {
    pub precision: u8,
    pub width: u16,
    pub height: u16,
    pub components: Vec<FrameComponent>,
    pub is_progressive: bool, // 根据SOF标记区分
}

#[repr(C)]
pub struct FrameComponent {
    pub id: u8,
    pub h_sampling: u8, // 水平采样因子
    pub v_sampling: u8, // 垂直采样因子
    pub quant_table_id: u8,
}

pub struct HuffmanTable {
    pub table_class: u8, // 0=DC, 1=AC
    pub table_id: u8,
    pub counts: [u8; 16],  // 即 BITS 表
    pub symbols: Vec<u8>,  // 即 HUFFVAL 表
}

pub struct ScanHeader {
    pub components: Vec<ScanComponent>,
    pub spectral_selection: (u8, u8, u8), // Ss, Se, Ah/Al
}

#[repr(C)]
pub struct ScanComponent {
    pub component_id: u8,
    pub dc_table_id: u8,
    pub ac_table_id: u8,
}

#[repr(C)]
pub struct RestartInterval {
    pub interval: u16,
}

#[test]
fn test_volume(){
    let val = std::mem::size_of::<ScanComponent>();
    println!("{val}");
}