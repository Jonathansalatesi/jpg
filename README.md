# A JPEG Codec in Pure Rust

A **dependency-free** baseline JPEG codec: it decodes a JPEG into pixels and writes them as PPM P6, and it encodes RGB pixels back into a JPEG file.

Every stage lives in this repository — segment parsing, Huffman coding, DCT/IDCT, quantization, chroma subsampling/upsampling, YCbCr ↔ RGB — with no image library involved.

- Edition: Rust 2024 (requires rustc ≥ 1.88; `src/mem.rs` uses the now-stable `#[unsafe(naked)]`)
- Dependencies: **none** — `[dependencies]` in `Cargo.toml` is empty

---

## Status

| Capability | State | Entry point |
| --- | --- | --- |
| Segment parsing (SOI / APP0 / DQT / SOF0 / DHT / SOS / DRI / RST / COM / unknown) | ✅ | `jpeg::file::JpegRaw::parse` |
| Baseline (SOF0) entropy decode → per-component sample planes | ✅ pixel-aligned with libjpeg | `jpeg::decoder::JpegImage::decode` |
| Nearest-neighbour upsampling + YCbCr → RGB | ✅ | `JpegImage::to_rgb` |
| Write PPM P6 | ✅ | `JpegImage::write_ppm` |
| Read PPM (P3 / P6) | ✅ | `ppm::read_ppm_channels` |
| Baseline (SOF0) encode, 4:4:4 or 4:2:0 | ✅ | `jpeg::encoder::JpegMeta::gen_file` |
| Per-image optimized Huffman tables + Annex K.2 length limiting | ✅ | `encoder::jpg_huffman::compress` |
| Restart intervals (DRI / RSTn) | decode ✅ / encode ❌ | — |
| Progressive (SOF2), arithmetic coding, 12-bit, CMYK/YCCK | ❌ returns an explicit error | — |
| APP0 (JFIF) header | ❌ not emitted by the encoder | — |

> `bin/` and `out/` are both in `.gitignore`, so sample images and output never reach the repo — drop a JPEG into `bin/` yourself.

---

## Layout

```
src/
├── main.rs              decode CLI: *.jpg → *.ppm
├── lib.rs               crate root: pub mod mem / jpeg / ppm, mod test
├── mem.rs               hand-written memcpy (x86_64 + Windows, naked asm, currently unused)
│                        + the u16_be / divide_u8 helpers
├── ppm/mod.rs           PPM reader (P3 text / P6 binary, any maxval normalised to 0..255)
├── test.rs              early encoder sandbox (example / encode / jpg_huffman) + DCT/Huffman unit tests
└── jpeg/
    ├── mod.rs
    ├── file.rs          segment parsing: JpegRaw (cursor + state machine) → JpegFile { segments }
    ├── decoder.rs       entropy decode → dequantize → IDCT → upsample → YCbCr→RGB → PPM
    └── encoder.rs       RGB → YCbCr → blocking → FDCT → quantize → RLE → Huffman → byte stream
```

`src/test.rs` is the prototype the encoder had before it was consolidated into `jpeg/encoder.rs`; the two contain
duplicated logic (see [Known issues](#known-issues--todo)). It is pulled in by `lib.rs` as `mod test;`, and its
`#[cfg(test)]` section only compiles for tests.

---

## Quick start

### Decode: JPEG → PPM

```bash
cargo run --release                                # bin/20210617054007202.jpg → out/20210617054007202.ppm
cargo run --release -- input.jpg output.ppm         # explicit input / output
```

The CLI reports the sampling factors, quantization-table ids and real sample dimensions of every component
(progress lines are printed in Chinese):

```
分段解析完成: 13 个段, 用时 614.6µs
解码完成: 3840x2400, 3 个分量, 重启间隔 0, 用时 288.3ms
  分量 id=1 采样 2x2 量化表 0 样本 3840x2400
  分量 id=2 采样 1x1 量化表 1 样本 1920x1200
  分量 id=3 采样 1x1 量化表 1 样本 1920x1200
写出 PPM(P6): out/20210617054007202.ppm (27648017 字节)
```

### Encode: RGB → JPEG

The encoder is currently driven by tests (it is not wired into the CLI yet):

```bash
cargo test --release test_example -- --nocapture
```

This produces two files:

| Output | Produced by | Content |
| --- | --- | --- |
| `bin/test1.jpg` | test in `src/jpeg/encoder.rs` | 3840×2400, 4:4:4, reads `out/20210617054007202.ppm` |
| `bin/test.jpg` | test in `src/test.rs` | 1920×1080, synthetic image (circle on white) |

### Viewing a PPM

PPM is uncompressed raw RGB and many viewers won't open it. Converting to PNG is the easy way:

```bash
ffmpeg -i out/20210617054007202.ppm out/preview.png
```

---

## Decode pipeline

```
JPEG file
  │
  ├─ file.rs: split on 0xFF markers → Vec<JpegSegment>
  │     SOI / APP0 / DQT / SOF0 / DHT / SOS / [entropy data] / EOI
  │
  ├─ decoder.rs: one pass per SOS (baseline has a single scan: Ss=0, Se=63, Ah=Al=0)
  │     ├ strip 0xFF00 stuffing, split at RSTn
  │     ├ for each MCU, for each 8×8 block:
  │     │     Huffman-decode the DC category → sign-extend → accumulate into the DC predictor
  │     │     Huffman-decode the AC (run, size) symbols → handle ZRL / EOB → sign-extend
  │     │     dequantize: F(u,v) = coefficient × Q(u,v)
  │     │     2-D IDCT + level shift (+128) + clamp
  │     │     scatter into the component's sample plane
  │     └ at every DRI interval, reset the DC predictors
  │
  ├─ upsampling: chroma planes scaled by hmax/h and vmax/v (nearest neighbour)
  │
  └─ YCbCr → RGB (BT.601)
        R = Y + 1.402·Cr'
        G = Y − 0.344136·Cb' − 0.714136·Cr'
        B = Y + 1.772·Cb'          (Cb' = Cb−128, Cr' = Cr−128)
```

Implementation notes:

- **Huffman decoding uses canonical-code lookup, not a tree.** `BITS`/`HUFFVAL` from DHT are expanded into
  `min_code[l] / max_code[l] / value_ptr[l]`; the decoder shifts in one bit at a time (`code = (code<<1)|bit`)
  and as soon as `code <= max_code[l]` it hits: `symbol = HUFFVAL[value_ptr[l] + code - min_code[l]]`.
- **Sign extension** (ITU T.81 F.2.2.1): read `s` bits into `Vt`; if `Vt < 2^(s-1)` then `V = Vt − 2^s + 1`,
  otherwise `V = Vt`.
- **IDCT is a separable float implementation**: the 2-D formula is split into a row pass and a column pass,
  with the kernel `basis[x][u] = C(u)/2 · cos((2x+1)uπ/16)`. Each pass carries a factor 1/2, so together they
  produce the 1/4 of the formula. When a row's AC terms are all zero the pass reduces to the constant
  `F(v,0)·C(0)/2`, which measurably removes most of the multiplies on real photos.
- **Upsampling is nearest neighbour**: `sx = x·h/hmax`, `sy = y·v/vmax`. libjpeg defaults to triangular-filter
  "fancy upsampling", which is why comparisons against it differ along chroma edges (see [Verification](#verification)).

---

## Encode pipeline

```rust
let mut jpg = JpegMeta::new(width, height, 4);   // resolute == 2 → 4:2:0, anything else → 4:4:4
jpg.rgb_in(r, g, b);                             // Vec<Vec<u8>> indexed [y][x]
jpg.gen_file(&LIGHT_QUANT, &COLOR_QUANT, &COLOR_QUANT, "./bin/test1.jpg");
```

```
RGB pixels
  │
  ├─ RGB → YCbCr (BT.601, +128 bias)
  ├─ chroma downsample: 4:2:0 keeps the sample at (2i, 2j) (nearest neighbour)
  ├─ cut into 8×8 blocks, replicating edge pixels along the right/bottom border
  │
  ├─ per block: level shift (−128) → float FDCT → divide by Q and round → zigzag reorder
  │             → DC differential coding → AC run-length coding (0xF0 = ZRL, skips 16 zeros; 0x00 = EOB)
  │
  ├─ collect DC / AC symbol frequencies over the whole image → build one Huffman tree each
  │        → Annex K.2 caps code lengths at 16 → emit BITS/HUFFVAL
  │        → one extra pseudo-symbol with frequency 0 occupies the all-ones codeword, so padding bits
  │          can never be mistaken for a real code
  │
  ├─ entropy-code in MCU order (4:4:4: 3 blocks per MCU; 4:2:0: 6 blocks — 4Y + Cb + Cr)
  │        a 0x00 is appended after every 0xFF
  │
  └─ emit segments: SOI → DQT (3 tables) → SOF0 → DHT (DC + AC) → SOS → entropy data → EOI
```

The quantization tables live in `encoder::example`: `LIGHT_QUANT` for luma, `COLOR_QUANT` for both chroma
components. Both are the standard tables (ITU T.81 Annex K.1) in float form — editing them changes quality.

> The DQT segment length must be `2 + Σ(1 + 64×Pq)`. For three 8-bit tables that is `2 + 3×65 = 197`.
> Forgetting those 2 header bytes shifts every following segment by two and the decoder fails outright with
> "Bogus marker length".

---

## API overview

| Module | Signature | Purpose |
| --- | --- | --- |
| `jpeg::file` | `JpegRaw::new(&[u8]) -> JpegRaw` | owns the byte buffer and the cursor |
| `jpeg::file` | `JpegRaw::parse(&mut self) -> JpegFile` | parses every segment in one go |
| `jpeg::decoder` | `JpegImage::decode(JpegFile) -> Result<JpegImage, String>` | consumes the segment list, produces the component planes |
| `jpeg::decoder` | `JpegImage::to_rgb(&self) -> Result<Vec<u8>, String>` | upsampling + colour conversion, returns an interleaved RGB buffer |
| `jpeg::decoder` | `JpegImage::write_ppm(&self, path) -> Result<(), String>` | writes a P6 file |
| `jpeg::decoder` | `HuffmanDecoder::from_table(&HuffmanTable)` | DHT → lookup-ready canonical codes |
| `jpeg::encoder` | `JpegMeta::new(w, h, resolute)` | allocates the planes and block grids |
| `jpeg::encoder` | `JpegMeta::rgb_in(r, g, b)` | loads pixels and converts to YCbCr |
| `jpeg::encoder` | `JpegMeta::gen_file(&qy, &qcb, &qcr, path)` | full encode pass, writes the file |
| `jpeg::encoder` | `Buf::{dqt, sof0, dht, sos, end}` | byte-level marker emitters |
| `ppm` | `read_ppm_channels(path) -> io::Result<(w, h, r, g, b)>` | reads P3/P6 |

---

## Verification

The decoder was compared pixel by pixel against **libjpeg** (`djpeg`) on `bin/20210617054007202.jpg`
(3840×2400, 4:2:0):

| Reference | Mean abs. error | Max error | Identical channels |
| --- | --- | --- | --- |
| `djpeg -nosmooth` (same nearest-neighbour upsampling) | **0.0096** | 3 | 99.19% |
| `djpeg` (default fancy upsampling) | 0.1559 | 39 | 92.76% |

The residual error in the first row comes from float IDCT versus libjpeg's integer IDCT. The gap in the second
row is confined to chroma edges and is purely a difference in upsampling strategy, not a decoding error.

Encoder: decoding `bin/test1.jpg` (4:4:4, 3840×2400, 285,740 bytes) and comparing against the source PPM gives a
mean absolute error of **1.32**; libjpeg at `-quality 50` scores 1.39 on the same image.

```bash
cargo test --release
# 8 passed: segment parsing (1), decoding (1), DCT/Huffman unit tests (4), encoder end-to-end (2)
```

---

## Known issues / TODO

- **`src/test.rs` and `src/jpeg/encoder.rs` are two implementations of the same encoding logic.** The former
  still hard-codes 1920×1080 inside `serialize()`. Merging them removes a maintenance burden.
- **The encoder does not write APP0/JFIF.** libjpeg, ffmpeg and Windows Photos all open the output, but a strict
  JFIF parser may complain about the missing density fields.
- **4:2:0 uses nearest-neighbour downsampling only**, so hard edges show chroma stair-stepping; the current tests
  also only exercise 4:4:4.
- **The encoder emits no DRI/RST**, so large images have no error resilience. The decoder already supports them.
- **FDCT/IDCT are direct float implementations** (the encoder side is a straight O(N⁴) loop) — correct but slow;
  production use needs an integer or separable fast transform.
- **The decoder is baseline-only**: progressive, arithmetic coding, 12-bit and CMYK/YCCK all return an error.
- **Decoding materialises every component plane in full** (~14 MB for a 3840×2400 4:2:0 image). Very large images
  would need strip-by-strip processing.
- 64 compiler warnings, almost all naming (`F`, `ff`-style locals).

---

## References

- **ITU-T T.81 / ISO/IEC 10918-1** (the JPEG standard)
  - Annex A.2 — DCT / IDCT definitions
  - Annex B.1.1.5 / B.2 — byte stuffing and restart markers in entropy-coded data
  - Annex C — Huffman table specification and decoding procedure
  - Annex F.2.2 — sequential DCT encoding and decoding (DC prediction, AC run-length, EXTEND)
  - Annex K.1 / K.2 / K.3 — example quantization tables, code-length limiting, optimized Huffman tables
- **ITU-T T.871 / ISO/IEC 10918-5** — JFIF file format
- **ITU-R BT.601** — YCbCr ↔ RGB conversion coefficients
