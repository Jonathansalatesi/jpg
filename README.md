# Rust JPEG Encoder

A from-scratch JPEG (JFIF, baseline DCT) encoder written in pure Rust. No external image codec dependencies — every stage from RGB → YCbCr → DCT → quantization → Huffman coding → byte stream is implemented in this crate.

The encoder currently produces **baseline SOF0 JPEG** with **4:2:0 chroma subsampling** and **optimized Huffman tables** generated per image (with Annex K.2 code-length limiting).

---

## Features

- Baseline DCT JPEG (SOF0), 8-bit precision
- RGB → YCbCr color conversion
- 4:2:0 chroma subsampling
- Forward 2D DCT (8×8, floating point)
- Quantization with configurable tables (DQT)
- Zigzag scan + run-length encoding for AC coefficients
- DC differential coding
- **Optimized Huffman table construction** per image:
  - Standard Huffman tree from symbol frequencies
  - Annex K.2 code-length limiting (max 16 bits)
  - Pseudo-symbol 256 to avoid all-ones Huffman codes
  - Canonical Huffman code assignment
- MSB-first bit writer with 0xFF byte stuffing
- Full JPEG marker output: `SOI`, `DQT`, `SOF0`, `DHT`, `SOS`, entropy data, `EOI`

---

## Architecture

```
src/
├── example.rs        # Top-level encode pipeline + marker writer (Jpeg struct)
│   ├── serialize()   # Main entry: RGB → YCbCr → blocks → entropy → bytes
│   ├── Binary        # MSB-first bit writer with 0xFF stuffing
│   └── Jpeg          # Marker segment builders (DQT, SOF0, DHT, SOS)
├── encode.rs         # DCT, quantization, extend(), run-length series()
└── jpg_huffman.rs    # Huffman tree, K.2 limiting, canonical coder (Coder)
```

### Key modules

| Module                  | Responsibility                                               |
| ----------------------- | ------------------------------------------------------------ |
| `encode::dct`           | 8×8 forward DCT                                              |
| `encode::round`         | Quantization with symmetric rounding                         |
| `encode::extend`        | JPEG VLI category/value for a coefficient                    |
| `encode::series`        | Zigzag + RLE, emits `(run<<4 \| size, value)` tuples incl. ZRL/EOB |
| `jpg_huffman::compress` | Build optimized code-length table with K.2 limiting          |
| `jpg_huffman::Coder`    | Canonical Huffman code assignment (`min_code`, `max_code`, `ptr_addr`) |
| `example::Binary`       | Bit-level writer, MSB-first, with byte stuffing              |
| `example::Jpeg`         | Writes marker segments and entropy payload                   |

---

## Encoding Pipeline

```
RGB pixels
  │
  ├─ RGB → YCbCr
  │
  ├─ Chroma subsample (4:2:0):  Cb, Cr → half resolution
  │
  ├─ Split into 8×8 blocks (Y padded to MCU boundary, edge-replicate)
  │
  ├─ Per block:
  │     level shift (−128)
  │     → DCT
  │     → quantize (divide by Q, round)
  │     → zigzag
  │     → DC diff (predictive), AC RLE (ZRL / EOB)
  │
  ├─ Scan all symbols:
  │     count DC and AC frequency tables
  │     add pseudo-symbol (freq 1) to each
  │     build Huffman tree → BITS + HUFFVAL via K.2 limiting
  │     remove pseudo-symbol
  │
  ├─ Entropy-encode in MCU order:
  │     for each MCU: 4 Y blocks, 1 Cb, 1 Cr
  │     write Huffman code + VLI bits (MSB first)
  │     stuff 0x00 after every 0xFF
  │
  └─ Emit JPEG:
        SOI
        DQT   (quantization table, zigzag order)
        SOF0  (dimensions, 3 components, 2×2 / 1×1 / 1×1 sampling)
        DHT   (DC table 0, AC table 1)
        SOS   (3 components, Ss=0, Se=63, Ah/Al=0)
        [entropy data]
        EOI
```

---

## Build & Run

```bash
cargo build --release
cargo test --release
```

The integration test writes a sample JPEG to `./bin/test.jpg`:

```rust
#[test]
fn test_example() {
    let jpeg = example::serialize();
    std::fs::write("./bin/test.jpg", &jpeg.data).unwrap();
}
```

Run it:

```bash
mkdir -p bin
cargo test --release test_example -- --nocapture
```

Then open `bin/test.jpg` in any image viewer.

---

## Configuration

### Quantization table

Edit the `QUANTIZATION` constant in `example.rs`. It is a natural-order `[[f64; 8]; 8]`; the writer reorders it to zigzag when emitting DQT.

```rust
const QUANTIZATION: [[f64; 8]; 8] = [[1.0; 8]; 8]; // near-lossless
```

### Image source

Replace `photo()` to encode a different image. It must return three `Vec<Vec<[u8; WIDTH]>>` planes (R, G, B).

### Dimensions

`WIDTH` and `HEIGHT` are compile-time constants in `example.rs`. They must match the size of the returned planes.

---

## Implementation Notes

### Annex K.2 code-length limiting

`jpg_huffman::limit_code_length` implements the JPEG K.2 procedure. If the standard Huffman tree produces a code longer than 16 bits, the algorithm rebalances the `bits[]` array (counts per code length) by pairing off the longest codes and promoting one level up, then taking a short code and adding two codes at `j+1`. The loop terminates when no code length exceeds 16.

### Pseudo-symbol 256

A pseudo-symbol with frequency 1 is inserted into both DC and AC symbol tables before tree construction. Because it has the lowest frequency, it takes the **last code of the longest class**, which is the all-ones code. After tree construction it is removed from `BITS`/`HUFFVAL`, leaving the all-ones code unused — this prevents padding bits (`1`s) from being mistaken for a real Huffman code.

### Byte stuffing

Every `0xFF` byte emitted in the entropy-coded segment is followed by `0x00`. This is required so decoders don't confuse entropy data with marker prefixes. `Binary::filter()` performs this on the final byte buffer.

### Bit order

JPEG entropy coding is **MSB-first**. `Binary::bit` writes to `1 << (7 - offset)`, and `Binary::bits` iterates the value from high to low.

---

## Known Limitations

- **4:2:0 chroma subsampling** uses nearest-neighbor (top-left pixel). This produces visible chroma stair-stepping on hard edges. Switching to 4:4:4 (all sampling factors 1×1) or averaging subsampling would improve edge quality.
- **No restart markers (DRI/RST)** are emitted. Fine for small images; large images may benefit from restart intervals for error resilience and parallel decoding.
- **No EXIF / APP0/JFIF header** is written. Most viewers accept headerless JFIF, but strict decoders may prefer an APP0 marker.
- **Baseline only** — no progressive (SOF2), no arithmetic coding, no 12-bit precision.
- **DCT is floating-point** (direct O(N⁴) loop). Correct but slow; a fast integer or separable DCT would be needed for production use.
- **Single-pass scan only** — no multiple scans, no spectral selection (Ss/Se fixed at 0/63).
- **Encoder only** — no decoder.

---

## References

- **ITU-T T.81 / ISO/IEC 10918-1** — the JPEG standard, especially:
  - Annex A.2.1 — MCU definition
  - Annex C — Huffman table specification and decoding procedure
  - Annex F — sequential DCT encoding
  - Annex K.1 — standard quantization tables
  - Annex K.2 — code-length limiting procedure
  - Annex K.3 — optimized Huffman table generation
- **ITU-T T.871 / ISO/IEC 10918-5** — JFIF file format
