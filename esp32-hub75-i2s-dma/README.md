# esp32-hub75-i2s-dma

ESP32-S3 display driver and main application for the Totone music visualizer.

## Overview

This crate provides the hardware-specific implementation for running the Totone music visualizer on ESP32-S3. It handles:
- I2S microphone input (INMP441)
- HUB75 LED matrix display output
- Dual-core task coordination
- Audio processing and visualization pipeline

## Hardware Setup

### Required Components

- ESP32-S3 development board
- INMP441 I2S microphone module
- HUB75 LED matrix panel (64x256 or compatible)
- Level shifters (for 5V LED matrix signals)

### Pin Connections

#### I2S Microphone (INMP441)

| INMP441 | ESP32-S3 | GPIO |
|---------|----------|------|
| VDD     | 3.3V     | -    |
| GND     | GND      | -    |
| WS/LRCL | GPIO5    | 5    |
| SCK/BCK | GPIO4    | 4    |
| SD      | GPIO6    | 6    |

#### HUB75 LED Matrix

The pin configuration is defined in `src/bin/async_main.rs`:

| HUB75 | ESP32-S3 | GPIO | Function |
|-------|----------|------|----------|
| R1    | GPIO3    | 3    | Red row 1 |
| G1    | GPIO43   | 43   | Green row 1 |
| B1    | GPIO46   | 46   | Blue row 1 |
| R2    | GPIO9    | 9    | Red row 2 |
| G2    | GPIO44   | 44   | Green row 2 |
| B2    | GPIO10   | 10   | Blue row 2 |
| A     | GPIO11   | 11   | Address 0 |
| B     | GPIO2    | 2    | Address 1 |
| C     | GPIO12   | 12   | Address 2 |
| D     | GPIO42   | 42   | Address 3 |
| E     | GPIO1    | 1    | Address 4 |
| CLK   | GPIO13   | 13   | Clock |
| LAT   | GPIO41   | 41   | Latch |
| OE    | GPIO14   | 14   | Blank/Enable |

**Note**: GPIOs may need level shifters (3.3V → 5V) depending on your LED matrix.

## Configuration

### Display Configuration

Edit `src/config.rs`:

```rust
pub const ROWS: usize = 64;        // Matrix height
pub const COLS: usize = 256;       // Matrix width
pub const BITS: u8 = 3;            // Color bit depth (lower = faster)
pub const HUB75_CLOCK_MHZ: u32 = 20; // Clock rate
```

### Audio Configuration

Edit `src/config.rs`:

```rust
pub const SAMPLE_RATE_HZ: u32 = 48_000; // I2S sample rate
pub const FFT_SIZE: usize = 1024;       // FFT window size
```

### Visualization Configuration

Edit `src/bin/async_main.rs`:

```rust
const BAR_WIDTH: u16 = 7;  // Width of each frequency bar
```

## Building

### Prerequisites

1. Install ESP Rust toolchain (see main [README.md](../README.md))
2. Ensure probe-rs or espflash is installed for flashing

### Build Commands

**Debug build:**
```bash
cargo build
```

**Release build:**
```bash
cargo build --release
```

The release build optimizes for size (`opt-level = 's'`) to fit in flash.

## Flashing

### Using probe-rs (Default)

```bash
cargo run --release
```

This builds and flashes automatically. Serial output is available via probe-rs.

### Using espflash

Edit `.cargo/config.toml` and uncomment:
```toml
runner = "espflash flash --monitor --baud 115200"
```

Then:
```bash
cargo run --release
```

### Manual Flashing

```bash
# Build first
cargo build --release

# Flash with probe-rs
probe-rs run --chip esp32s3 target/xtensa-esp32s3-none-elf/release/esp32-hub75-i2s-dma

# Or with espflash
espflash flash --chip esp32s3 target/xtensa-esp32s3-none-elf/release/esp32-hub75-i2s-dma
```

## Architecture

### Dual-Core Design

The application uses ESP32-S3's dual-core architecture:

**Core 0 (Audio Processing):**
- `microphone_reader` task: Reads I2S samples via DMA
- `audio_processor` task: Performs FFT and sends bins to Core 1

**Core 1 (Display & Visualization):**
- `display_task`: Updates visualization from FFT data
- `hub75_task`: Drives LED matrix via DMA

### Communication

- **Samples Signal**: Core 0 microphone → Core 0 processor
- **FFT Channel**: Core 0 processor → Core 1 display task
- **Framebuffer Signals**: Core 1 display ↔ Core 1 hub75 (double buffering)

### Framebuffers

Double buffering is used for smooth display updates:
- Display task renders to one buffer
- Hub75 task displays the other buffer
- Buffers swap after each frame

## Task Overview

### microphone_reader

- Reads I2S audio samples using DMA circular buffer
- Extracts mono samples from stereo I2S stream
- Signals complete frames (1024 samples) to audio processor

**Location:** `src/audio.rs`

### audio_processor

- Receives 1024-sample audio frames
- Normalizes, windows, and computes FFT
- Sends 512-bin magnitude spectrum to display task

**Location:** `src/audio.rs`

### display_task

- Receives FFT magnitude bins from audio processor
- Updates `BarGraph` visualization
- Renders to framebuffer
- Swaps framebuffer with hub75 task

**Location:** `src/bin/async_main.rs`

### hub75_task

- Waits for rendered framebuffer from display task
- Drives HUB75 LED matrix via DMA
- Returns empty buffer to display task

**Location:** `src/bin/async_main.rs`

## Memory Configuration

Edit `src/config.rs`:

```rust
pub const HEAP_SIZE: usize = 32 * 1024;        // 32KB heap
pub const CORE1_STACK_SIZE: usize = 16 * 1024; // 16KB stack for Core 1
```

Increase if you encounter allocation failures or stack overflows.

## Troubleshooting

### No Display Output

1. Check HUB75 wiring and GPIO configuration
2. Verify level shifters are working
3. Check matrix dimensions match configuration
4. Verify clock and data signals with oscilloscope

### No Audio Processing

1. Check I2S microphone wiring
2. Verify sample rate matches microphone capability (INMP441: 16-48kHz)
3. Check DMA buffer sizes
4. Monitor serial output for errors

### Low Frame Rate

1. Reduce `BITS` in config (lower bit depth = faster refresh)
2. Reduce matrix dimensions
3. Optimize FFT size or visualization complexity

### Memory Issues

1. Increase `HEAP_SIZE` in config
2. Check stack sizes for tasks
3. Use `cargo size` to analyze binary
4. Enable `defmt` logging to see memory usage

### Build Issues

1. Ensure ESP toolchain is installed and sourced
2. Check `rust-toolchain.toml` specifies `channel = "esp"`
3. Verify `.cargo/config.toml` has correct target

## Logging

Enable defmt logging by setting in `.cargo/config.toml`:

```toml
[env]
DEFMT_LOG="info"
```

Or use feature flags:
```bash
cargo run --release --features logging
```

Logs are available via probe-rs or espflash monitor.

## Dependencies

- `esp-hal`: ESP32-S3 hardware abstraction
- `embassy-executor`: Async task executor
- `embassy-sync`: Inter-task communication
- `esp-hub75`: HUB75 display driver
- `micro-dsp`: Audio processing library
- `micro-viz`: Visualization library

See `Cargo.toml` for full dependency list.

## References

- [ESP Rust Book](https://esp-rs.github.io/book/)
- [Embassy Documentation](https://embassy.dev/)
- [esp-hub75 Driver](https://github.com/liebman/esp-hub75)
- Main project [README.md](../README.md)

