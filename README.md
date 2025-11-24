# Totone - ESP32-S3 LED Music Visualizer

A real-time audio spectrum analyzer for ESP32-S3 that processes audio input from an I2S microphone and visualizes the frequency spectrum on a HUB75 LED matrix display.

## Overview

Totone is a multi-crate Rust workspace for embedded audio visualization on ESP32-S3. It processes audio in real-time using FFT (Fast Fourier Transform) and displays frequency bands as animated bars on an LED matrix.

### Features

- Real-time audio spectrum analysis using FFT
- Dual-core processing (Core 0: Audio, Core 1: Display)
- Multiple visualization modes and color schemes
- Smooth animation with configurable interpolation
- Hardware-independent DSP and visualization libraries
- Simulator for testing without hardware

## Project Structure

```
totone/
├── esp32-hub75-i2s-dma/    # ESP32-S3 display driver and main application
├── totone/                  # Core embedded control & FFT logic (legacy)
├── micro-dsp/               # no_std DSP library (FFT processing)
├── micro-viz/               # no_std visualization library
└── micro-viz-simulator/     # Desktop simulator for testing
```

See [LIBRARY_LAYOUT.md](./LIBRARY_LAYOUT.md) for a detailed architecture diagram.

## Prerequisites

### Required Tools

1. **Rust with ESP Toolchain**
   - Install Rust: https://rustup.rs/
   - Install ESP Rust toolchain: https://esp-rs.github.io/book/installation/

   Quick setup:
   ```bash
   cargo install espup
   espup install
   source $HOME/export-esp.sh  # On Windows: espup.bat
   ```

2. **Python 3.12+** (for helper scripts)
   - Python is needed for utility scripts in `micro-dsp/python/`
   - Install Poetry if using the Python scripts:
     ```bash
     pip install poetry
     ```

3. **SDL2** (for simulator only, optional)
   - Required only if you want to run the `micro-viz-simulator`
   - **macOS**: Install via Homebrew:
     ```bash
     brew install sdl2
     ```
     After installation, if you get a "library 'SDL2' not found" error, set these environment variables:
     ```bash
     export PKG_CONFIG_PATH="/opt/homebrew/opt/sdl2/lib/pkgconfig"
     export LIBRARY_PATH="/opt/homebrew/opt/sdl2/lib"
     ```
     The simulator's `.cargo/config.toml` should handle this automatically, but you may need to set these manually.
   - **Linux**: Install via package manager:
     ```bash
     # Ubuntu/Debian
     sudo apt-get install libsdl2-dev
     # Fedora/RHEL
     sudo dnf install SDL2-devel
     ```
   - **Windows**: Install from [SDL2 website](https://www.libsdl.org/download-2.0.php) or use vcpkg

4. **Hardware Tools** (for flashing)
   - **Option A**: `probe-rs` (recommended)
     ```bash
     cargo install probe-rs --features cli
     ```
   - **Option B**: `espflash`
     ```bash
     cargo install espflash
     ```

### Hardware Requirements

- ESP32-S3 development board
- INMP441 or compatible I2S microphone
- HUB75 LED matrix panel (64x256 or compatible)
- Wiring connections (see hardware setup in `esp32-hub75-i2s-dma/README.md`)

## Building the Project

### Build Configuration

The project uses ESP Rust toolchain specified in `rust-toolchain.toml`. Each crate that targets ESP32-S3 uses the `esp` toolchain channel.

### Building Individual Crates

**Build the main application:**
```bash
cd esp32-hub75-i2s-dma
cargo build --release
```

**Build the simulator (for testing without hardware):**
```bash
cd micro-viz-simulator
cargo build --release
cargo run --release
```

**Note**: The simulator requires SDL2. See Prerequisites section for installation instructions.

If you get a "library 'SDL2' not found" error on macOS after installing SDL2 via Homebrew, you may need to set environment variables:

```bash
export PKG_CONFIG_PATH="/opt/homebrew/opt/sdl2/lib/pkgconfig"
export LIBRARY_PATH="/opt/homebrew/opt/sdl2/lib"
```

Alternatively, the simulator has a `.cargo/config.toml` that should configure this automatically. If it doesn't work, try the environment variables above.

**Build the DSP library:**
```bash
cd micro-dsp
cargo build
```

**Build the visualization library:**
```bash
cd micro-viz
cargo build
```

### Clean Build Artifacts

To clean all build artifacts:
```bash
cargo clean
```

Or clean individual crates:
```bash
cd <crate-directory>
cargo clean
```

## Flashing to ESP32-S3

### Using probe-rs (Recommended)

The project is configured to use `probe-rs` by default (see `.cargo/config.toml`):

```bash
cd esp32-hub75-i2s-dma
cargo run --release
```

This will:
1. Build the project in release mode
2. Flash the binary to ESP32-S3
3. Start execution and display logs

### Using espflash

To use `espflash` instead, edit `.cargo/config.toml` and uncomment:
```toml
runner = "espflash flash --monitor --baud 115200"
```

Then:
```bash
cd esp32-hub75-i2s-dma
cargo run --release
```

### Manual Flashing

You can also flash manually:
```bash
# Build first
cargo build --release

# Flash with probe-rs
probe-rs run --chip esp32s3 target/xtensa-esp32s3-none-elf/release/esp32-hub75-i2s-dma

# Or with espflash
espflash flash --chip esp32s3 target/xtensa-esp32s3-none-elf/release/esp32-hub75-i2s-dma
```

## Running the Simulator

Test the visualization logic without hardware:

```bash
cd micro-viz-simulator
cargo run --release
```

**Note**: The simulator requires SDL2 to be installed. See the Prerequisites section above for installation instructions for your platform.

The simulator creates a window showing the visualization with synthetic audio data. Use this to:
- Test visualization modes and parameters
- Debug animation and color logic
- Verify DSP processing pipeline

## Configuration

### Display Configuration

Edit `esp32-hub75-i2s-dma/src/config.rs` to configure:
- Matrix dimensions (ROWS, COLS)
- Bit depth (BITS) - affects color depth and refresh rate
- Hub75 clock rate

### Audio Configuration

Edit `esp32-hub75-i2s-dma/src/config.rs`:
- Sample rate (SAMPLE_RATE_HZ)
- FFT size (FFT_SIZE)
- Number of visualization bands

### Visualization Configuration

Edit `esp32-hub75-i2s-dma/src/bin/async_main.rs`:
- `BAR_WIDTH`: Width of each frequency bar
- Visualization mode (see `micro-viz` crate)

## Development

### Running Tests

**Test the DSP library:**
```bash
cd micro-dsp
cargo test
```

**Test the visualization library:**
```bash
cd micro-viz
cargo test
```

### Debugging

1. **Enable defmt logging**: Set `DEFMT_LOG=info` in environment or `.cargo/config.toml`
2. **Monitor serial output**: Use `probe-rs` or `espflash` monitor
3. **Use simulator**: Test logic on desktop first

### Project Architecture

The system uses a dual-core architecture:

- **Core 0 (Audio Processing)**:
  - `microphone_reader` task: Reads I2S audio samples
  - `audio_processor` task: Performs FFT and sends bins to Core 1

- **Core 1 (Visualization & Display)**:
  - `display_task`: Updates visualization from FFT data
  - `hub75_task`: Drives LED matrix via DMA

Communication between cores uses Embassy channels and signals.

## Python Scripts

### bin_maker.py

Generates precomputed FFT bin indices for efficient bar mapping. Run:

```bash
python bin_maker.py
```

Configure `USER_NUM_BARS` and `TOTAL_FFT_BINS_PARAM` at the top of the script.

### micro-dsp/python/

Helper scripts for DSP development:
- `generate_q15_cos.py`: Generate Q15 fixed-point cosine tables
- `generate_sine.py`: Generate sine wave test data
- `check_apply_hann_window.py`: Verify window function implementation

See `micro-dsp/python/README.md` for details.

## Troubleshooting

### Build Issues

- **"toolchain not found"**: Run `espup install` and source the export script
- **"linker script not found"**: Ensure `build.rs` is present in crate root
- **"no_std" errors**: Ensure crate has `#![no_std]` and proper alloc setup

### Runtime Issues

- **No display output**: Check HUB75 wiring and GPIO configuration
- **No audio processing**: Verify I2S microphone connections and sample rate
- **Low frame rate**: Reduce bit depth (`BITS`) or optimize FFT size

### Memory Issues

- Increase `HEAP_SIZE` in `config.rs` if allocation fails
- Check stack sizes if tasks crash
- Use `cargo size` to analyze binary size

## Contributing

1. Follow Rust embedded best practices
2. Keep `no_std` compatibility in `micro-dsp` and `micro-viz`
3. Add documentation for public APIs
4. Test with simulator before hardware testing

## License

[Specify your license here]

## References

- [ESP Rust Book](https://esp-rs.github.io/book/)
- [Embassy Async Framework](https://embassy.dev/)
- [embedded-graphics](https://docs.rs/embedded-graphics/)
- [HUB75 Protocol](https://github.com/liebman/esp-hub75)
