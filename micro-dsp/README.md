# micro-dsp

A `no_std` compatible DSP (Digital Signal Processing) library for embedded audio analysis.

## Overview

`micro-dsp` provides efficient audio processing functions for real-time spectrum analysis, including:
- Sample normalization (i16 to f32)
- Window functions (Hann window)
- Fast Fourier Transform (FFT) computation
- Magnitude spectrum calculation

## Features

- `no_std` compatible - works in embedded environments
- Hardware-independent - usable on any platform
- Unit testable - includes test suite
- Efficient FFT processing using `microfft`

## Usage

### Basic Example

```rust
use micro_dsp::process_frame;

// Process a frame of 1024 audio samples
let samples: [i16; 1024] = /* ... your audio samples ... */;

match process_frame(&samples) {
    Ok(magnitude_spectrum) => {
        // magnitude_spectrum is [f32; 512]
        // Contains FFT magnitude bins
        for (bin, magnitude) in magnitude_spectrum.iter().enumerate() {
            // Use magnitude for visualization
        }
    }
    Err(e) => {
        // Handle error (usually wrong input size)
    }
}
```

### Processing Pipeline

The library implements a complete audio processing pipeline:

1. **Normalization**: Converts i16 samples to f32 range [-1.0, 1.0]
2. **Windowing**: Applies Hann window to reduce spectral leakage
3. **FFT**: Computes 1024-point real FFT (outputs 512 complex bins)
4. **Magnitude**: Calculates magnitude spectrum from complex FFT output

### Individual Functions

You can also use individual processing steps:

```rust
use micro_dsp::{normalize_samples, apply_hann_window, compute_fft, compute_magnitude};

// Step 1: Normalize
let mut normalized = [0.0f32; 1024];
normalize_samples(&samples, &mut normalized);

// Step 2: Apply window
apply_hann_window(&mut normalized);

// Step 3: Compute FFT
let fft_output = compute_fft(&mut normalized);

// Step 4: Compute magnitude
let magnitude = compute_magnitude(fft_output)?;
```

## Dependencies

- `microfft`: Real FFT implementation
- `micromath`: Math functions for embedded
- `microdsp`: Window function implementations

## Configuration

The library is configured for:
- **FFT Size**: 1024 samples (fixed)
- **Output Bins**: 512 magnitude bins
- **Window Function**: Hann window

To change these, modify the source code directly.

## Testing

Run the test suite:

```bash
cargo test
```

The tests verify:
- Magnitude calculation correctness
- Window function application
- FFT processing pipeline

## Python Utilities

The `python/` directory contains helper scripts for DSP development:

- `generate_q15_cos.py`: Generate Q15 fixed-point cosine tables
- `generate_sine.py`: Generate sine wave test data
- `check_apply_hann_window.py`: Verify window function
- `check_q15cos.py`: Verify Q15 cosine implementation
- `check_arc_cos.py`: Test arccosine functions
- `rand.py`: Random test data generation

See `python/README.md` for detailed usage.

## Performance Considerations

- **FFT Size**: Fixed at 1024 for optimal embedded performance
- **Allocation**: Uses stack arrays, no heap allocation
- **Window Function**: Hann window precomputed for efficiency
- **Magnitude**: Simple sqrt(a² + b²) calculation

## Limitations

- Fixed FFT size (1024 samples)
- Requires exactly 1024 input samples
- Real FFT only (no complex input support)
- No streaming/incremental FFT (processes complete frames)

## Integration

This library is designed to work with:
- `micro-viz`: Visualization library
- `totone`: Main embedded application
- `esp32-hub75-i2s-dma`: ESP32-S3 display driver

See the main [README.md](../README.md) for full project documentation.

