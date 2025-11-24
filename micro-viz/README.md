# micro-viz

A `no_std` compatible visualization library for embedded graphics displays.

## Overview

`micro-viz` provides hardware-independent visualization components for real-time audio spectrum visualization. It implements animated bar graphs, color modes, and spectrum aggregation strategies.

## Features

- `no_std` compatible - works in embedded environments
- Hardware-independent - uses `embedded-graphics` trait system
- Multiple color modes (Spectrum, Gradient, Dynamic FFT, etc.)
- Smooth animation with configurable interpolation
- Flexible bin aggregation strategies (Even, Exponential)
- Value animator with easing functions

## Components

### BarGraph

The main visualization component that draws frequency bars on a display.

**Features:**
- Configurable screen dimensions and bar width
- Multiple color modes
- Smooth animation between FFT updates
- Rolling max for dynamic range scaling
- Multiple bin summary strategies (Max, Average, etc.)

**Usage:**

```rust
use micro_viz::BarGraph;
use embedded_graphics::prelude::*;

// Create a bar graph for a 256x64 display with 7px wide bars
let mut bar_graph = BarGraph::new(256, 64, 7);

// Update with FFT magnitude data (512 bins)
let fft_bins: [f32; 512] = /* ... your FFT magnitude data ... */;

// Update and draw on a framebuffer
bar_graph.update(&mut framebuffer, &fft_bins)?;
```

### SpectralBandAggregator

Maps FFT bins to display bars using configurable spread modes.

**Spread Modes:**
- `Even`: Equal-width frequency bands
- `Exponential`: Logarithmic frequency distribution

**Usage:**

```rust
use micro_viz::{SpectralBandAggregator, BandSpreadModeConfig};

let aggregator = SpectralBandAggregator::new(
    512,  // Total FFT bins
    8,    // Number of bars
    BandSpreadModeConfig::Exponential { exp_factor: 7.0 }
);

let ranges = aggregator.band_ranges();
// ranges contains (start, end) indices for each bar
```

### SpectrumValueAnimator

Smoothly interpolates bar heights between FFT updates.

**Features:**
- Configurable interpolation steps
- Easing functions for smooth transitions
- Handles frame-by-frame updates

### BinSummaryStrategy

Strategies for aggregating FFT bins within a frequency band:

- `Max`: Use maximum value in band
- `Average`: Use average value in band
- `Sum`: Sum all values in band

**Usage:**

```rust
use micro_viz::BinSummaryStrategy;

let strategy = BinSummaryStrategy::Max;
let value = strategy.calculate(&bins[start..end]);
```

## Color Modes

The library supports multiple color modes:

- **Spectrum**: Color wheel distribution across bars
- **Gradient**: Intensity-based grayscale
- **StaticPalette**: Fixed color palette
- **DynamicFFT**: FFT magnitude-based colors
- **ShiftingSpectrum**: Animated color wheel

## Configuration

### Display Dimensions

Configure in `BarGraph::new()`:
- `screen_width`: Display width in pixels
- `screen_height`: Display height in pixels
- `bar_width`: Width of each frequency bar

### Animation

Control via `SpectrumValueAnimator`:
- `interpolation_steps`: Number of frames between updates
- `max_display_height`: Maximum bar height in pixels

### Aggregation

Configure via `SpectralBandAggregator`:
- Exponential factor for logarithmic distribution
- Number of bars vs. FFT bin count

## Dependencies

- `embedded-graphics`: Hardware abstraction for displays
- `micromath`: Math functions for embedded
- `defmt` (optional): Logging support

## Testing

Run tests:

```bash
cargo test
```

The library includes integration tests that verify:
- Bin aggregation correctness
- Animation interpolation
- Color calculations

## Simulator

Use `micro-viz-simulator` to test without hardware:

```bash
cd ../micro-viz-simulator
cargo run --release
```

The simulator creates a desktop window showing the visualization with synthetic audio data.

## Performance Considerations

- Uses stack arrays where possible (limited allocation)
- Frame-by-frame updates (no double buffering in library)
- Efficient color calculations using bit operations
- Precomputed bin ranges for aggregation

## Integration

This library is designed to work with:
- `micro-dsp`: Provides FFT magnitude data
- `esp32-hub75-i2s-dma`: ESP32-S3 display driver
- Any `embedded-graphics` compatible display driver

See the main [README.md](../README.md) for full project documentation.

## Notes

- The library assumes FFT provides 512 magnitude bins
- Bar count is calculated automatically from screen dimensions
- Animation smooths transitions between FFT updates (typically slower than frame rate)

