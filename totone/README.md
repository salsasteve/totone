# totone

Core embedded control and FFT logic for the Totone music visualizer.

## Overview

This crate provides the core embedded application logic for audio processing and visualization. It's the original implementation that has been refactored into separate libraries (`micro-dsp` and `micro-viz`).

## Status

This crate is currently in a transitional state. The main active development happens in:
- `esp32-hub75-i2s-dma/`: Main ESP32-S3 application
- `micro-dsp/`: Audio processing library
- `micro-viz/`: Visualization library

## Legacy Code

Some components in this crate may be deprecated or superseded by the modular libraries. Check the main [README.md](../README.md) for the current recommended usage.

## Features

- FFT processing logic
- Audio visualization control
- Embedded task coordination

## Usage

This crate is primarily used as a dependency reference. For active development, see:
- `esp32-hub75-i2s-dma/` for the main application
- `micro-dsp/` for DSP functions
- `micro-viz/` for visualization

## Documentation

See the main project [README.md](../README.md) for comprehensive documentation.
