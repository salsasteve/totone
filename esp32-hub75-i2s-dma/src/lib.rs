//! # esp32-hub75-i2s-dma
//!
//! ESP32-S3 display driver and main application for the Totone music visualizer.
//!
//! This crate provides the hardware-specific implementation for running the Totone music
//! visualizer on ESP32-S3. It handles I2S microphone input, HUB75 LED matrix display output,
//! and dual-core task coordination using the Embassy async framework.
//!
//! ## Architecture
//!
//! The application uses ESP32-S3's dual-core architecture:
//!
//! - **Core 0 (Audio Processing)**: Reads I2S samples and performs FFT
//! - **Core 1 (Display & Visualization)**: Updates visualization and drives LED matrix
//!
//! ## Modules
//!
//! - `audio`: I2S microphone reader and FFT processor tasks
//! - `config`: Display and audio configuration constants

#![no_std]

pub mod audio;
pub mod config;
