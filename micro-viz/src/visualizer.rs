//! Main visualizer coordinator.
//!
//! This module provides the high-level `Visualizer` struct that orchestrates
//! all components: visualization modes, color modes, effects, and FFT data processing.
//!
//! Note: Due to trait object limitations with generic methods in no_std environments,
//! the visualizer uses an enum-based approach for modes and effects.

use crate::{
    fft_provider::FftDataProvider,
    visualization_mode::{VisualizationMode, VisualizationConfig},
    modes::bars::{BarsMode, SkinnyBarsMode},
    modes::waves::{SineWaveMode, SawtoothWaveMode, SquareWaveMode, ZigZagMode},
};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::Rgb888,
};

/// Enum of available visualization modes.
pub enum VisualizationModeEnum {
    Bars(BarsMode),
    SkinnyBars(SkinnyBarsMode),
    SineWave(SineWaveMode),
    SawtoothWave(SawtoothWaveMode),
    SquareWave(SquareWaveMode),
    ZigZag(ZigZagMode),
}

impl VisualizationMode for VisualizationModeEnum {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        match self {
            VisualizationModeEnum::Bars(mode) => mode.render(target, fft_data, config),
            VisualizationModeEnum::SkinnyBars(mode) => mode.render(target, fft_data, config),
            VisualizationModeEnum::SineWave(mode) => mode.render(target, fft_data, config),
            VisualizationModeEnum::SawtoothWave(mode) => mode.render(target, fft_data, config),
            VisualizationModeEnum::SquareWave(mode) => mode.render(target, fft_data, config),
            VisualizationModeEnum::ZigZag(mode) => mode.render(target, fft_data, config),
        }
    }

    fn update(&mut self, frame_count: u32) {
        match self {
            VisualizationModeEnum::Bars(mode) => mode.update(frame_count),
            VisualizationModeEnum::SkinnyBars(mode) => mode.update(frame_count),
            VisualizationModeEnum::SineWave(mode) => mode.update(frame_count),
            VisualizationModeEnum::SawtoothWave(mode) => mode.update(frame_count),
            VisualizationModeEnum::SquareWave(mode) => mode.update(frame_count),
            VisualizationModeEnum::ZigZag(mode) => mode.update(frame_count),
        }
    }
}

/// Main visualizer that coordinates all visualization components.
///
/// This struct provides a high-level API for spectrum visualization.
/// It manages the visualization mode, configuration, and FFT data processing.
pub struct Visualizer {
    mode: VisualizationModeEnum,
    config: VisualizationConfig,
    frame_count: u32,
}

impl Visualizer {
    /// Creates a new visualizer with a visualization mode and configuration.
    ///
    /// # Arguments
    /// * `mode` - The visualization mode to use
    /// * `config` - Configuration for the visualization
    pub fn new(mode: VisualizationModeEnum, config: VisualizationConfig) -> Self {
        Self {
            mode,
            config,
            frame_count: 0,
        }
    }

    /// Updates and renders the visualization.
    ///
    /// This is the main update loop that should be called once per frame.
    ///
    /// # Arguments
    /// * `target` - The drawing target (framebuffer)
    /// * `fft_data` - FFT magnitude data provider
    ///
    /// # Returns
    /// Result indicating success or drawing error
    pub fn update<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
    ) -> Result<(), D::Error> {
        // Update mode (for animated modes)
        self.mode.update(self.frame_count);

        // Render the visualization mode
        self.mode.render(target, fft_data, &mut self.config)?;

        self.frame_count = self.frame_count.wrapping_add(1);
        Ok(())
    }

    /// Sets a new visualization mode.
    ///
    /// # Arguments
    /// * `mode` - The new visualization mode
    pub fn set_mode(&mut self, mode: VisualizationModeEnum) {
        self.mode = mode;
    }

    /// Gets the current frame count.
    pub fn frame_count(&self) -> u32 {
        self.frame_count
    }
}

