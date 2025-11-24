//! Visualization mode trait and base implementations.
//!
//! This module provides the trait for different visualization modes (bars, waves, etc.)
//! and base implementations. Each mode defines how FFT data is visualized on screen.

use crate::{
    fft_provider::FftDataProvider,
    spectral_band_aggregator::SpectralBandAggregator,
    spectrum_value_animator::SpectrumValueAnimator,
    bin_summary_strategy::BinSummaryStrategy,
};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::Rgb888,
};

/// Trait for visualization modes.
///
/// Each visualization mode defines how FFT magnitude data is rendered on screen.
/// Modes can implement different shapes (bars, waves, circles, etc.) and their own
/// animation behavior.
pub trait VisualizationMode {
    /// Renders the visualization to the target.
    ///
    /// # Arguments
    /// * `target` - The drawing target (framebuffer)
    /// * `fft_data` - FFT magnitude data provider
    /// * `config` - Visualization configuration (mutable for animator updates)
    ///
    /// # Returns
    /// Result indicating success or drawing error
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error>;

    /// Updates internal state for animated modes.
    ///
    /// Called once per frame before rendering.
    /// Default implementation does nothing (for static modes).
    fn update(&mut self, _frame_count: u32) {
        // Default: no-op for static modes
    }

    /// Checks if a redraw is needed.
    ///
    /// Some modes may only need redraws when data changes.
    /// Default returns true (always redraw).
    fn needs_redraw(&self) -> bool {
        true
    }
}

/// Configuration for visualization modes.
pub struct VisualizationConfig {
    /// Screen dimensions
    pub screen_width: u16,
    pub screen_height: u16,
    /// Number of visualization elements (bars, wave points, etc.)
    pub num_elements: usize,
    /// Bar width (for bar modes)
    pub bar_width: u16,
    /// Bar spacing (for bar modes)
    pub bar_spacing: u32,
    /// Spectral band aggregator for mapping FFT bins to elements
    pub band_aggregator: SpectralBandAggregator,
    /// Bin summary strategy for aggregating bin values
    pub bin_strategy: BinSummaryStrategy,
    /// Value animator for smooth transitions
    pub animator: SpectrumValueAnimator,
    /// Rolling max tracker for dynamic scaling
    pub rolling_max: f32,
    /// Rolling max decay factor
    pub rolling_max_decay: f32,
}

impl VisualizationConfig {
    /// Creates a new configuration with default values.
    pub fn new(screen_width: u16, screen_height: u16, bar_width: u16) -> Self {
        let bar_spacing = 1;
        let num_elements = (screen_width / (bar_width + bar_spacing as u16)).max(1) as usize;
        let num_bins = 512;

        let band_aggregator = SpectralBandAggregator::new(
            num_bins,
            num_elements,
            crate::spectral_band_aggregator::BandSpreadModeConfig::Exponential {
                exp_factor: 7.0,
            },
        );

        let initial_height = 1.0f32;
        let interpolation_steps = 5;
        let max_display_height = screen_height.saturating_sub(1).max(1);

        let animator = SpectrumValueAnimator::new(
            num_elements,
            initial_height,
            interpolation_steps,
            max_display_height,
        );

        Self {
            screen_width,
            screen_height,
            num_elements,
            bar_width,
            bar_spacing,
            band_aggregator,
            bin_strategy: BinSummaryStrategy::Max,
            animator,
            rolling_max: 1.0,
            rolling_max_decay: 0.9,
        }
    }

    /// Creates a builder for custom configuration.
    pub fn builder(screen_width: u16, screen_height: u16, bar_width: u16) -> VisualizationConfigBuilder {
        VisualizationConfigBuilder::new(screen_width, screen_height, bar_width)
    }
}

/// Builder for visualization configuration.
pub struct VisualizationConfigBuilder {
    config: VisualizationConfig,
}

impl VisualizationConfigBuilder {
    fn new(screen_width: u16, screen_height: u16, bar_width: u16) -> Self {
        Self {
            config: VisualizationConfig::new(screen_width, screen_height, bar_width),
        }
    }

    /// Sets the bin summary strategy.
    pub fn bin_strategy(mut self, strategy: BinSummaryStrategy) -> Self {
        self.config.bin_strategy = strategy;
        self
    }

    /// Sets the rolling max decay factor.
    pub fn rolling_max_decay(mut self, decay: f32) -> Self {
        self.config.rolling_max_decay = decay.max(0.0).min(1.0);
        self
    }

    /// Sets the animation interpolation steps.
    pub fn animation_steps(mut self, steps: u32) -> Self {
        let max_display_height = self.config.screen_height.saturating_sub(1).max(1);
        self.config.animator = SpectrumValueAnimator::new(
            self.config.num_elements,
            1.0f32,
            steps,
            max_display_height,
        );
        self
    }

    /// Sets the band spread mode.
    pub fn band_spread(mut self, mode: crate::spectral_band_aggregator::BandSpreadModeConfig) -> Self {
        self.config.band_aggregator = SpectralBandAggregator::new(
            512,
            self.config.num_elements,
            mode,
        );
        self
    }

    /// Builds the configuration.
    pub fn build(self) -> VisualizationConfig {
        self.config
    }
}

