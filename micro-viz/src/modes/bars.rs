//! Bar visualization mode implementations.
//!
//! This module provides bar-based visualization modes, including standard bars,
//! skinny bars, fat bars, and rounded bars.

use crate::{
    fft_provider::FftDataProvider,
    visualization_mode::{VisualizationMode, VisualizationConfig},
    color_mode::{ColorMode, ColorContext},
    renderer,
    scaling::{RollingMaxTracker, ScalingContext, ScalingStrategy, LinearRollingMaxScaling},
};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::Rgb888,
    prelude::*,
};
use alloc::vec;
use alloc::boxed::Box;

/// Standard bars visualization mode.
///
/// Renders FFT data as vertical bars with configurable width.
pub struct BarsMode {
    color_mode: Box<dyn ColorMode>,
    rolling_max_tracker: RollingMaxTracker,
    scaling_strategy: LinearRollingMaxScaling,
    frame_count: u32,
}

impl BarsMode {
    /// Creates a new bars mode with a default color mode.
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            color_mode,
            rolling_max_tracker: RollingMaxTracker::new(1.0, 0.9),
            scaling_strategy: LinearRollingMaxScaling,
            frame_count: 0,
        }
    }

    /// Creates a new bars mode with spectrum color mode.
    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }
}

impl VisualizationMode for BarsMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        // Update rolling max
        let rolling_max = self.rolling_max_tracker.update(fft_data.get_magnitude_bins());
        let scaling_context = ScalingContext::new(rolling_max, config.screen_height as f32);

        // Update animator targets if needed
        if config.animator.is_new_cycle_start() {
            let mut new_targets = vec![0.0f32; config.num_elements];
            
            // Calculate average for low signal detection
            let avg: f32 = fft_data.get_magnitude_bins().iter().sum::<f32>() 
                / fft_data.get_magnitude_bins().len() as f32;

            if avg < 0.002 {
                // Low signal - set minimum height
                for target in &mut new_targets {
                    *target = 1.0f32;
                }
            } else {
                // Calculate targets from FFT data
                let band_ranges = config.band_aggregator.band_ranges();
                let bin_strategy = config.bin_strategy;
                for (i, &(start, end)) in band_ranges.iter().enumerate().take(config.num_elements) {
                    let bin_val = fft_data.get_band_aggregate(
                        start as usize,
                        end as usize,
                        bin_strategy,
                    );
                    let scaled = self.scaling_strategy.scale(
                        bin_val,
                        config.animator.get_max_display_value() as f32,
                        &scaling_context,
                    );
                    new_targets[i] = scaled.clamp(1.0, config.animator.get_max_display_value() as f32);
                }
            }
            config.animator.set_new_targets(&new_targets);
        }

        // Update animator
        let current_heights = config.animator.update_and_get_current_values();

        // Clear screen
        target.fill_solid(&target.bounding_box(), Rgb888::BLACK)?;

        // Draw bars
        let bar_spacing = config.bar_spacing;
        for (i, &height) in current_heights.iter().enumerate() {
            let bar_start_x = (i as u32 * (config.bar_width as u32 + bar_spacing)) as i32;
            let y_bottom = (config.screen_height - 1) as i32;
            // Ensure height fits within screen bounds - height is u16, screen_height is u16
            let height_clamped = height.min(config.screen_height);
            let y_top = ((config.screen_height as i32) - (height_clamped as i32))
                .max(0)
                .min((config.screen_height as i32).saturating_sub(1));

            // Calculate color
            let color_ctx = ColorContext::new(
                i,
                height as f32,
                config.num_elements,
                config.screen_height as f32,
                rolling_max,
                self.frame_count,
            );
            let color = self.color_mode.calculate_color(&color_ctx);

            // Render bar
            renderer::render_bar(target, bar_start_x, y_top, y_bottom, config.bar_width, color)?;
        }

        Ok(())
    }

    fn update(&mut self, frame_count: u32) {
        self.frame_count = frame_count;
        self.color_mode.update(frame_count);
    }
}

/// Skinny bars mode - thinner bars with configurable spacing.
///
/// This mode renders bars with width of 1 pixel and configurable spacing
/// to spread them out across the screen.
pub struct SkinnyBarsMode {
    color_mode: Box<dyn ColorMode>,
    rolling_max_tracker: RollingMaxTracker,
    scaling_strategy: LinearRollingMaxScaling,
    frame_count: u32,
    /// Spacing between bars in pixels (default: calculated from screen width)
    bar_spacing: u32,
}

impl SkinnyBarsMode {
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            color_mode,
            rolling_max_tracker: RollingMaxTracker::new(1.0, 0.9),
            scaling_strategy: LinearRollingMaxScaling,
            frame_count: 0,
            bar_spacing: 1, // Will be calculated based on screen width
        }
    }

    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }

    /// Sets the spacing between bars.
    pub fn with_spacing(mut self, spacing: u32) -> Self {
        self.bar_spacing = spacing.max(1);
        self
    }

    /// Calculates optimal spacing to fill screen width.
    ///
    /// This ensures bars are evenly distributed across the screen.
    fn calculate_spacing(&mut self, screen_width: u16, num_elements: usize) {
        if num_elements > 0 && num_elements <= screen_width as usize {
            // Calculate spacing to evenly distribute bars across screen
            let total_bar_width = num_elements as u32; // 1 pixel per bar
            let available_space = screen_width as u32 - total_bar_width;
            if num_elements > 1 {
                self.bar_spacing = (available_space / (num_elements - 1) as u32).max(1);
            } else {
                self.bar_spacing = 1;
            }
        }
    }
}

impl VisualizationMode for SkinnyBarsMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        // Calculate spacing based on screen width to ensure bars fit
        self.calculate_spacing(config.screen_width, config.num_elements);

        let rolling_max = self.rolling_max_tracker.update(fft_data.get_magnitude_bins());
        let scaling_context = ScalingContext::new(rolling_max, config.screen_height as f32);

        if config.animator.is_new_cycle_start() {
            let mut new_targets = vec![0.0f32; config.num_elements];
            let avg: f32 = fft_data.get_magnitude_bins().iter().sum::<f32>() 
                / fft_data.get_magnitude_bins().len() as f32;

            if avg < 0.002 {
                for target in &mut new_targets {
                    *target = 1.0f32;
                }
            } else {
                let band_ranges = config.band_aggregator.band_ranges();
                let bin_strategy = config.bin_strategy;
                for (i, &(start, end)) in band_ranges.iter().enumerate().take(config.num_elements) {
                    let bin_val = fft_data.get_band_aggregate(
                        start as usize,
                        end as usize,
                        bin_strategy,
                    );
                    let scaled = self.scaling_strategy.scale(
                        bin_val,
                        config.animator.get_max_display_value() as f32,
                        &scaling_context,
                    );
                    new_targets[i] = scaled.clamp(1.0, config.animator.get_max_display_value() as f32);
                }
            }
            config.animator.set_new_targets(&new_targets);
        }

        let current_heights = config.animator.update_and_get_current_values();
        target.fill_solid(&target.bounding_box(), Rgb888::BLACK)?;

        let skinny_bar_width = 1u16; // Always 1 pixel for skinny bars
        
        for (i, &height) in current_heights.iter().enumerate() {
            // Calculate x position with spacing, ensuring it fits within screen bounds
            let x_spacing = skinny_bar_width as u32 + self.bar_spacing;
            let bar_start_x = ((i as u32 * x_spacing) as i32)
                .max(0)
                .min((config.screen_width as i32).saturating_sub(1));

            // Calculate y positions, ensuring they fit within screen bounds
            let y_bottom = (config.screen_height - 1) as i32;
            // Ensure height fits within screen bounds - height is u16, screen_height is u16
            let height_clamped = height.min(config.screen_height);
            let y_top = ((config.screen_height as i32) - (height_clamped as i32))
                .max(0)
                .min((config.screen_height as i32).saturating_sub(1));

            // Skip if bar would be outside screen bounds
            if bar_start_x >= config.screen_width as i32 {
                continue;
            }

            let color_ctx = ColorContext::new(
                i,
                height as f32,
                config.num_elements,
                config.screen_height as f32,
                rolling_max,
                self.frame_count,
            );
            let color = self.color_mode.calculate_color(&color_ctx);

            renderer::render_bar(target, bar_start_x, y_top, y_bottom, skinny_bar_width, color)?;
        }

        Ok(())
    }

    fn update(&mut self, frame_count: u32) {
        self.frame_count = frame_count;
        self.color_mode.update(frame_count);
    }
}

