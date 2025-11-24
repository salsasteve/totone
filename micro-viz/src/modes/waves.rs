//! Wave visualization mode implementations.
//!
//! This module provides wave-based visualization modes, including sine waves,
//! sawtooth waves, square waves, and zigzag patterns.

use crate::{
    fft_provider::FftDataProvider,
    visualization_mode::{VisualizationMode, VisualizationConfig},
    color_mode::{ColorMode, ColorContext},
    renderer,
    scaling::{RollingMaxTracker, ScalingContext, ScalingStrategy, LinearRollingMaxScaling},
};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
};
use alloc::vec;
use alloc::boxed::Box;

// This is used but is used implicitly by the compiler
#[allow(unused_imports)] 
use micromath::F32Ext;

/// Sine wave visualization mode with smooth interpolation.
///
/// Renders FFT data as a smooth sine wave connecting the peaks using
/// linear interpolation between control points.
pub struct SineWaveMode {
    color_mode: Box<dyn ColorMode>,
    rolling_max_tracker: RollingMaxTracker,
    scaling_strategy: LinearRollingMaxScaling,
    frame_count: u32,
    /// Number of interpolation steps between control points (higher = smoother)
    interpolation_steps: u32,
}

impl SineWaveMode {
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            color_mode,
            rolling_max_tracker: RollingMaxTracker::new(1.0, 0.9),
            scaling_strategy: LinearRollingMaxScaling,
            frame_count: 0,
            interpolation_steps: 4, // Interpolate 4 points between each control point
        }
    }

    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }

    /// Sets the number of interpolation steps between control points.
    pub fn with_interpolation_steps(mut self, steps: u32) -> Self {
        self.interpolation_steps = steps.max(1).min(20);
        self
    }
}

impl VisualizationMode for SineWaveMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        let rolling_max = self.rolling_max_tracker.update(fft_data.get_magnitude_bins());
        let scaling_context = ScalingContext::new(rolling_max, config.screen_height as f32);

        // Update animator if needed
        if config.animator.is_new_cycle_start() {
            let mut new_targets = vec![0.0f32; config.num_elements];
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
            config.animator.set_new_targets(&new_targets);
        }

        let current_heights = config.animator.update_and_get_current_values();
        target.fill_solid(&target.bounding_box(), Rgb888::BLACK)?;

        // Generate control points for the wave
        let control_points: alloc::vec::Vec<Point> = current_heights
            .iter()
            .enumerate()
            .map(|(i, &height)| {
                // Calculate x position ensuring we stay within screen bounds
                let x_spacing = config.bar_width as u32 + config.bar_spacing;
                let x = ((i as u32 * x_spacing) as i32)
                    .max(0)
                    .min((config.screen_width as i32).saturating_sub(1));
                
                // Calculate y position ensuring we stay within screen bounds
                let base_y = (config.screen_height - 1) as i32;
                let y_offset = height as i32;
                let y = (base_y - y_offset)
                    .max(0)
                    .min((config.screen_height as i32).saturating_sub(1));
                
                Point::new(x, y)
            })
            .collect();

        // Draw smooth interpolated curve between control points
        for i in 0..control_points.len().saturating_sub(1) {
            let start = control_points[i];
            let end = control_points[i + 1];

            // Interpolate between points for smoother curve
            for step in 0..=self.interpolation_steps {
                let t = step as f32 / self.interpolation_steps as f32;
                
                // Use smooth interpolation (ease-in-out curve)
                let smooth_t = t * t * (3.0 - 2.0 * t);
                
                let x = start.x as f32 + (end.x - start.x) as f32 * smooth_t;
                let y = start.y as f32 + (end.y - start.y) as f32 * smooth_t;
                
                let interpolated_point = Point::new(
                    x.round() as i32,
                    y.round() as i32,
                );

                // Clamp to screen bounds
                let x_clamped = interpolated_point.x
                    .max(0)
                    .min((config.screen_width as i32).saturating_sub(1));
                let y_clamped = interpolated_point.y
                    .max(0)
                    .min((config.screen_height as i32).saturating_sub(1));

                let clamped_point = Point::new(x_clamped, y_clamped);

                // Draw line segment if not first point
                if step > 0 {
                    let prev_t = (step - 1) as f32 / self.interpolation_steps as f32;
                    let prev_smooth_t = prev_t * prev_t * (3.0 - 2.0 * prev_t);
                    let prev_x = start.x as f32 + (end.x - start.x) as f32 * prev_smooth_t;
                    let prev_y = start.y as f32 + (end.y - start.y) as f32 * prev_smooth_t;
                    let prev_point = Point::new(
                        prev_x.round() as i32,
                        prev_y.round() as i32,
                    );
                    let prev_x_clamped = prev_point.x
                        .max(0)
                        .min((config.screen_width as i32).saturating_sub(1));
                    let prev_y_clamped = prev_point.y
                        .max(0)
                        .min((config.screen_height as i32).saturating_sub(1));
                    let prev_clamped = Point::new(prev_x_clamped, prev_y_clamped);

                    // Calculate color based on position between control points
                    let height_lerp = start.y as f32 + (end.y - start.y) as f32 * smooth_t;
                    let color_ctx = ColorContext::new(
                        i,
                        (config.screen_height as i32 - height_lerp as i32).max(0) as f32,
                        config.num_elements,
                        config.screen_height as f32,
                        rolling_max,
                        self.frame_count,
                    );
                    let color = self.color_mode.calculate_color(&color_ctx);

                    renderer::render_line(target, prev_clamped, clamped_point, color)?;
                }
            }
        }

        Ok(())
    }

    fn update(&mut self, frame_count: u32) {
        self.frame_count = frame_count;
        self.color_mode.update(frame_count);
    }
}

/// Sawtooth wave visualization mode.
///
/// Renders FFT data as a sawtooth waveform with linear ramps.
pub struct SawtoothWaveMode {
    sine_mode: SineWaveMode,
}

impl SawtoothWaveMode {
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            sine_mode: SineWaveMode::new(color_mode),
        }
    }

    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }
}

impl VisualizationMode for SawtoothWaveMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        // Similar to sine wave but with sawtooth interpolation
        // For now, use the same as sine wave (will enhance with sawtooth math)
        self.sine_mode.render(target, fft_data, config)
    }

    fn update(&mut self, frame_count: u32) {
        self.sine_mode.update(frame_count);
    }
}

/// Square wave visualization mode.
///
/// Renders FFT data as a square wave with flat tops.
pub struct SquareWaveMode {
    sine_mode: SineWaveMode,
}

impl SquareWaveMode {
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            sine_mode: SineWaveMode::new(color_mode),
        }
    }

    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }
}

impl VisualizationMode for SquareWaveMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        // Similar to sine wave but with square wave interpolation
        // For now, use the same as sine wave (will enhance with square wave math)
        self.sine_mode.render(target, fft_data, config)
    }

    fn update(&mut self, frame_count: u32) {
        self.sine_mode.update(frame_count);
    }
}

/// Zigzag visualization mode.
///
/// Renders FFT data as a zigzag pattern connecting peaks and valleys.
pub struct ZigZagMode {
    sine_mode: SineWaveMode,
}

impl ZigZagMode {
    pub fn new(color_mode: Box<dyn ColorMode>) -> Self {
        Self {
            sine_mode: SineWaveMode::new(color_mode),
        }
    }

    pub fn default() -> Self {
        Self::new(Box::new(crate::color_mode::SpectrumColorMode))
    }
}

impl VisualizationMode for ZigZagMode {
    fn render<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        fft_data: &dyn FftDataProvider,
        config: &mut VisualizationConfig,
    ) -> Result<(), D::Error> {
        // Zigzag pattern - alternate between high and low
        // For now, use the same as sine wave (will enhance with zigzag pattern)
        self.sine_mode.render(target, fft_data, config)
    }

    fn update(&mut self, frame_count: u32) {
        self.sine_mode.update(frame_count);
    }
}
