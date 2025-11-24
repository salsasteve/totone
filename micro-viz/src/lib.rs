//! # micro-viz
//!
//! A `no_std` compatible visualization library for embedded graphics displays.
//!
//! This library provides hardware-independent visualization components for real-time
//! audio spectrum visualization. It implements animated bar graphs, color modes,
//! and spectrum aggregation strategies using the `embedded-graphics` trait system.
//!
//! ## Components
//!
//! - **BarGraph**: Main visualization component for drawing frequency bars
//! - **SpectralBandAggregator**: Maps FFT bins to display bars with configurable spread modes
//! - **SpectrumValueAnimator**: Smoothly interpolates bar heights between FFT updates
//! - **BinSummaryStrategy**: Strategies for aggregating FFT bins within frequency bands
//!
//! ## Example
//!
//! ```no_run
//! use micro_viz::BarGraph;
//! use embedded_graphics::prelude::*;
//!
//! // Create a bar graph for a 256x64 display with 7px wide bars
//! let mut bar_graph = BarGraph::new(256, 64, 7);
//!
//! // Update with FFT magnitude data (512 bins)
//! let fft_bins: [f32; 512] = [/* ... */];
//! bar_graph.update(&mut framebuffer, &fft_bins)?;
//! ```

#![no_std]
extern crate alloc;
use alloc::vec;

use core::cmp; // Add this for cmp::max/min
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "logging")]
use defmt::info;
#[cfg(feature = "logging")]
use defmt_rtt as _;

// Define the constant if it's not already defined elsewhere
const BAR_SPACING: u32 = 1; // Space between bars
const BLACK: Rgb888 = Rgb888::BLACK; // Example color
#[allow(unused_imports)] 
use micromath::F32Ext; 



// Core modules
pub mod spectral_band_aggregator; 
pub use spectral_band_aggregator::*; 
pub mod bin_summary_strategy;
pub use bin_summary_strategy::*;
pub mod spectrum_value_animator;
pub use spectrum_value_animator::*;

// New modular components
pub mod fft_provider;
pub use fft_provider::*;
pub mod scaling;
pub use scaling::*;
pub mod color_mode;
pub use color_mode::*;
pub mod renderer;
pub use renderer::*;
pub mod visualization_mode;
pub use visualization_mode::*;
pub mod effects;
pub use effects::*;

// Visualization mode implementations
pub mod modes;
pub use modes::*;

// Main visualizer coordinator
pub mod visualizer;
pub use visualizer::*;

/// Color modes for spectrum visualization.
///
/// Different modes provide various visual effects for the frequency bars.
pub enum ColorMode {
    /// Color wheel distribution across bars (rainbow effect)
    Spectrum,
    /// Intensity-based grayscale gradient
    Gradient,
    /// Fixed color palette cycling
    StaticPalette,
    /// FFT magnitude-based dynamic colors
    DynamicFFT,
    /// Animated color wheel that shifts over time
    ShiftingSpectrum,
}


/// Real-time audio spectrum visualizer using animated bar graphs.
///
/// This struct handles the visualization of FFT magnitude data as animated
/// frequency bars on a display. It supports multiple color modes, smooth
/// animation between updates, and configurable bin aggregation strategies.
pub struct BarGraph {
    wheel_val: u8,
    step_counter: u32,
    screen_width: u16,
    screen_height: u16,
    bar_width: u16,
    color_wheel_multiplier: u8,
    num_bars: usize,
    rolling_max_fft: f32,
    rolling_max_decay: f32,
    color_mode: ColorMode,
    log_counter: u8,
    bss: BinSummaryStrategy,
    current_bin_average: f32,
    sba: SpectralBandAggregator,
    animator: SpectrumValueAnimator,
}

impl BarGraph {
    /// Creates a new `BarGraph` visualizer.
    ///
    /// # Arguments
    ///
    /// * `screen_width` - Display width in pixels
    /// * `screen_height` - Display height in pixels
    /// * `bar_width` - Width of each frequency bar in pixels
    ///
    /// # Returns
    ///
    /// A new `BarGraph` instance configured for the given display dimensions.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use micro_viz::BarGraph;
    ///
    /// // Create a bar graph for a 256x64 LED matrix with 7px wide bars
    /// let mut bar_graph = BarGraph::new(256, 64, 7);
    /// ```
    pub fn new(screen_width: u16, screen_height: u16, bar_width: u16) -> Self {
        #[cfg(feature = "std")]
        std::println!("BarGraph::new called with screen_width: {}, screen_height: {}, bar_width: {}", screen_width, screen_height, bar_width);
        #[cfg(feature = "logging")]
        info!("BarGraph::new called with screen_width: {}, screen_height: {}, bar_width: {}", screen_width, screen_height, bar_width);
        let num_bars = (screen_width / (bar_width + BAR_SPACING as u16)).max(1) as usize;
        
        let sba = SpectralBandAggregator::new(512, num_bars, BandSpreadModeConfig::Exponential{exp_factor: 7.0});

        let initial_bar_height_calc = 1.0f32;
        let animator_interpolation_steps = 5; 
        let animator_max_display_height = screen_height.saturating_sub(1).max(1);

        let animator = SpectrumValueAnimator::new(
            num_bars,
            initial_bar_height_calc,
            animator_interpolation_steps,
            animator_max_display_height,
        );


        Self {
            wheel_val: 0,
            step_counter: 0,
            screen_width,
            screen_height,
            bar_width,
            color_wheel_multiplier: 32,
            num_bars,
            rolling_max_fft: 1.0,
            rolling_max_decay: 0.9, // Decay factor for the rolling max
            color_mode: ColorMode::Spectrum,
            log_counter: 0,
            bss: BinSummaryStrategy::Max, // Default bin calculation mode
            current_bin_average: 0.0,
            sba,
            animator,
        }
    }

    fn color_wheel(&self, pos: u8) -> Rgb888 {
        let pos = pos % 255;
        if pos < 85 {
            Rgb888::new(
                pos.saturating_mul(3),
                255u8.saturating_sub(pos.saturating_mul(3)),
                0,
            )
        } else if pos < 170 {
            let pos = pos - 85;
            Rgb888::new(
                255u8.saturating_sub(pos.saturating_mul(3)),
                0,
                pos.saturating_mul(3),
            )
        } else {
            let pos = pos - 170;
            Rgb888::new(
                0,
                pos.saturating_mul(3),
                255u8.saturating_sub(pos.saturating_mul(3)),
            )
        }
    }

    fn draw_bars<D>(&self, fb: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        for i in 0..self.num_bars {
            // Get current height from the animator's display values
            let current_height_val = self.animator.get_current_values_display()[i];
            
            let bar_start_x = (i as u32 * (self.bar_width as u32 + BAR_SPACING)) as i32;
            let bar_end_x = bar_start_x + (self.bar_width as i32) - 1;

            let x_left = cmp::max(0, cmp::min((self.screen_width - 1) as i32, bar_start_x));
            let x_right = cmp::max(x_left, cmp::min((self.screen_width - 1) as i32, bar_end_x));

            if x_left > x_right { 
                continue;
            }
            
            let color = self.apply_color_mode(current_height_val as f32, i);
            
            // Draw bar from bottom of screen up to current_height_val
            // (0,0) is top-left, so bars grow upwards from bottom
            // Bottom coordinate: screen_height - 1
            // Top coordinate: screen_height - height (clamped to 0)
            let y_bottom = (self.screen_height - 1) as i32;
            let y_top = (self.screen_height.saturating_sub(current_height_val).max(0)) as i32;


            for bar_x_offset in 0..self.bar_width {
                let x_coord = bar_start_x + bar_x_offset as i32;
                if x_coord < 0 || x_coord >= self.screen_width as i32 { continue; }

                Line::new(Point::new(x_coord, y_top), Point::new(x_coord, y_bottom))
                    .into_styled(PrimitiveStyle::with_stroke(color, 1)) 
                    .draw(fb)?;
            }
        }
        Ok(())
    }

    fn apply_color_mode(&self, current_height_val: f32, bar_number: usize) -> Rgb888 {
        let color = match self.color_mode {
            ColorMode::Spectrum => {
                let color_position = ((bar_number as u32 * 255 / (self.num_bars as u32 + BAR_SPACING)) % 255) as u8;
                self.color_wheel(color_position)
            }
            ColorMode::Gradient => {
                let intensity =
                    (current_height_val as f32 / self.screen_height as f32 * 255.0) as u8;
                Rgb888::new(intensity, intensity, intensity)
            }
            ColorMode::StaticPalette => {
                let palette = [Rgb888::RED, Rgb888::GREEN, Rgb888::BLUE];
                palette[bar_number % palette.len()]
            }
            ColorMode::DynamicFFT => {
                let intensity = (current_height_val as f32 / self.rolling_max_fft * 255.0) as u8;
                Rgb888::new(intensity, 0, 255 - intensity)
            }
            ColorMode::ShiftingSpectrum => self.color_wheel(
                (bar_number as u8)
                    .wrapping_mul(self.color_wheel_multiplier)
                    .wrapping_add(self.wheel_val),
            ),
        };
        color
    }

    fn update_bar_target_height(&mut self, bins: &[f32], bar_number: usize) -> f32 { // Return f32 for animator
        if bar_number >= self.num_bars {
            #[cfg(feature = "std")]
            std::println!("Warning: bar_number {} out of bounds for target_heights.", bar_number);
            return 1.0f32; // Return a default low value
        }

        let bin_val = self.get_bin_value(bins, bar_number);
        // apply_rolling_max_scaling now scales relative to screen_height for dynamic range
        // but the final clamp should be to what the animator can display.
        let scaled_bin_val = self.apply_rolling_max_scaling(bin_val);

        // Clamp to the animator's displayable range.
        // Ensure target is at least 1.0 if that's a desired minimum visual.
        scaled_bin_val.clamp(1.0, self.animator.get_max_display_value() as f32)
    }

    fn apply_rolling_max_scaling(&self, value: f32) -> f32 {
        // Scale the value based on the rolling max FFT relative to screen height for full dynamic range
        if self.rolling_max_fft <= 0.0 { return 0.0; } // Avoid division by zero or negative
        let scaled_value = (value / self.rolling_max_fft) * self.screen_height as f32;
        scaled_value.max(0.0) // Ensure non-negative
    }

    fn get_bin_value(&self, bins: &[f32], bar_number: usize) -> f32 {
        let (start_index, end_index) = self.sba.band_ranges()[bar_number];
        self.run_bin_calculation(bins, start_index as usize, end_index as usize)
    }

    fn run_bin_calculation(&self, bins: &[f32], start_index: usize, end_index: usize) -> f32 {
        self.bss.calculate(&bins[start_index..end_index])
    }

    fn update_max_rolling_fft(&mut self, bins: &[f32]) {
        let max_in_current_bins = bins.iter().cloned().fold(0.0, f32::max);
        if max_in_current_bins > self.rolling_max_fft {
            self.rolling_max_fft = max_in_current_bins.max(self.rolling_max_fft);
        } else {
            self.rolling_max_fft = self.rolling_max_fft * self.rolling_max_decay;
        };
    }

    fn get_all_bins_average(&self, bins: &[f32]) -> f32 {
        let sum: f32 = bins.iter().copied().sum();
        let avg = sum / bins.len() as f32;
        avg
    }

    /// Updates the visualization with new FFT magnitude data and renders to the framebuffer.
    ///
    /// This is the main update loop that should be called once per frame. It:
    /// 1. Updates the rolling max FFT value for dynamic scaling
    /// 2. Calculates new target bar heights from FFT bins (when animator cycle starts)
    /// 3. Updates the animator's interpolation state
    /// 4. Draws the bars to the framebuffer
    ///
    /// # Arguments
    ///
    /// * `fb` - Mutable reference to the framebuffer (must implement `DrawTarget<Color = Rgb888>`)
    /// * `bins` - Slice of FFT magnitude values (typically 512 bins)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Successfully updated and rendered
    /// * `Err(D::Error)` - Drawing error from the framebuffer
    ///
    /// # Example
    ///
    /// ```no_run
    /// use micro_viz::BarGraph;
    /// use embedded_graphics::prelude::*;
    ///
    /// let mut bar_graph = BarGraph::new(256, 64, 7);
    /// let fft_bins: [f32; 512] = [/* ... magnitude spectrum ... */];
    ///
    /// // Update visualization (call once per frame)
    /// bar_graph.update(&mut framebuffer, &fft_bins)?;
    /// # Ok::<(), ()>(())
    /// ```
    pub fn update<D>(&mut self, fb: &mut D, bins: &[f32]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        self.update_max_rolling_fft(bins);
        self.current_bin_average = self.get_all_bins_average(bins);

        // Optional: Log counter for other purposes
        self.log_counter = self.log_counter.wrapping_add(1);
        if self.log_counter >= 200 { // Increased logging interval
            #[cfg(feature = "logging")]
            info!(
                "Animator current_values_display: {:?}",
                defmt::Debug2Format(&self.animator.get_current_values_display())
            );
            self.log_counter = 0;
        }
        #[cfg(feature = "std")]
        {
             std::println!(
                "BEGIN BarGraph UPDATE ------ Step Counter: {}", self.step_counter
            );
             std::println!(
                "Animator State: Counter={}, Steps={}, IsNewCycle={}",
                self.animator.get_interpolation_counter(),
                self.animator.get_interpolation_steps(),
                self.animator.is_new_cycle_start()
            );
            std::println!(
                "Bins length: {}, Num_bars: {}",
                bins.len(),
                self.num_bars,
            );
            std::println!("Rolling Max FFT: {:.2}, Current Bin Avg: {:.4}", self.rolling_max_fft, self.current_bin_average);
        }


        // Step 1: If animator is ready for new targets, calculate and set them.
        if self.animator.is_new_cycle_start() {
            let mut new_targets_for_animator = vec![0.0f32; self.num_bars];

            if self.current_bin_average < 0.002 { // Threshold for low signal
                // If signal is very low, make targets decay to a minimum visible height (e.g., 1.0)
                for i in 0..self.num_bars {
                    // Option 1: Decay towards a floor value
                    // let current_anim_target = self.animator.get_target_values_calc().get(i).copied().unwrap_or(1.0);
                    // new_targets_for_animator[i] = (current_anim_target * self.rolling_max_decay).max(1.0);
                    // Option 2: Simpler, just set to a minimum floor if signal is low
                     new_targets_for_animator[i] = 1.0f32;
                }
                 #[cfg(feature = "std")]
                 std::println!("Low signal: Decaying targets.");
            } else {
                // Calculate new targets from FFT bins
                for i in 0..self.num_bars {
                    new_targets_for_animator[i] = self.update_bar_target_height(bins, i);
                }
            }
            self.animator.set_new_targets(&new_targets_for_animator);
            #[cfg(feature = "std")]
            {
                std::println!("ANIMATOR: NEW TARGETS SET:");
                std::println!("  Previous (from animator): {:?}", self.animator.get_previous_values_calc());
                std::println!("  New Targets (to animator): {:?}", self.animator.get_target_values_calc());
            }
        }

        // Step 2: Tell the animator to update its state for this frame.
        // This calculates eased progress, interpolates values, and advances its internal counter.
        // The returned value is a reference to animator's internal current_values_display.
        let _current_display_heights_ref = self.animator.update_and_get_current_values();

        #[cfg(feature = "std")]
        {
            std::println!(
                "ANIMATOR: Updated. Counter_now = {}, Display[0]={:.2}",
                self.animator.get_interpolation_counter(),
                self.animator.get_current_values_display()[0]
            );
        }

        // Step 3: Draw the bars using the animator's current display values.
        // draw_bars internally accesses self.animator.current_values_display
        fb.fill_solid(&fb.bounding_box(), BLACK)?; // Clear screen before drawing
        self.draw_bars(fb)?;

        // Step 4: Update BarGraph's own non-animation counters/state
        self.wheel_val = self.wheel_val.wrapping_add(1);
        self.step_counter = self.step_counter.wrapping_add(1); // General frame counter

        #[cfg(feature = "std")]
        {
            std::println!(
                "END BarGraph UPDATE ------ Next Animator Counter Potential: {}", self.animator.get_interpolation_counter()
            );
            std::println!("-----------------------------------------------------");
        }
        Ok(())
    }

}

