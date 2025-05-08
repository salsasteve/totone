#![no_std]
extern crate alloc;
use alloc::{vec, vec::Vec};

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

// Define the constant if it's not already defined elsewhere
const BAR_SPACING: u32 = 1;
const BLACK: Rgb888 = Rgb888::BLACK; // Example color

use micromath::F32Ext; // For f32.max/min in no_std

pub enum ColorMode {
    Spectrum,
    Gradient,
    StaticPalette,
    DynamicFFT,
}
pub enum BinSpreadMode {
    Even,
    Exponential,
}

pub struct BarGragh {
    wheel_val: u8,
    step_counter: u32,
    interpolation_counter: u32,
    stroke_width: u8,
    screen_width: u16,
    screen_height: u16,
    color_wheel_multiplier: u8,
    previous_heights: Vec<f32>,
    target_heights: Vec<f32>,
    current_heights: Vec<u16>,
    interpolation_steps: u32,
    num_bars: usize,
    group_size_to_average: usize,
    rolling_max_fft: f32,
    rolling_max_decay: f32,
    color_mode: ColorMode,
    bin_spread_mode: BinSpreadMode,
}

impl BarGragh {
    pub fn new(screen_width: u16, screen_height: u16) -> Self {
        let num_bars = (screen_width / 8).max(1) as usize; // Ensure num_bars is at least 1
        let group_size_to_average = if num_bars > 0 {
            (512 / num_bars).max(1) // Ensure group_size is at least 1
        } else {
            1 // Default if num_bars is 0 (though prevented above)
        };

        Self {
            wheel_val: 0,
            step_counter: 0,
            interpolation_counter: 0,
            stroke_width: 1,
            screen_width,
            screen_height,
            color_wheel_multiplier: 32,
            previous_heights: vec![63.0f32; num_bars],
            target_heights: vec![63.0f32; num_bars],
            current_heights: vec![63u16; num_bars],
            interpolation_steps: 20, // How fast to move the bars. Higher = slower
            num_bars,
            group_size_to_average,
            rolling_max_fft: 1.0,
            rolling_max_decay: 0.80, // Decay factor for the rolling max
            color_mode: ColorMode::Spectrum,
            bin_spread_mode: BinSpreadMode::Exponential, // Default bin spread mode
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

    fn ease_out_quad(&self, t: f32) -> f32 {
        let t_clamped = t.max(0.0).min(1.0); // Using F32Ext for max/min
        1.0 - (1.0 - t_clamped) * (1.0 - t_clamped)
    }

    /// Calculates the current interpolation progress using easing.
    /// Ensures progress reaches 1.0 at the end of the interpolation period.
    fn calculate_eased_progress(&self) -> f32 {
        let progress = if self.interpolation_steps == 0 || self.interpolation_steps == 1 {
            // If 0 steps, or only 1 step, jump directly to target.
            // For 1 step, interpolation_counter is 0, leading to progress 1.0.
            1.0
        } else {
            // For interpolation_steps > 1
            // self.interpolation_counter ranges from 0 to self.interpolation_steps - 1.
            // Division by (self.interpolation_steps - 1) makes progress span [0.0, 1.0].
            self.interpolation_counter as f32 / (self.interpolation_steps - 1) as f32
        };
        // ease_out_quad handles clamping its input, so raw progress can be passed.
        self.ease_out_quad(progress)
    }

    /// Interpolates the current heights based on previous, target, and progress.
    fn interpolate_current_heights(&mut self, eased_progress: f32) {
        if eased_progress >= 1.0 || self.interpolation_steps == 0 {
            // Snap to target heights if progress is 1.0 or no interpolation steps
            for i in 0..self.num_bars {
                if i < self.target_heights.len() && i < self.current_heights.len() {
                    self.current_heights[i] = self.target_heights[i].max(0.0) as u16;
                }
            }
            return;
        }

        // If eased_progress is 0.0 (start of a new cycle), this correctly sets
        // current_heights[i] = previous_heights[i].
        for i in 0..self.num_bars {
            if i < self.previous_heights.len()
                && i < self.target_heights.len()
                && i < self.current_heights.len()
            {
                let prev = self.previous_heights[i];
                let target = self.target_heights[i];
                let interpolated_val = prev * (1.0 - eased_progress) + target * eased_progress;
                self.current_heights[i] = interpolated_val.max(0.0) as u16;
            }
        }
    }

    fn draw_bars<D>(&self, fb: &mut D, bar_width: u32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        if bar_width == 0 {
            return Ok(());
        }

        for i in 0..self.num_bars {
            let current_height_val = self.current_heights.get(i).copied().unwrap_or(0);
            let bar_pixel_height = cmp::min(current_height_val, self.screen_height) as i32;
    
            // Invert the Y-axis: Bars are drawn from the top of the screen downwards
            let y_top = 0; // Top of the screen
            let y_bottom = cmp::min(self.screen_height as i32, bar_pixel_height);
    
            let bar_start_x = (i as u32 * (bar_width + BAR_SPACING)) as i32;
            let bar_end_x = bar_start_x + bar_width as i32 - 1;
    
            let x_left = cmp::max(0, cmp::min(self.screen_width as i32 - 1, bar_start_x));
            let x_right = cmp::max(x_left, cmp::min(self.screen_width as i32 - 1, bar_end_x));
    
            // let color = self
            //     .color_wheel((i as u8).wrapping_mul(self.color_wheel_multiplier).wrapping_add(self.wheel_val));
            let color = self.apply_color_mode(current_height_val as f32, i);
    
            // Draw the bar as a horizontal line at the inverted Y-coordinate
            Line::new(Point::new(x_left, y_bottom), Point::new(x_right, y_bottom))
                .into_styled(PrimitiveStyle::with_stroke(color, self.stroke_width as u32))
                .draw(fb)?;
        }
        Ok(())
    }

    fn apply_color_mode(&self, current_height_val:f32, bar_number: usize) -> Rgb888 {
        let color = match self.color_mode {
            ColorMode::Spectrum => {
                let color_position = ((bar_number as u16 * 255 / self.num_bars as u16) % 255) as u8;
                self.color_wheel(color_position)
            }
            ColorMode::Gradient => {
                let intensity = (current_height_val as f32 / self.screen_height as f32 * 255.0) as u8;
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
        };
        color
    }

    fn update_animation_state(&mut self) {
        self.wheel_val = self.wheel_val.wrapping_add(1);
        self.step_counter = self.step_counter.wrapping_add(1);

        if self.interpolation_steps > 0 {
            // self.interpolation_counter ranges from 0 to self.interpolation_steps - 1
            if self.interpolation_counter < self.interpolation_steps -1 { // Stop before exceeding max index for (S-1) division
                self.interpolation_counter += 1;
            } else {
                // Reached the end of the interpolation period (e.g., counter was S-1)
                // or if steps = 1, counter was 0.
                self.interpolation_counter = 0; // Reset for the next cycle
            }
        } else { // interpolation_steps == 0
            self.interpolation_counter = 0; // Keep at 0 for jump-to-target behavior
        }
    }
    
    fn update_bar_target_height(&mut self, bins: &[f32], bar_number: usize) {
        if bar_number >= self.num_bars || bar_number >= self.target_heights.len() {
            #[cfg(feature = "std")]
            std::println!("Warning: bar_number {} out of bounds.", bar_number);
            return;
        }

        let avg_val = self.get_bin_average_value(&bins, bar_number);
        let scaled_avg_val = self.apply_rolling_max_scaling(avg_val);
        self.target_heights[bar_number] = self.flip_height(scaled_avg_val);
    }
    
    fn apply_rolling_max_scaling(&self, value: f32) -> f32 {
        // Scale the value based on the rolling max FFT
        let scaled_value = value / self.rolling_max_fft * self.screen_height as f32;
        scaled_value.max(0.0) // Ensure non-negative
    }
    fn flip_height(&self, height: f32) -> f32 {
        ((self.screen_height - 1) as f32 - height).max(0.0)
    }

    fn get_bin_average_value(&self, bins: &[f32], bar_number: usize) -> f32 {
        let avg_val = match self.bin_spread_mode {
            BinSpreadMode::Even => {
                // Evenly spread bins
                let start_index = bar_number * self.group_size_to_average;
                let end_index = cmp::min(start_index + self.group_size_to_average, bins.len());
                if start_index < end_index {
                    let sum: f32 = bins[start_index..end_index].iter().sum();
                    sum / (end_index - start_index) as f32
                } else {
                    0.0
                }
            }
            BinSpreadMode::Exponential => {
                // Exponentially spread bins
                let exp_factor = 2.0; // Adjust this factor to control the exponential spread
                let max_bar = self.num_bars as f32;
                let max_bin = bins.len() as f32;
            
                // Normalize the exponential indices to fit within the range of bins
                let start_index = ((exp_factor.powf(bar_number as f32 / max_bar) - 1.0) / (exp_factor - 1.0) * max_bin) as usize;
                let end_index = ((exp_factor.powf((bar_number + 1) as f32 / max_bar) - 1.0) / (exp_factor - 1.0) * max_bin) as usize;
            
                // Clamp indices to valid range
                let start_index = start_index.min(bins.len());
                let end_index = end_index.min(bins.len());
            
                if start_index < end_index {
                    let sum: f32 = bins[start_index..end_index].iter().sum();
                    sum / (end_index - start_index) as f32
                } else {
                    0.0
                }
            }
        };
        avg_val
    }
    
     
    fn update_max_rolling_fft(&mut self, bins: &[f32]) {
        
        let max_in_current_bins = bins.iter().cloned().fold(0.0, f32::max);
        if max_in_current_bins > self.rolling_max_fft {
            self.rolling_max_fft = max_in_current_bins.max(self.rolling_max_fft);
        } else {
            self.rolling_max_fft = self.rolling_max_fft * self.rolling_max_decay
        };
        
    }

    pub fn update<D>(&mut self, fb: &mut D, bins: &[f32]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        self.update_max_rolling_fft(bins);
        #[cfg(feature = "std")]
        {
            std::println!("BEGIN UPDATE ------ Interpolation Counter: {}", self.interpolation_counter);
            std::println!("Bins length: {}, Num_bars: {}, Group size: {}", bins.len(), self.num_bars, self.group_size_to_average);
            std::println!("Initial previous_heights: {:?}", self.previous_heights);
            std::println!("Initial target_heights: {:?}", self.target_heights);
            std::println!("Initial current_heights: {:?}", self.current_heights);
        }

        // Step 1: Update target heights if it's the start of an interpolation cycle.
        if self.interpolation_counter == 0 {
            // Current target_heights become the previous_heights for the new interpolation.
            core::mem::swap(&mut self.previous_heights, &mut self.target_heights);
            // Now self.previous_heights holds values from the end of the last cycle.
            // self.target_heights (which was old previous) will be filled with new targets.

            // Calculate new target_heights from the input `bins`.
            for i in 0..self.num_bars {
                self.update_bar_target_height(bins, i);
            }
            #[cfg(feature = "std")]
            {
                std::println!("NEW TARGETS LOADED (counter is 0):");
                std::println!("  Previous (old targets): {:?}", self.previous_heights);
                std::println!("  Target (new from bins): {:?}", self.target_heights);
            }
        }

        // Step 2: Calculate interpolation progress for the current frame.
        let eased_progress = self.calculate_eased_progress();

        // Step 3: Interpolate current heights towards the target heights.
        // If interpolation_counter was 0 (and steps > 1), eased_progress is 0.
        // This correctly sets current_heights[i] = previous_heights[i].
        // If steps are 0 or 1, or at end of cycle, eased_progress is 1, setting current = target.
        self.interpolate_current_heights(eased_progress);
        
        #[cfg(feature = "std")]
        {
            std::println!("Interpolating: Progress_raw_calc = {:.2} (counter {} / steps {}) -> Eased_Progress = {:.2}",
                if self.interpolation_steps <= 1 { 1.0 } else { self.interpolation_counter as f32 / (self.interpolation_steps -1).max(1) as f32},
                self.interpolation_counter, self.interpolation_steps, eased_progress
            );
            std::println!("  Current Heights after interpolation: {:?}", self.current_heights);
        }


        // Step 4: Draw the bars onto the framebuffer.
        self.draw_bars(fb, 7)?; // Assuming bar_width is 7, make this configurable if needed

        // Step 5: Update animation state for the next frame.
        self.update_animation_state();
        
        #[cfg(feature = "std")]
        {
            std::println!("END UPDATE ------ Next Interpolation Counter: {}", self.interpolation_counter);
            std::println!("-----------------------------------------------------");
        }

        Ok(())
    }
}