use alloc::{vec, vec::Vec};

use core::cmp;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};

#[cfg(feature = "logging")]
use defmt::info;
#[cfg(feature = "logging")]
use defmt_rtt as _;

// Define the constant if it's not already defined elsewhere
const BAR_SPACING: u32 = 1; // Space between bars
const BLACK: Rgb888 = Rgb888::BLACK; // Example color
#[allow(unused_imports)]
use micromath::F32Ext;

use crate::bin_summary_strategy::*;
pub enum ColorMode {
    Spectrum,
    Gradient,
    StaticPalette,
    DynamicFFT,
    ShiftingSpectrum,
}
pub trait Renderer {
    fn draw<D: DrawTarget<Color = Rgb888>>(
        &self,
        target: &mut D,
        current_values: &[u16],
    ) -> Result<(), D::Error>;
}

pub struct BarGraphRenderer {
    wheel_val: u8,
    step_counter: u32,
    screen_width: u16,
    screen_height: u16,
    bar_width: u16,
    color_wheel_multiplier: u8,
    num_bars: usize,
    rolling_max_per_bar: Vec<f32>,
    rolling_max_decay: f32,
    color_mode: ColorMode,
    log_counter: u8,
    bss: BinSummaryStrategy,
    current_bin_average: f32,
}

impl Renderer for BarGraphRenderer {
    fn draw<D>(&self, fb: &mut D, current_values: &[u16]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        for i in 0..self.num_bars {
            // Get current height from the animator's display values
            let current_height_val = current_values[i];

            // The y_line_coord should be screen_height - height, assuming 0,0 is top-left
            // And bars grow upwards from the bottom.
            // If y_line_coord was meant to be the height itself (from top):
            let y_pos = self
                .screen_height
                .saturating_sub(current_height_val)
                .saturating_sub(1);
            let y_line_coord = cmp::max(0, y_pos) as i32;

            let bar_start_x = (i as u32 * (self.bar_width as u32 + BAR_SPACING)) as i32;
            let bar_end_x = bar_start_x + (self.bar_width as i32) - 1;

            let x_left = cmp::max(0, cmp::min((self.screen_width - 1) as i32, bar_start_x));
            let x_right = cmp::max(x_left, cmp::min((self.screen_width - 1) as i32, bar_end_x));

            if x_left > x_right {
                continue;
            }

            let color = self.apply_color_mode(current_height_val as f32, i);

            // Draw line from bottom of screen up to current_height_val
            // Assuming (x, screen_height-1) is bottom, (x, screen_height - 1 - height) is top of bar
            let y_bottom = (self.screen_height - 1) as i32;
            let y_top = (self.screen_height.saturating_sub(current_height_val).max(0)) as i32;

            for bar_x_offset in 0..self.bar_width {
                let x_coord = bar_start_x + bar_x_offset as i32;
                if x_coord < 0 || x_coord >= self.screen_width as i32 {
                    continue;
                }

                Line::new(Point::new(x_coord, y_top), Point::new(x_coord, y_bottom))
                    .into_styled(PrimitiveStyle::with_stroke(color, 1))
                    .draw(fb)?;
            }
        }
        Ok(())
    }
}

impl BarGraphRenderer {
    pub fn new(
        screen_width: u16,
        screen_height: u16,
        bar_width: u16,
        num_bars: usize,
        color_mode: ColorMode,
        rolling_max_decay: f32,
        bss: BinSummaryStrategy,
    ) -> Self {
        let bar_width = cmp::max(1, bar_width);
        let color_wheel_multiplier = 255 / num_bars as u8;
        let rolling_max_per_bar = vec![0.0; num_bars];
        Self {
            wheel_val: 0,
            step_counter: 0,
            screen_width,
            screen_height,
            bar_width,
            color_wheel_multiplier,
            num_bars,
            rolling_max_per_bar,
            rolling_max_decay,
            color_mode,
            log_counter: 0,
            bss,
            current_bin_average: 0.0,
        }
    }

    fn apply_color_mode(&self, current_height_val: f32, bar_number: usize) -> Rgb888 {
        let color = match self.color_mode {
            ColorMode::Spectrum => {
                let color_position =
                    ((bar_number as u32 * 255 / (self.num_bars as u32 + BAR_SPACING)) % 255) as u8;
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
                let max_display_value = 10.0;
                if max_display_value <= 0.0 {
                    return Rgb888::BLACK;
                }
                let intensity = (current_height_val as f32 / max_display_value * 255.0) as u8;
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

    pub fn update_rolling_max_per_bar(&mut self, bins: &[f32], band_ranges: &[(u16, u16)]) {
        for i in 0..self.num_bars {
            // Get the summarized value for the current bar
            let bin_val = self.get_bin_value(bins, i, band_ranges);

            // Update the personal rolling max for this specific bar
            if bin_val > self.rolling_max_per_bar[i] {
                self.rolling_max_per_bar[i] = bin_val;
            } else {
                self.rolling_max_per_bar[i] =
                    (self.rolling_max_per_bar[i] * self.rolling_max_decay).max(1.0);
            }
        }
    }

    fn get_bin_value(&self, bins: &[f32], bar_number: usize, band_ranges: &[(u16, u16)]) -> f32 {
        let (start_index, end_index) = band_ranges[bar_number];
        self.run_bin_calculation(bins, start_index as usize, end_index as usize)
    }

    fn run_bin_calculation(&self, bins: &[f32], start_index: usize, end_index: usize) -> f32 {
        self.bss.calculate(&bins[start_index..end_index])
    }

    pub fn update_bar_target_height(
        &mut self,
        bins: &[f32],
        bar_number: usize,
        max_display_value: f32,
        band_ranges: &[(u16, u16)],
    ) -> f32 {
        // Return f32 for animator
        if bar_number >= self.num_bars {
            #[cfg(feature = "std")]
            std::println!(
                "Warning: bar_number {} out of bounds for target_heights.",
                bar_number
            );
            return 1.0f32; // Return a default low value
        }

        let bin_val = self.get_bin_value(bins, bar_number, band_ranges);
        // apply_rolling_max_scaling now scales relative to screen_height for dynamic range
        // but the final clamp should be to what the animator can display.
        let max_for_this_bar = self.rolling_max_per_bar[bar_number];
        if max_for_this_bar <= 0.0 {
            return 1.0f32;
        }
        let scaled_val = (bin_val / max_for_this_bar) * max_display_value;

        // Clamp to the animator's displayable range.
        // Ensure target is at least 1.0 if that's a desired minimum visual.
        scaled_val.clamp(1.0, max_display_value)
    }
}
