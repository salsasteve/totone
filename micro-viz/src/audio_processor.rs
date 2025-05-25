use crate::bin_summary_strategy::*;
use alloc::{vec, vec::Vec};

use crate::types::*;


pub struct AudioProcessor {
    num_bars: usize,
    rolling_max_per_bar: Vec<f32>,
    rolling_max_decay: f32,
    bss: BinSummaryStrategy,
}

impl AudioProcessor {
    pub fn new(num_bars: usize, rolling_max_decay: f32, bss: BinSummaryStrategy) -> Self {
        Self {
            num_bars,
            rolling_max_per_bar: vec![1.0; num_bars], // Start with 1.0 to avoid division by zero
            rolling_max_decay,
            bss,
        }
    }

    pub fn process(&mut self, bins: &[f32], band_ranges: &[(u16, u16)]) -> AudioFrame {
        self.update_rolling_max_per_bar(bins, band_ranges);

        let mut frame: AudioFrame = Vec::with_capacity(self.num_bars);

        for i in 0..self.num_bars {
            let bin_val = self.get_bin_value(bins, i, band_ranges);
            let peak_val = self.rolling_max_per_bar[i];

            let normalized = if peak_val > 0.0 {
                (bin_val / peak_val).clamp(0.0, 1.0)
            } else {
                0.0
            };

            frame.push(BandData {
                index: i,
                current_value: bin_val,
                normalized_value: normalized,
                peak_value: peak_val,
            });
        }
        frame
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
    
    fn get_bin_value(&self, bins: &[f32], bar_number: usize, band_ranges: &[(u16, u16)]) -> f32 {
        let (start_index, end_index) = band_ranges[bar_number];
        self.run_bin_calculation(bins, start_index as usize, end_index as usize)
    }

    fn run_bin_calculation(&self, bins: &[f32], start_index: usize, end_index: usize) -> f32 {
        self.bss.calculate(&bins[start_index..end_index])
    }
}