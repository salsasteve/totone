use alloc::vec;

use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb888};

#[cfg(feature = "logging")]
use defmt::info;
#[cfg(feature = "logging")]
use defmt_rtt as _;

const BAR_SPACING: u32 = 1;
#[allow(unused_imports)]
use micromath::F32Ext;

use crate::bin_summary_strategy::*;
use crate::renderer::*;
use crate::spectral_band_aggregator::*;
use crate::spectrum_value_animator::*;

pub struct BarGragh {
    wheel_val: u8,
    step_counter: u32,
    num_bars: usize,
    log_counter: u8,
    current_bin_average: f32,
    sba: SpectralBandAggregator,
    animator: SpectrumValueAnimator,
    renderer: BarGraphRenderer,
}

impl BarGragh {
    pub fn new(screen_width: u16, screen_height: u16, bar_width: u16) -> Self {
        #[cfg(feature = "std")]
        std::println!(
            "BarGragh::new called with screen_width: {}, screen_height: {}, bar_width: {}",
            screen_width,
            screen_height,
            bar_width
        );
        #[cfg(feature = "logging")]
        info!(
            "BarGragh::new called with screen_width: {}, screen_height: {}, bar_width: {}",
            screen_width, screen_height, bar_width
        );
        let num_bars = (screen_width / (bar_width + BAR_SPACING as u16)).max(1) as usize;

        let sba = SpectralBandAggregator::new(
            512,
            num_bars,
            BandSpreadModeConfig::Exponential { exp_factor: 7.0 },
        );

        let initial_bar_height_calc = 1.0f32;
        let animator_interpolation_steps = 10;
        let animator_max_display_height = screen_height.saturating_sub(1).max(1);

        let animator = SpectrumValueAnimator::new(
            num_bars,
            initial_bar_height_calc,
            animator_interpolation_steps,
            animator_max_display_height,
        );
        let renderer = BarGraphRenderer::new(
            screen_width,
            screen_height,
            bar_width,
            num_bars,
            ColorMode::ShiftingSpectrum,
            0.50,
            BinSummaryStrategy::Max,
        );

        Self {
            wheel_val: 0,
            step_counter: 0,
            num_bars,
            log_counter: 0,
            current_bin_average: 0.0,
            sba,
            animator,
            renderer,
        }
    }

    fn get_all_bins_average(&self, bins: &[f32]) -> f32 {
        let sum: f32 = bins.iter().copied().sum();
        let avg = sum / bins.len() as f32;
        avg
    }

    pub fn update<D>(&mut self, fb: &mut D, bins: &[f32]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        let band_ranges = self.sba.band_ranges();
        self.renderer.update_rolling_max_per_bar(bins, band_ranges);
        self.current_bin_average = self.get_all_bins_average(bins);

        // Optional: Log counter for other purposes
        self.log_counter = self.log_counter.wrapping_add(1);
        if self.log_counter >= 200 {
            // Increased logging interval
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
                "BEGIN BarGragh UPDATE ------ Step Counter: {}",
                self.step_counter
            );
            std::println!(
                "Animator State: Counter={}, Steps={}, IsNewCycle={}",
                self.animator.get_interpolation_counter(),
                self.animator.get_interpolation_steps(),
                self.animator.is_new_cycle_start()
            );
            std::println!("Bins length: {}, Num_bars: {}", bins.len(), self.num_bars,);
            std::println!(
                "Rolling Max FFT: {:.2}, Current Bin Avg: {:.4}",
                self.rolling_max_fft,
                self.current_bin_average
            );
        }

        // Step 1: If animator is ready for new targets, calculate and set them.
        if self.animator.is_new_cycle_start() {
            let mut new_targets_for_animator = vec![0.0f32; self.num_bars];

            if self.current_bin_average < 0.002 {
                // Threshold for low signal
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
                let max_display_value = self.animator.get_max_display_value() as f32;
                for i in 0..self.num_bars {
                    new_targets_for_animator[i] = self.renderer.update_bar_target_height(
                        bins,
                        i,
                        max_display_value,
                        band_ranges,
                    );
                }
            }
            self.animator.set_new_targets(&new_targets_for_animator);
            #[cfg(feature = "std")]
            {
                std::println!("ANIMATOR: NEW TARGETS SET:");
                std::println!(
                    "  Previous (from animator): {:?}",
                    self.animator.get_previous_values_calc()
                );
                std::println!(
                    "  New Targets (to animator): {:?}",
                    self.animator.get_target_values_calc()
                );
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
        // fb.fill_solid(&fb.bounding_box(), BLACK)?; // Clear screen before drawing
        let current_height_val = self.animator.get_current_values_display();
        self.renderer.draw(fb, current_height_val)?;

        // Step 4: Update BarGragh's own non-animation counters/state
        self.wheel_val = self.wheel_val.wrapping_add(1);
        self.step_counter = self.step_counter.wrapping_add(1); // General frame counter

        #[cfg(feature = "std")]
        {
            std::println!(
                "END BarGragh UPDATE ------ Next Animator Counter Potential: {}",
                self.animator.get_interpolation_counter()
            );
            std::println!("-----------------------------------------------------");
        }
        Ok(())
    }
}
