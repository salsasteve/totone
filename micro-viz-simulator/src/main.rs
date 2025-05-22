use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};
use micro_viz::BarGragh;
use std::{thread, time::Duration};

// Constants for visualization parameters
pub const WIDTH: u32 = 128;
pub const HEIGHT: u32 = 64;
pub const FRAME_DELAY_MS: u64 = 1;

const BLACK: Rgb888 = Rgb888::new(0, 0, 0);

use std::f32::consts::PI;

const NUM_BINS: usize = 512;
const F_MIN: f32 = 20.0;
const F_MAX: f32 = 20000.0;

// Pre-calculate log values for min/max frequencies for efficiency

lazy_static::lazy_static! {
    static ref LOG_F_MIN: f32 = F_MIN.ln();
    static ref LOG_F_MAX: f32 = F_MAX.ln();
    static ref LOG_F_RANGE: f32 = *LOG_F_MAX - *LOG_F_MIN;
}

// // Parameter controlling the "width" of peaks in log-frequency space.
// // Smaller values = sharper peaks. A value of 0.2 means the influence
// // drops to ~60% when the frequency is ~1.22x or 1/1.22x the peak frequency.
const PEAK_LOG_STD_DEV: f32 = 0.15; // Standard deviation for Gaussian spread in log-freq space
const INV_2_SIGMA_SQ: f32 = 1.0 / (2.0 * PEAK_LOG_STD_DEV * PEAK_LOG_STD_DEV);

pub fn simulate_fft_simple_output_scaling(
    time: f32,
    target_min_output: f32,
    target_max_output: f32,
) -> [f32; NUM_BINS] {
    // NUM_BINS is assumed to be a const
    if target_min_output >= target_max_output {
        panic!("target_min_output must be less than target_max_output");
    }
    // Note: NUM_BINS is a const, so if it's 0, it's a compile-time issue for array,
    // or runtime panic with division by zero if (NUM_BINS - 1) is used.
    // Assuming NUM_BINS > 0. If NUM_BINS = 1, (NUM_BINS - 1) would be 0.
    if NUM_BINS == 0 {
        panic!("NUM_BINS constant must be greater than 0.");
    }

    // Define our 8 core source frequencies on a logarithmic scale
    let source_core_frequencies = [
        20.0,    // Low bass
        63.0,    // Bass
        200.0,   // Low-mid
        630.0,   // Mid
        2000.0,  // Upper-mid
        6300.0,  // Presence
        12000.0, // Brilliance
        20000.0, // High end (adjusted to be at F_MAX if F_MAX is one of the constants)
    ];

    // Base amplitude for each source frequency
    let source_base_amplitudes = [0.9, 0.85, 0.75, 0.6, 0.5, 0.4, 0.3, 0.25];

    // Array to store magnitudes for each frequency bin
    let mut magnitudes = [0.0f32; NUM_BINS];

    // --- 1. Accumulate influence from source frequencies ---
    for source_idx in 0..source_core_frequencies.len() {
        let core_freq = source_core_frequencies[source_idx];
        let base_amp = source_base_amplitudes[source_idx];

        let phase = time * core_freq * 0.015 + (source_idx as f32 * PI / 4.0); // Using PI
        let activity = (phase.sin() * 0.5 + 0.5) * base_amp;

        let log_core_freq = core_freq.ln();

        for i in 0..NUM_BINS {
            // Normalized position (0 to 1), handles NUM_BINS = 1 case
            let bin_pos_norm = if NUM_BINS > 1 {
                i as f32 / (NUM_BINS - 1) as f32
            } else {
                0.0
            };
            // Using assumed global/static constants LOG_F_MIN, LOG_F_RANGE, INV_2_SIGMA_SQ
            // The `*` dereference is kept from your original code, implying these might be Lazy or static refs.
            let bin_center_log_freq = *LOG_F_MIN + bin_pos_norm * *LOG_F_RANGE;
            let log_dist_sq = (bin_center_log_freq - log_core_freq).powi(2);
            let influence = (-log_dist_sq * INV_2_SIGMA_SQ).exp();
            magnitudes[i] += activity * influence;
        }
    }

    // --- 2. Apply overall spectral shaping and find max before noise ---
    let mut current_max_val_before_noise = 0.0f32;
    for i in 0..NUM_BINS {
        let bin_pos_norm = if NUM_BINS > 1 {
            i as f32 / (NUM_BINS - 1) as f32
        } else {
            0.0
        };
        let shaping_factor = 0.4 + 0.6 * (1.0 - bin_pos_norm).powi(2);
        magnitudes[i] *= shaping_factor;

        if magnitudes[i] > current_max_val_before_noise {
            current_max_val_before_noise = magnitudes[i];
        }
    }

    // --- 3. Normalize based on peak after shaping (from original) ---
    // This makes noise relative to the shaped signal's peak.
    if current_max_val_before_noise > 1e-6 {
        // Use a small epsilon to prevent division by zero/small numbers
        for i in 0..NUM_BINS {
            magnitudes[i] /= current_max_val_before_noise;
        }
    }
    // Magnitudes are now roughly in [0, 1]

    // --- 4. Add dynamic noise (from original) ---
    for i in 0..NUM_BINS {
        let bin_pos_norm = if NUM_BINS > 1 {
            i as f32 / (NUM_BINS - 1) as f32
        } else {
            0.0
        };
        let bin_center_log_freq = *LOG_F_MIN + bin_pos_norm * *LOG_F_RANGE;
        let bin_freq = bin_center_log_freq.exp();

        let noise_phase = time * bin_freq * 0.0025
            + (i as f32 * 0.27)
            + (i as f32 * PI * 5.5 / NUM_BINS as f32).sin() * 0.6;
        let noise_amplitude_factor = 0.05 + 0.03 * (1.0 - bin_pos_norm);
        let noise_val = (noise_phase.sin() * 0.5 + 0.5) * noise_amplitude_factor;
        magnitudes[i] += noise_val;
        // Magnitudes are now roughly in [0, 1 + max_noise_amplitude_factor (approx 0.08)]
    }

    // --- 5. Final scaling to [target_min_output, target_max_output] ---
    // Find the current actual min and max of the magnitudes after noise addition
    let mut actual_min = magnitudes[0];
    let mut actual_max = magnitudes[0];
    for i in 1..NUM_BINS {
        // Start from 1 since 0 is already assigned
        let val = magnitudes[i];
        if val < actual_min {
            actual_min = val;
        }
        if val > actual_max {
            actual_max = val;
        }
    }

    // If the signal is flat (all values are the same after all calculations)
    if (actual_max - actual_min).abs() < 1e-6 {
        let fill_value = (target_min_output + target_max_output) / 2.0;
        for i in 0..NUM_BINS {
            magnitudes[i] = fill_value;
        }
    } else {
        let current_actual_range = actual_max - actual_min;
        let desired_output_range = target_max_output - target_min_output;

        for i in 0..NUM_BINS {
            // Normalize the current value from its actual range [actual_min, actual_max] to [0, 1]
            let normalized_val = (magnitudes[i] - actual_min) / current_actual_range;
            // Scale and shift to the desired output range
            magnitudes[i] = target_min_output + normalized_val * desired_output_range;
        }
    }

    // Final clamp to ensure values are strictly within the target range,
    // handling any potential floating point inaccuracies.
    for i in 0..NUM_BINS {
        magnitudes[i] = magnitudes[i].max(target_min_output).min(target_max_output);
    }

    magnitudes
}

fn main() -> Result<(), std::convert::Infallible> {
    // Create a display with the specified width and height
    let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));

    // Create a window with the specified title and size
    let mut window = Window::new(
        "MicroViz Simulator",
        &OutputSettingsBuilder::new()
            .theme(BinaryColorTheme::Default)
            .build(),
    );

    // Create a drawing demo instance
    let mut demo = BarGragh::new(WIDTH as u16, HEIGHT as u16);

    // Initialize time variable for animation
    let mut time: f32 = 0.0;

    // Main loop
    loop {
        // Clear the display
        display.clear(BinaryColor::Off.into())?;

        let mut heights = simulate_fft_simple_output_scaling(time, 0.0, 250.0);
        // Update the drawing demo with the calculated heights
        heights[448..].fill(0.0);
        heights[0..64].fill(64.0);
        // heights = [0.0; NUM_BINS];
        demo.update(&mut display, &heights)?;

        // Draw a rectangle around the display area
        Rectangle::with_corners(
            Point::zero(),
            Point::new(WIDTH as i32 - 1, HEIGHT as i32 - 1),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(BLACK)
                .stroke_width(2)
                .build(),
        )
        .draw(&mut display)?;

        // Draw the display on the window
        window.update(&display);

        // Increment time for animation
        time += 0.05;

        // Wait for a frame delay
        thread::sleep(Duration::from_millis(FRAME_DELAY_MS));

        // Handle window events
        if let Some(event) = window.events().next() {
            if let SimulatorEvent::Quit = event {
                break;
            }
        }
    }

    Ok(())
}
