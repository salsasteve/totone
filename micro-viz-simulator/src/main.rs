use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};
use micro_viz::{
    Visualizer,
    VisualizationModeEnum,
    VisualizationConfig,
    FftArrayProvider,
    modes::bars::{BarsMode, SkinnyBarsMode},
    modes::waves::{SineWaveMode, SawtoothWaveMode, SquareWaveMode, ZigZagMode},
    color_mode::SpectrumColorMode,
};
use std::{thread, time::Duration};

// Constants for visualization parameters
pub const WIDTH: u32 = 256;
pub const HEIGHT: u32 = 64;
pub const FRAME_DELAY_MS: u64 = 1;
pub const BAR_WIDTH: u16 = 7;

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

/// Enhanced FFT simulation with better frequency response and harmonics.
///
/// This function generates more realistic FFT data with:
/// - Multiple harmonic frequencies
/// - Smooth frequency response curves
/// - Realistic amplitude decay
/// - Better temporal dynamics
pub fn simulate_fft_enhanced(
    time: f32,
    target_min_output: f32,
    target_max_output: f32,
) -> [f32; NUM_BINS] {
    if target_min_output >= target_max_output {
        panic!("target_min_output must be less than target_max_output");
    }
    if NUM_BINS == 0 {
        panic!("NUM_BINS constant must be greater than 0.");
    }

    // Enhanced frequency bands with harmonics
    let frequency_bands = [
        (20.0, 0.95, 2),    // Sub-bass with 2 harmonics
        (60.0, 0.90, 3),    // Bass with 3 harmonics
        (125.0, 0.85, 4),   // Low-mid with 4 harmonics
        (250.0, 0.80, 3),   // Mid with 3 harmonics
        (500.0, 0.75, 2),   // Upper-mid with 2 harmonics
        (1000.0, 0.70, 2),  // Presence with 2 harmonics
        (2000.0, 0.65, 2),  // Clarity with 2 harmonics
        (4000.0, 0.60, 1),  // Brilliance with 1 harmonic
        (8000.0, 0.50, 1),  // Air with 1 harmonic
        (12000.0, 0.40, 0), // High-end
        (16000.0, 0.30, 0), // Extended high
    ];

    let mut magnitudes = [0.0f32; NUM_BINS];

    // --- 1. Generate fundamental frequencies with harmonics ---
    for (fundamental_freq, base_amplitude, num_harmonics) in frequency_bands.iter() {
        // Generate time-varying amplitude with envelope
        let envelope_freq = *fundamental_freq * 0.008;
        let envelope_phase = time * envelope_freq + (*fundamental_freq / 1000.0);
        let envelope = (envelope_phase.sin() * 0.5 + 0.5).powf(1.5); // Smoother envelope
        
        // Add some randomness to make it more organic
        let mod_phase = time * *fundamental_freq * 0.012 + (*fundamental_freq / 500.0);
        let modulation = (mod_phase.sin() * 0.3 + 0.7);
        let activity = base_amplitude * envelope * modulation;

        // Add fundamental frequency
        add_frequency_contribution(&mut magnitudes, *fundamental_freq, activity, 1.0);

        // Add harmonics with decreasing amplitude
        for harmonic_num in 1..=*num_harmonics {
            let harmonic_freq = *fundamental_freq * (harmonic_num + 1) as f32;
            if harmonic_freq <= F_MAX {
                let harmonic_amplitude = activity / ((harmonic_num + 1) as f32 * 1.5);
                let harmonic_width = 1.0 / (harmonic_num + 1) as f32; // Narrower for higher harmonics
                add_frequency_contribution(&mut magnitudes, harmonic_freq, harmonic_amplitude, harmonic_width);
            }
        }
    }

    // --- 2. Apply realistic frequency response curve (pink noise tilt) ---
    for i in 0..NUM_BINS {
        let bin_pos_norm = if NUM_BINS > 1 {
            i as f32 / (NUM_BINS - 1) as f32
        } else {
            0.0
        };
        
        // Pink noise tilt: -3dB per octave
        // More realistic than flat response
        let freq_response = (1.0 - bin_pos_norm * 0.6).max(0.3);
        
        // Smooth roll-off in high frequencies
        let high_freq_rolloff = if bin_pos_norm > 0.8 {
            (1.0 - (bin_pos_norm - 0.8) / 0.2).max(0.1)
        } else {
            1.0
        };
        
        magnitudes[i] *= freq_response * high_freq_rolloff;
    }

    // --- 3. Normalize to [0, 1] range ---
    let mut max_val = 0.0f32;
    for &val in &magnitudes {
        if val > max_val {
            max_val = val;
        }
    }

    if max_val > 1e-6 {
        for i in 0..NUM_BINS {
            magnitudes[i] /= max_val;
        }
    }

    // --- 4. Add subtle noise with frequency-dependent characteristics ---
    for i in 0..NUM_BINS {
        let bin_pos_norm = if NUM_BINS > 1 {
            i as f32 / (NUM_BINS - 1) as f32
        } else {
            0.0
        };
        
        let bin_center_log_freq = *LOG_F_MIN + bin_pos_norm * *LOG_F_RANGE;
        let bin_freq = bin_center_log_freq.exp();

        // High frequencies have more noise (thermal noise characteristic)
        let noise_level = 0.03 + bin_pos_norm * 0.02;
        
        let noise_phase = time * bin_freq * 0.003
            + (i as f32 * 0.37)
            + (i as f32 * PI * 7.3 / NUM_BINS as f32).sin() * 0.5;
        
        let noise_val = (noise_phase.sin() * 0.5 + 0.5) * noise_level;
        magnitudes[i] = (magnitudes[i] + noise_val).min(1.0);
    }

    // --- 5. Apply smoothing filter to reduce aliasing ---
    // Simple 3-point moving average
    let mut smoothed = [0.0f32; NUM_BINS];
    for i in 0..NUM_BINS {
        let left = if i > 0 { magnitudes[i - 1] } else { magnitudes[i] };
        let center = magnitudes[i];
        let right = if i < NUM_BINS - 1 { magnitudes[i + 1] } else { magnitudes[i] };
        smoothed[i] = (left + center * 2.0 + right) / 4.0;
    }
    magnitudes = smoothed;

    // --- 6. Scale to target range ---
    let mut actual_min = magnitudes[0];
    let mut actual_max = magnitudes[0];
    for &val in &magnitudes {
        if val < actual_min {
            actual_min = val;
        }
        if val > actual_max {
            actual_max = val;
        }
    }

    let range = actual_max - actual_min;
    if range > 1e-6 {
        let output_range = target_max_output - target_min_output;
        for i in 0..NUM_BINS {
            let normalized = (magnitudes[i] - actual_min) / range;
            magnitudes[i] = target_min_output + normalized * output_range;
        }
    } else {
        let fill_value = (target_min_output + target_max_output) / 2.0;
        for i in 0..NUM_BINS {
            magnitudes[i] = fill_value;
        }
    }

    // Final clamp
    for i in 0..NUM_BINS {
        magnitudes[i] = magnitudes[i].max(target_min_output).min(target_max_output);
    }

    magnitudes
}

/// Helper function to add a frequency contribution to the magnitude array.
fn add_frequency_contribution(
    magnitudes: &mut [f32; NUM_BINS],
    frequency: f32,
    amplitude: f32,
    width_factor: f32,
) {
    if frequency < F_MIN || frequency > F_MAX {
        return;
    }

    let log_freq = frequency.ln();
    let adjusted_std_dev = PEAK_LOG_STD_DEV * width_factor;
    let inv_2_sigma_sq = 1.0 / (2.0 * adjusted_std_dev * adjusted_std_dev);

    for i in 0..NUM_BINS {
        let bin_pos_norm = if NUM_BINS > 1 {
            i as f32 / (NUM_BINS - 1) as f32
        } else {
            0.0
        };
        
        let bin_center_log_freq = *LOG_F_MIN + bin_pos_norm * *LOG_F_RANGE;
        let log_dist_sq = (bin_center_log_freq - log_freq).powi(2);
        let influence = (-log_dist_sq * inv_2_sigma_sq).exp();
        magnitudes[i] += amplitude * influence;
    }
}

fn main() -> Result<(), std::convert::Infallible> {
    // Create a display with the specified width and height
    let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));

    // Create a window with the specified title and size
    let mut window = Window::new(
        "MicroViz Simulator - Auto-cycling through visualization modes",
        &OutputSettingsBuilder::new()
            .theme(BinaryColorTheme::Default)
            .scale(2)
            .build(),
    );

    // Create visualization configuration
    let config = VisualizationConfig::builder(
        WIDTH as u16,
        HEIGHT as u16,
        BAR_WIDTH,
    )
    .build();

    // Create initial visualization mode (BarsMode with spectrum colors)
    let initial_mode = VisualizationModeEnum::Bars(
        BarsMode::new(std::boxed::Box::new(SpectrumColorMode))
    );

    // Create the visualizer with the new modular architecture
    let mut visualizer = Visualizer::new(initial_mode, config);

    // Initialize time variable for animation
    let mut time: f32 = 0.0;
    
    // Track current mode for switching
    let mut current_mode_index: usize = 0;
    let mode_names = vec![
        "Bars",
        "Skinny Bars",
        "Sine Wave",
        "Sawtooth Wave",
        "Square Wave",
        "ZigZag",
    ];
    
    // Helper function to create a mode by index
    let create_mode = |index: usize| -> VisualizationModeEnum {
        match index {
            0 => VisualizationModeEnum::Bars(BarsMode::new(std::boxed::Box::new(SpectrumColorMode))),
            1 => VisualizationModeEnum::SkinnyBars(SkinnyBarsMode::new(std::boxed::Box::new(SpectrumColorMode))),
            2 => VisualizationModeEnum::SineWave(SineWaveMode::new(std::boxed::Box::new(SpectrumColorMode))),
            3 => VisualizationModeEnum::SawtoothWave(SawtoothWaveMode::new(std::boxed::Box::new(SpectrumColorMode))),
            4 => VisualizationModeEnum::SquareWave(SquareWaveMode::new(std::boxed::Box::new(SpectrumColorMode))),
            5 => VisualizationModeEnum::ZigZag(ZigZagMode::new(std::boxed::Box::new(SpectrumColorMode))),
            _ => VisualizationModeEnum::Bars(BarsMode::new(std::boxed::Box::new(SpectrumColorMode))),
        }
    };
    
    // Auto-cycle through modes every 5 seconds
    let mode_cycle_interval = 5.0;
    let mut last_mode_change_time = time;

    // Main loop
    loop {
        // Clear the display
        display.clear(BinaryColor::Off.into())?;

        // Generate FFT data using the enhanced simulator function
        let fft_data = simulate_fft_enhanced(time, 0.0, 250.0);
        
        // Create FFT provider using the new modular FFT interface
        let fft_provider = FftArrayProvider::new(&fft_data);

        // Update and render using the new visualizer
        visualizer.update(&mut display, &fft_provider)?;

        // Draw the display on the window
        window.update(&display);

        // Increment time for animation
        time += 0.05;

        // Wait for a frame delay
        thread::sleep(Duration::from_millis(FRAME_DELAY_MS));

        // Auto-cycle through modes
        if time - last_mode_change_time >= mode_cycle_interval {
            current_mode_index = (current_mode_index + 1) % mode_names.len();
            let new_mode = create_mode(current_mode_index);
            visualizer.set_mode(new_mode);
            println!("Switched to mode: {} (Index: {})", mode_names[current_mode_index], current_mode_index);
            last_mode_change_time = time;
        }

        // Handle window events
        if let Some(event) = window.events().next() {
            match event {
                SimulatorEvent::Quit => break,
                _ => {}
            }
        }
    }

    Ok(())
}
