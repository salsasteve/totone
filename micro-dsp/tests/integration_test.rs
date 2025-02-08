use micro_dsp::{apply_hann_window, normalize_samples, process_frame};
pub mod common;
use common::*;
use std::println;

const TOLERANCE: f32 = 1e-3;

#[test]
fn test_normalize_samples_real_world() {
    let sine_i16: [i16; 1024] = generate_sine_samples_i16();
    let expected_output: [f32; 1024] = generate_sine_samples_f32();
    let mut normalized_samples = [0.0; 1024];
    normalize_samples(&sine_i16, &mut normalized_samples);
    for (i, &normalized) in normalized_samples.iter().enumerate() {
        assert!(
            (normalized - expected_output[i]).abs() < TOLERANCE,
            "Expected {}, got {}",
            expected_output[i],
            normalized
        );
    }
}

#[test]
fn test_apply_hann_window_real_world_1024_sine_samples() {
    let expected_output = windowed_sine_wave();
    let sine_i16 = generate_sine_samples_i16();
    let mut normalized_samples = [0.0; 1024];
    normalize_samples(&sine_i16, &mut normalized_samples);

    // Apply the Hann window
    apply_hann_window(&mut normalized_samples);

    // Compare the actual output with the expected output
    for i in 0..1024 {
        // println!(
        //     "Index {}: Expected {}, got {}",
        //     i, expected_output[i], normalized_samples[i]
        // );
        assert!(
            (normalized_samples[i] - expected_output[i]).abs() < TOLERANCE,
            "Expected {}, got {} at index {}",
            expected_output[i],
            normalized_samples[i],
            i
        );
    }
}

#[test]
fn test_process_frame() {
    let sine_i16: [i16; 1024] = generate_sine_samples_i16();
    let expected_output: [f32; 512] = fft_data();

    let output = process_frame(&sine_i16).unwrap();
    const TOLERANCE: f32 = 1e-1;
    for (i, &value) in output.iter().enumerate() {
        println!(
            "Index {}: Expected {}, got {}",
            i, expected_output[i], value
        );
        assert!(
            (value - expected_output[i]).abs() < TOLERANCE,
            "Expected {}, got {} at index {}",
            expected_output[i],
            value,
            i
        );
    }
}
