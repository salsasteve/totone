#![no_std]

use core::i16;
use microdsp::common::{apply_window_function, real_fft, WindowFunctionType::Hann};
use microfft::Complex32;
use micromath::F32Ext;

// pub fn normalize_sample(sample: i16) -> f32 {
//     sample as f32 / i16::MAX as f32
// }

// pub fn normalize_samples(samples: &[i16], normalized_samples: &mut [f32]) {
//     for (i, &sample) in samples.iter().enumerate() {
//         normalized_samples[i] = normalize_sample(sample);
//     }
// }

// pub fn apply_hann_window(
//     samples: &[i16],
//     windowed_samples: &mut [f32],
// ) -> Result<(), &'static str> {
//     let length = windowed_samples.len();
//     if length > samples.len() {
//         return Err("Length exceeds sample size");
//     }
//     // check for valid length of 1024
//     if length != 1024 {
//         return Err("Length must be 1024");
//     }

//     normalize_samples(samples, windowed_samples);

//     apply_window_function(Hann, windowed_samples);

//     Ok(())
// }

// fn apply_hann_window(samples: &mut [f32]) {
//     apply_window_function(Hann, samples);
// }

// fn compute_fft(samples: &mut [f32]) -> [microfft::Complex32; 1024] {
//     real_fft(samples)
// }

// fn compute_magnitude(fft_output: &[microfft::Complex32; 512]) -> [f32; 512] {
//     let mut magnitude = [0.0; 512];
//     for i in 0..512 {
//         magnitude[i] = fft_output[i].hypot(fft_output[i + 512]);
//     }
//     magnitude
// }

/// Normalize a single sample from i16 to f32.
pub fn normalize_sample(sample: i16) -> f32 {
    sample as f32 / i16::MAX as f32
}

/// Normalize a slice of i16 samples to a slice of f32 samples.
pub fn normalize_samples(samples: &[i16], normalized_samples: &mut [f32]) {
    for (i, &sample) in samples.iter().enumerate() {
        normalized_samples[i] = normalize_sample(sample);
    }
}

/// Apply a Hann window to a slice of f32 samples.
pub fn apply_hann_window(samples: &mut [f32]) {
    apply_window_function(Hann, samples);
}

/// Compute the FFT of a slice of f32 samples.
pub fn compute_fft(samples: &mut [f32; 1024]) -> &mut [Complex32] {
    real_fft(samples)
}

/// Compute the magnitude of the FFT output.
pub fn compute_magnitude(fft_output: &[Complex32]) -> Result<[f32; 512], &'static str> {
    if fft_output.len() != 512 {
        return Err("FFT output must contain exactly 512 complex numbers");
    }
    let mut magnitude = [0.0; 512];
    for i in 0..512 {
        let component = fft_output[i];
        magnitude[i] = (component.re * component.re + component.im * component.im).sqrt();
    }
    Ok(magnitude)
}

/// Process a frame of i16 samples and return the magnitude spectrum.
pub fn process_frame(samples: &[i16]) -> Result<[f32; 512], &'static str> {
    if samples.len() != 1024 {
        return Err("Input must contain exactly 1024 samples");
    }

    // Step 1: Normalize samples
    let mut normalized_samples = [0.0; 1024];
    normalize_samples(samples, &mut normalized_samples);

    // Step 2: Apply Hann window
    apply_hann_window(&mut normalized_samples);

    // Step 3: Compute FFT
    let fft_output = compute_fft(&mut normalized_samples);

    // Step 4: Compute magnitude spectrum
    let magnitude = compute_magnitude(&fft_output).unwrap();

    Ok(magnitude)
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_compute_magnitude() {
        // Create a simple FFT output array with 512 elements
        let mut fft_output = [microfft::Complex32 { re: 0.0, im: 0.0 }; 512];

        // Set test values
        fft_output[0] = microfft::Complex32 { re: 1.0, im: 0.0 }; // DC component
        fft_output[1] = microfft::Complex32 { re: 0.0, im: 1.0 }; // Bin 1
        fft_output[511] = microfft::Complex32 { re: -1.0, im: 0.0 }; // Nyquist bin

        let magnitude = compute_magnitude(&fft_output).unwrap();

        // Check magnitudes
        assert_eq!(magnitude[0], 1.0); // sqrt(1^2 + 0^2)
        assert_eq!(magnitude[1], 1.0); // sqrt(0^2 + 1^2)
        assert_eq!(magnitude[511], 1.0); // sqrt((-1)^2 + 0^2)
    }
}
