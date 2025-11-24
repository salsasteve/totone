//! # micro-dsp
//!
//! A `no_std` compatible DSP (Digital Signal Processing) library for embedded audio analysis.
//!
//! This library provides efficient audio processing functions for real-time spectrum analysis,
//! including sample normalization, window functions, FFT computation, and magnitude spectrum calculation.
//!
//! ## Processing Pipeline
//!
//! The library implements a complete audio processing pipeline:
//!
//! 1. **Normalization**: Converts i16 samples to f32 range [-1.0, 1.0]
//! 2. **Windowing**: Applies Hann window to reduce spectral leakage
//! 3. **FFT**: Computes 1024-point real FFT (outputs 512 complex bins)
//! 4. **Magnitude**: Calculates magnitude spectrum from complex FFT output
//!
//! ## Example
//!
//! ```no_run
//! use micro_dsp::process_frame;
//!
//! let samples: [i16; 1024] = /* ... audio samples ... */;
//! match process_frame(&samples) {
//!     Ok(magnitude_spectrum) => {
//!         // magnitude_spectrum is [f32; 512]
//!         // Use bins for visualization
//!     }
//!     Err(e) => {
//!         // Handle error
//!     }
//! }
//! ```

#![no_std]

use core::i16;
use microdsp::common::{apply_window_function, real_fft, WindowFunctionType::Hann};
use microfft::Complex32;


/// Normalize a single sample from i16 to f32.
///
/// Converts an i16 sample value to the floating-point range [-1.0, 1.0].
/// This is the first step in the audio processing pipeline.
///
/// # Arguments
///
/// * `sample` - Input sample in i16 format
///
/// # Returns
///
/// Normalized sample value in f32 format, range [-1.0, 1.0]
///
/// # Example
///
/// ```
/// use micro_dsp::normalize_sample;
///
/// let sample: i16 = 16384;
/// let normalized = normalize_sample(sample);
/// // normalized is approximately 0.5
/// ```
pub fn normalize_sample(sample: i16) -> f32 {
    sample as f32 / i16::MAX as f32
}

/// Normalize a slice of i16 samples to a slice of f32 samples.
///
/// Converts a buffer of i16 audio samples to f32 format in the range [-1.0, 1.0].
/// This function processes the entire input buffer in one call.
///
/// # Arguments
///
/// * `samples` - Input slice of i16 samples
/// * `normalized_samples` - Output slice of f32 samples (must be same length as input)
///
/// # Panics
///
/// This function will panic if `normalized_samples` is shorter than `samples`.
///
/// # Example
///
/// ```
/// use micro_dsp::normalize_samples;
///
/// let samples: [i16; 1024] = [/* ... */];
/// let mut normalized = [0.0f32; 1024];
/// normalize_samples(&samples, &mut normalized);
/// ```
pub fn normalize_samples(samples: &[i16], normalized_samples: &mut [f32]) {
    for (i, &sample) in samples.iter().enumerate() {
        normalized_samples[i] = normalize_sample(sample);
    }
}

/// Apply a Hann window to a slice of f32 samples.
///
/// The Hann window function is applied to reduce spectral leakage in FFT analysis.
/// This is done in-place, modifying the input buffer.
///
/// # Arguments
///
/// * `samples` - Mutable slice of f32 samples to window (typically normalized)
///
/// # Notes
///
/// The Hann window reduces spectral leakage at the cost of some frequency resolution.
/// It's a good general-purpose window for audio analysis.
///
/// # Example
///
/// ```
/// use micro_dsp::{normalize_samples, apply_hann_window};
///
/// let samples: [i16; 1024] = [/* ... */];
/// let mut normalized = [0.0f32; 1024];
/// normalize_samples(&samples, &mut normalized);
/// apply_hann_window(&mut normalized);
/// ```
pub fn apply_hann_window(samples: &mut [f32]) {
    apply_window_function(Hann, samples);
}

/// Compute the FFT of a slice of f32 samples.
///
/// Performs a 1024-point real FFT on the input samples. The input buffer is modified
/// in-place during computation. The output contains 512 complex frequency bins.
///
/// # Arguments
///
/// * `samples` - Mutable array of exactly 1024 f32 samples (typically windowed)
///
/// # Returns
///
/// A mutable slice of 512 complex numbers representing the FFT output.
/// - Bin 0: DC component
/// - Bins 1-511: Frequency bins
/// - Bin 511: Nyquist frequency (for 48kHz sample rate: ~24kHz)
///
/// # Notes
///
/// The input buffer is modified during FFT computation. If you need to preserve
/// the original samples, make a copy before calling this function.
///
/// # Example
///
/// ```
/// use micro_dsp::compute_fft;
///
/// let mut samples = [0.0f32; 1024]; // Windowed samples
/// let fft_output = compute_fft(&mut samples);
/// // fft_output.len() == 512
/// ```
pub fn compute_fft(samples: &mut [f32; 1024]) -> &mut [Complex32] {
    real_fft(samples)
}

/// Compute the magnitude of the FFT output.
///
/// Calculates the magnitude spectrum from complex FFT output using the formula:
/// magnitude = sqrt(real² + imaginary²)
///
/// # Arguments
///
/// * `fft_output` - Slice of 512 complex numbers from FFT computation
///
/// # Returns
///
/// * `Ok([f32; 512])` - Array of 512 magnitude values, one per frequency bin
/// * `Err(&'static str)` - Error if input length is not exactly 512
///
/// # Example
///
/// ```
/// use micro_dsp::{compute_fft, compute_magnitude};
///
/// let mut samples = [0.0f32; 1024];
/// let fft_output = compute_fft(&mut samples);
/// let magnitude = compute_magnitude(fft_output)?;
/// // magnitude contains 512 magnitude values
/// # Ok::<(), &'static str>(())
/// ```
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
///
/// This is the main entry point for audio processing. It performs the complete
/// processing pipeline: normalization, windowing, FFT, and magnitude calculation.
///
/// # Arguments
///
/// * `samples` - Slice of exactly 1024 i16 audio samples
///
/// # Returns
///
/// * `Ok([f32; 512])` - Array of 512 magnitude values representing the frequency spectrum
/// * `Err(&'static str)` - Error if input length is not exactly 1024 samples
///
/// # Processing Steps
///
/// 1. Normalizes i16 samples to f32 range [-1.0, 1.0]
/// 2. Applies Hann window to reduce spectral leakage
/// 3. Computes 1024-point real FFT (produces 512 complex bins)
/// 4. Calculates magnitude spectrum from complex FFT output
///
/// # Example
///
/// ```
/// use micro_dsp::process_frame;
///
/// // Get 1024 samples from audio source
/// let samples: [i16; 1024] = [/* ... audio samples ... */];
///
/// // Process frame and get magnitude spectrum
/// match process_frame(&samples) {
///     Ok(magnitude_spectrum) => {
///         // Use magnitude_spectrum for visualization
///         // magnitude_spectrum[i] represents the magnitude at frequency bin i
///         for (bin, magnitude) in magnitude_spectrum.iter().enumerate() {
///             // Process each frequency bin
///         }
///     }
///     Err(e) => {
///         // Handle error (typically wrong input size)
///     }
/// }
/// ```
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
