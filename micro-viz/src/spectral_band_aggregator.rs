#![no_std]
extern crate alloc;

use alloc::{vec, vec::Vec};

#[allow(unused_imports)] 
use micromath::F32Ext;

pub enum BandSpreadModeConfig {
    Even,
    Exponential { exp_factor: f32 },
}

pub const NUMBER_OF_BANDS_IN_512: u32 = 512;
pub const NUMBER_OF_BANDS_IN_256: u32 = 256;
pub const NUMBER_OF_BANDS_IN_128: u32 = 128;
pub const NUMBER_OF_BANDS_IN_64: u32 = 64;
pub const NUMBER_OF_BANDS_IN_32: u32 = 32;

pub const NUMBER_OF_BANDS_OUT_64: u32 = 64;
pub const NUMBER_OF_BANDS_OUT_32: u32 = 32;
pub const NUMBER_OF_BANDS_OUT_16: u32 = 16;
pub const NUMBER_OF_BANDS_OUT_8: u32 = 8;
pub const NUMBER_OF_BANDS_OUT_4: u32 = 4;
pub const NUMBER_OF_BANDS_OUT_2: u32 = 2;

pub const VALID_BAND_SIZES_IN: [u32; 5] = [
    NUMBER_OF_BANDS_IN_512,
    NUMBER_OF_BANDS_IN_256,
    NUMBER_OF_BANDS_IN_128,
    NUMBER_OF_BANDS_IN_64,
    NUMBER_OF_BANDS_IN_32,
];

pub const VALID_BAND_SIZES_OUT: [u32; 6] = [
    NUMBER_OF_BANDS_OUT_64,
    NUMBER_OF_BANDS_OUT_32,
    NUMBER_OF_BANDS_OUT_16,
    NUMBER_OF_BANDS_OUT_8,
    NUMBER_OF_BANDS_OUT_4,
    NUMBER_OF_BANDS_OUT_2,
];

pub fn is_valid_input_band_size(value: u32) -> bool {
    VALID_BAND_SIZES_IN.contains(&value)
}

pub fn is_valid_output_band_size(value: u32) -> bool {
    VALID_BAND_SIZES_OUT.contains(&value)
}

pub struct SpectralBandAggregator {
    active_band_ranges: Vec<(u16, u16)>,
}

impl SpectralBandAggregator {
    pub fn new(
        number_of_bands_in: usize,
        number_of_bands_out: usize,
        band_spread_config: BandSpreadModeConfig,
    ) -> Self {
        if !is_valid_input_band_size(number_of_bands_in as u32) {
            panic!(
                "Invalid number of input bands: {}. Must be one of {:?}.",
                number_of_bands_in, VALID_BAND_SIZES_IN
            );
        }
        if !is_valid_output_band_size(number_of_bands_out as u32) {
            panic!(
                "Invalid number of output bands: {}. Must be one of {:?}.",
                number_of_bands_out, VALID_BAND_SIZES_OUT
            );
        }
        if number_of_bands_out == 0 {
            panic!("Number of bands out must be greater than 0");
        }
        if number_of_bands_in < number_of_bands_out {
            panic!("Number of bands in must be greater than or equal to number of bands out");
        }

        let active_band_ranges = match &band_spread_config {
            BandSpreadModeConfig::Even => {
                Self::calculate_even_spread_bands(number_of_bands_in, number_of_bands_out)
            }
            BandSpreadModeConfig::Exponential { exp_factor } => {
                if *exp_factor <= 0.0 {
                    panic!(
                        "Exponential factor must be greater than 0. Got: {}",
                        exp_factor
                    );
                }
                // (exp_factor - 1.0) being zero is handled in the calculation by effectively switching to linear.
                Self::calculate_exponential_spread_bands(
                    number_of_bands_in,
                    number_of_bands_out,
                    *exp_factor,
                )
            }
        };

        Self { active_band_ranges }
    }

    pub fn band_ranges(&self) -> &[(u16, u16)] {
        &self.active_band_ranges
    }

    fn calculate_even_spread_bands(
        number_of_bands_in: usize,
        number_of_bands_out: usize,
    ) -> Vec<(u16, u16)> {
        let band_size = number_of_bands_in / number_of_bands_out; // 512/8 = 32 bands from the FFT
        let mut band_ranges = vec![(0, 0); number_of_bands_out];

        for i in 0..number_of_bands_out {
            let start = i * band_size;
            let end = if i == number_of_bands_out - 1 {
                number_of_bands_in
            } else {
                start + band_size
            };
            band_ranges[i] = (start as u16, end as u16);
        }

        band_ranges
    }

    fn calculate_exponential_spread_bands(
        number_of_bands_in: usize,
        number_of_bands_out: usize,
        exp_factor: f32,
    ) -> Vec<(u16, u16)> {
        let mut band_ranges = vec![(0, 0); number_of_bands_out];

        for i in 0..number_of_bands_out {
            let start_fraction = i as f32 / number_of_bands_out as f32;
            let end_fraction = (i + 1) as f32 / number_of_bands_out as f32;

            let float_start_boundary = ((exp_factor.powf(start_fraction) - 1.0)
                / (exp_factor - 1.0))
                * number_of_bands_in as f32;
            let float_end_boundary = ((exp_factor.powf(end_fraction) - 1.0) / (exp_factor - 1.0))
                * number_of_bands_in as f32;

            let start_index = float_start_boundary as usize;
            let end_index = float_end_boundary as usize;

            let start_index = start_index.min(number_of_bands_in);
            let end_index = end_index.min(number_of_bands_in);
            band_ranges[i] = (start_index as u16, end_index as u16);
        }
        band_ranges
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn test_new_spectral_band_aggregator() {
        let sba = SpectralBandAggregator::new(512, 8, BandSpreadModeConfig::Even);
        assert_eq!(sba.active_band_ranges.len(), 8);
    }

    #[test]
    fn test_calculate_even_spread_bands() {
        let sba = SpectralBandAggregator::new(512, 4, BandSpreadModeConfig::Even);
        let band_ranges = sba.band_ranges();
        assert_eq!(band_ranges.len(), 4);
        assert_eq!(band_ranges[0], (0, 128));
        assert_eq!(band_ranges[1], (128, 256));
        assert_eq!(band_ranges[2], (256, 384));
        assert_eq!(band_ranges[3], (384, 512));
    }

    #[test]
    fn test_calculate_exponential_spread_bands() {
        let sba = SpectralBandAggregator::new(32, 4, BandSpreadModeConfig::Exponential{exp_factor: 7.0});
        let band_ranges = sba.band_ranges();
        assert_eq!(band_ranges.len(), 4);
        assert_eq!(band_ranges[0], (0, 3));
        assert_eq!(band_ranges[1], (3, 8));
        assert_eq!(band_ranges[2], (8, 17));
        assert_eq!(band_ranges[3], (17, 32));
    }
}
