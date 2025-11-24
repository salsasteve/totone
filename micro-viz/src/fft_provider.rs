//! Generic FFT data provider interface.
//!
//! This module provides a trait-based interface for FFT magnitude data that works
//! with any visualization mode. This decouples visualization logic from specific
//! FFT formats or libraries.

use crate::bin_summary_strategy::BinSummaryStrategy;

/// Trait for providing FFT magnitude data to visualization modes.
///
/// This trait abstracts over different FFT data sources, allowing visualization
/// modes to work with any format (512 bins, 256 bins, custom sizes, etc.).
pub trait FftDataProvider {
    /// Returns a slice of all magnitude bins.
    fn get_magnitude_bins(&self) -> &[f32];
    
    /// Returns the number of FFT bins.
    fn num_bins(&self) -> usize {
        self.get_magnitude_bins().len()
    }
    
    /// Gets a single bin value by index.
    ///
    /// # Panics
    /// Panics if index is out of bounds.
    fn get_bin_value(&self, index: usize) -> f32 {
        self.get_magnitude_bins()[index]
    }
    
    /// Gets an aggregated value for a range of bins using a summary strategy.
    ///
    /// # Arguments
    /// * `start` - Start index (inclusive)
    /// * `end` - End index (exclusive)
    /// * `strategy` - Strategy for aggregating the bin values
    ///
    /// # Returns
    /// Aggregated value for the range
    fn get_band_aggregate(
        &self,
        start: usize,
        end: usize,
        strategy: BinSummaryStrategy,
    ) -> f32 {
        let bins = self.get_magnitude_bins();
        if start >= end || end > bins.len() {
            return 0.0;
        }
        strategy.calculate(&bins[start..end])
    }
}

/// Simple array-based FFT data provider.
///
/// Wraps a slice of f32 magnitude values. This is the most common case.
pub struct FftArrayProvider<'a> {
    bins: &'a [f32],
}

impl<'a> FftArrayProvider<'a> {
    /// Creates a new FFT array provider from a slice.
    pub fn new(bins: &'a [f32]) -> Self {
        Self { bins }
    }
}

impl<'a> FftDataProvider for FftArrayProvider<'a> {
    fn get_magnitude_bins(&self) -> &[f32] {
        self.bins
    }
}

/// Fixed-size array FFT data provider.
///
/// Useful when you have a fixed-size array like `[f32; 512]`.
impl<const N: usize> FftDataProvider for [f32; N] {
    fn get_magnitude_bins(&self) -> &[f32] {
        self
    }
}

/// Vector-based FFT data provider.
///
/// Wraps a `Vec<f32>` of magnitude values.
pub struct FftVecProvider {
    bins: alloc::vec::Vec<f32>,
}

impl FftVecProvider {
    /// Creates a new FFT vector provider.
    pub fn new(bins: alloc::vec::Vec<f32>) -> Self {
        Self { bins }
    }
    
    /// Creates from an iterator.
    pub fn from_iter<I: Iterator<Item = f32>>(iter: I) -> Self {
        Self {
            bins: iter.collect(),
        }
    }
}

impl FftDataProvider for FftVecProvider {
    fn get_magnitude_bins(&self) -> &[f32] {
        &self.bins
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bin_summary_strategy::BinSummaryStrategy;

    #[test]
    fn test_array_provider() {
        let bins = [1.0, 2.0, 3.0, 4.0, 5.0];
        let provider = FftArrayProvider::new(&bins);
        
        assert_eq!(provider.num_bins(), 5);
        assert_eq!(provider.get_bin_value(0), 1.0);
        assert_eq!(provider.get_bin_value(4), 5.0);
    }

    #[test]
    fn test_fixed_array_provider() {
        let bins: [f32; 3] = [1.0, 2.0, 3.0];
        
        assert_eq!(bins.num_bins(), 3);
        assert_eq!(bins.get_bin_value(1), 2.0);
    }

    #[test]
    fn test_band_aggregate() {
        let bins = [1.0, 2.0, 3.0, 4.0, 5.0];
        let provider = FftArrayProvider::new(&bins);
        
        let max = provider.get_band_aggregate(0, 3, BinSummaryStrategy::Max);
        assert_eq!(max, 3.0);
        
        let avg = provider.get_band_aggregate(0, 3, BinSummaryStrategy::Average);
        assert_eq!(avg, 2.0);
    }

    #[test]
    fn test_vec_provider() {
        let bins = alloc::vec![1.0, 2.0, 3.0];
        let provider = FftVecProvider::new(bins);
        
        assert_eq!(provider.num_bins(), 3);
        assert_eq!(provider.get_bin_value(1), 2.0);
    }
}


