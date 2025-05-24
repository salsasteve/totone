#![no_std]
#[allow(unused_imports)]
use micromath::F32Ext;

pub enum BinSummaryStrategy {
    Average,         // Use the average value of the bins
    WeightedAverage, // Use a weighted average of the bins
    Max,             // Use the maximum value of the bins
    RMS,             // Use the root mean square of the bins
}

impl BinSummaryStrategy {
    pub fn calculate(&self, bin_slice: &[f32]) -> f32 {
        if bin_slice.is_empty() {
            return 0.0;
        }
        let num_elements = bin_slice.len() as f32;

        match *self {
            BinSummaryStrategy::Average => bin_slice.iter().cloned().sum::<f32>() / num_elements,
            BinSummaryStrategy::Max => bin_slice
                .iter()
                .cloned()
                .fold(f32::NEG_INFINITY, f32::max)
                .max(0.0),
            BinSummaryStrategy::RMS => {
                let sum_of_squares: f32 = bin_slice.iter().map(|&x| x * x).sum();
                (sum_of_squares / num_elements).sqrt()
            }
            BinSummaryStrategy::WeightedAverage => {
                let mut weighted_sum: f32 = 0.0;
                let mut total_weight: f32 = 0.0;
                for (i, &x) in bin_slice.iter().enumerate() {
                    let weight = i as f32 + 1.0;
                    weighted_sum += x * weight;
                    total_weight += weight;
                }
                if total_weight == 0.0 {
                    0.0
                } else {
                    weighted_sum / total_weight
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_bin_summary_strategy() {
        let data = [1.0, 2.0, 3.0, 4.0, 5.0];

        assert_eq!(BinSummaryStrategy::Average.calculate(&data), 3.0);
        assert_eq!(BinSummaryStrategy::Max.calculate(&data), 5.0);
        let left = BinSummaryStrategy::RMS.calculate(&data);
        let right = (55.0 / 5.0).sqrt();
        assert_abs_diff_eq!(left, right, epsilon = 0.06);
        assert_eq!(
            BinSummaryStrategy::WeightedAverage.calculate(&data),
            3.6666667
        );
    }
}
