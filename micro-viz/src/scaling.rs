//! Scaling and normalization strategies for FFT data.
//!
//! This module provides different strategies for scaling FFT magnitude values
//! to display coordinates, including rolling max tracking and normalization methods.

// This is used but is used implicitly by the compiler
#[allow(unused_imports)] 
use micromath::F32Ext;

/// Tracks a rolling maximum value with decay.
///
/// This is useful for dynamic range scaling where the maximum value adapts
/// to the input signal over time.
pub struct RollingMaxTracker {
    current_max: f32,
    decay_factor: f32,
}

impl RollingMaxTracker {
    /// Creates a new rolling max tracker.
    ///
    /// # Arguments
    /// * `initial_max` - Starting maximum value
    /// * `decay_factor` - Decay factor (0.0-1.0). Higher values decay slower.
    ///   A value of 0.9 means the max decays by 10% each frame when no new max is found.
    pub fn new(initial_max: f32, decay_factor: f32) -> Self {
        Self {
            current_max: initial_max.max(0.0),
            decay_factor: decay_factor.max(0.0).min(1.0),
        }
    }

    /// Updates the tracker with new values and returns the current maximum.
    ///
    /// # Arguments
    /// * `values` - Slice of values to check for a new maximum
    pub fn update(&mut self, values: &[f32]) -> f32 {
        if let Some(&new_max) = values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()) {
            if new_max > self.current_max {
                self.current_max = new_max;
            } else {
                // Decay the maximum if no new max found
                self.current_max = self.current_max * self.decay_factor;
            }
        } else {
            // Empty slice, just decay
            self.current_max = self.current_max * self.decay_factor;
        }
        self.current_max
    }

    /// Gets the current maximum value.
    pub fn get_max(&self) -> f32 {
        self.current_max
    }

    /// Resets the tracker to a new initial maximum.
    pub fn reset(&mut self, initial_max: f32) {
        self.current_max = initial_max.max(0.0);
    }
}

/// Scaling strategy for converting FFT values to display coordinates.
pub trait ScalingStrategy {
    /// Scales a value from FFT magnitude space to display coordinate space.
    ///
    /// # Arguments
    /// * `value` - FFT magnitude value to scale
    /// * `max_display_value` - Maximum display coordinate (e.g., screen height)
    /// * `context` - Scaling context (may contain rolling max, etc.)
    fn scale(&self, value: f32, max_display_value: f32, context: &ScalingContext) -> f32;
}

/// Context passed to scaling strategies.
pub struct ScalingContext {
    /// Current rolling maximum value
    pub rolling_max: f32,
    /// Screen dimensions for reference
    pub screen_height: f32,
}

impl ScalingContext {
    pub fn new(rolling_max: f32, screen_height: f32) -> Self {
        Self {
            rolling_max: rolling_max.max(0.0),
            screen_height: screen_height.max(1.0),
        }
    }
}

/// Linear scaling relative to rolling maximum.
///
/// Scales values linearly: `(value / rolling_max) * max_display_value`
pub struct LinearRollingMaxScaling;

impl ScalingStrategy for LinearRollingMaxScaling {
    fn scale(&self, value: f32, max_display_value: f32, context: &ScalingContext) -> f32 {
        if context.rolling_max <= 0.0 {
            return 0.0;
        }
        let scaled = (value / context.rolling_max) * max_display_value;
        scaled.max(0.0).min(max_display_value)
    }
}

/// Logarithmic scaling for better visualization of frequency ranges.
///
/// Uses logarithmic scaling to emphasize lower values:
/// `log10(1 + value * 9) / log10(10) * max_display_value`
pub struct LogarithmicScaling;

impl ScalingStrategy for LogarithmicScaling {
    fn scale(&self, value: f32, max_display_value: f32, context: &ScalingContext) -> f32 {
        if context.rolling_max <= 0.0 {
            return 0.0;
        }
        // Normalize value to 0-1 range
        let normalized = (value / context.rolling_max).min(1.0).max(0.0);
        // Apply logarithmic scaling: log10(1 + normalized * 9) / log10(10)
        // Calculate log10 using natural log: log10(x) = ln(x) / ln(10)
        let val = 1.0 + normalized * 9.0;
        let log_val = val.ln();
        let log_10 = 10.0_f32.ln();
        let log_scaled = log_val / log_10;
        (log_scaled * max_display_value).max(0.0).min(max_display_value)
    }
}

/// Square root scaling for smooth transitions.
///
/// Uses square root scaling to compress high values:
/// `sqrt(value / rolling_max) * max_display_value`
pub struct SquareRootScaling;

impl ScalingStrategy for SquareRootScaling {
    fn scale(&self, value: f32, max_display_value: f32, context: &ScalingContext) -> f32 {
        if context.rolling_max <= 0.0 {
            return 0.0;
        }
        let normalized = (value / context.rolling_max).min(1.0).max(0.0);
        let sqrt_scaled = normalized.sqrt();
        (sqrt_scaled * max_display_value).max(0.0).min(max_display_value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rolling_max_tracker() {
        let mut tracker = RollingMaxTracker::new(1.0, 0.9);
        
        // Initial max
        assert_eq!(tracker.get_max(), 1.0);
        
        // Update with new max
        tracker.update(&[0.5, 2.0, 1.5]);
        assert_eq!(tracker.get_max(), 2.0);
        
        // Update with lower values (should decay)
        tracker.update(&[0.5, 1.0, 1.5]);
        assert_eq!(tracker.get_max(), 2.0 * 0.9);
    }

    #[test]
    fn test_linear_scaling() {
        let scaling = LinearRollingMaxScaling;
        let context = ScalingContext::new(10.0, 100.0);
        
        // Value at max should scale to max_display_value
        assert_eq!(scaling.scale(10.0, 100.0, &context), 100.0);
        // Half value should scale to half
        assert_eq!(scaling.scale(5.0, 100.0, &context), 50.0);
        // Zero should be zero
        assert_eq!(scaling.scale(0.0, 100.0, &context), 0.0);
    }

    #[test]
    fn test_logarithmic_scaling() {
        let scaling = LogarithmicScaling;
        let context = ScalingContext::new(10.0, 100.0);
        
        let result = scaling.scale(10.0, 100.0, &context);
        assert!(result > 0.0 && result <= 100.0);
        
        // Logarithmic should compress high values
        let linear = LinearRollingMaxScaling.scale(10.0, 100.0, &context);
        assert!(result <= linear);
    }
}

