#![no_std]
extern crate alloc;

use alloc::{vec, vec::Vec};

#[allow(unused_imports)] 
use micromath::F32Ext;

#[cfg(feature = "std")]
extern crate std; // For println in debug/error cases, guarded by feature flag

/// Manages the state and animation of a set of numerical values.
///
/// This struct handles the interpolation between previous, target, and current values
/// over a specified number of steps, using an easing function. It's designed to
/// animate values like bar heights in a spectrum visualizer.
/// It operates on `f32` for internal calculations and provides `u16` for display,
/// ensuring values are clamped to a specified maximum displayable value.
pub struct SpectrumValueAnimator {
    /// The values at the start of the current interpolation cycle (f32 for calculation).
    previous_values_calc: Vec<f32>,
    /// The target values for the current interpolation cycle (f32 for calculation).
    target_values_calc: Vec<f32>,
    /// The currently interpolated values, ready for display (u16).
    current_values_display: Vec<u16>,
    /// The number of steps over which to interpolate from previous to target values.
    /// Higher values mean slower, smoother animation. A value of 0 means instant snap to target.
    interpolation_steps: u32,
    /// Tracks the current step in the interpolation cycle.
    /// Ranges from 0 to `interpolation_steps - 1`.
    interpolation_counter: u32,
    /// The number of values being animated (e.g., number of bars).
    num_values: usize,
    /// The maximum displayable value for the u16 representation (e.g., screen_height - 1).
    max_display_value: u16,
}

impl SpectrumValueAnimator {
    /// Creates a new `SpectrumValueAnimator`.
    ///
    /// # Arguments
    /// * `num_values`: The number of individual values to animate.
    /// * `initial_value_calc`: The initial value for internal calculations (f32).
    /// * `interpolation_steps`: The number of frames for the animation. 0 means instant.
    /// * `max_display_value`: The maximum value the `u16` display representation can take.
    pub fn new(
        num_values: usize,
        initial_value_calc: f32,
        interpolation_steps: u32,
        max_display_value: u16,
    ) -> Self {
        let initial_value_display_clamped = initial_value_calc
            .max(0.0)
            .min(max_display_value as f32) as u16;

        Self {
            previous_values_calc: vec![initial_value_calc; num_values],
            target_values_calc: vec![initial_value_calc; num_values],
            current_values_display: vec![initial_value_display_clamped; num_values],
            interpolation_steps: interpolation_steps.max(0), // Ensure non-negative
            interpolation_counter: 0, // Start at the beginning of an interpolation cycle
            num_values,
            max_display_value,
        }
    }

    /// Sets new target values for the animation.
    ///
    /// This should be called when `is_new_cycle_start()` returns true.
    /// The animator's current `target_values_calc` become the `previous_values_calc`
    /// for the new animation cycle.
    ///
    /// # Arguments
    /// * `new_targets_calc`: A slice of `f32` values representing the new targets.
    ///   These should already be scaled by the caller (e.g., to pixel heights).
    pub fn set_new_targets(&mut self, new_targets_calc: &[f32]) {
        if new_targets_calc.len() != self.num_values {
            #[cfg(feature = "std")]
            std::println!(
                "SpectrumValueAnimator Error: new_targets_calc length ({}) does not match num_values ({}). Targets not updated.",
                new_targets_calc.len(),
                self.num_values
            );
            // In a no_std environment, consider a panic or a specific error type if this is critical.
            return;
        }

        // The values that were the target of the *completed* animation cycle
        // now become the starting point (previous_values_calc) for the new cycle.
        self.previous_values_calc.copy_from_slice(&self.target_values_calc);

        // Update self.target_values_calc with the new incoming targets.
        self.target_values_calc.copy_from_slice(new_targets_calc);

        // Reset interpolation counter for the new cycle.
        self.interpolation_counter = 0;
    }

    /// Returns the current value of the interpolation counter.
    /// Primarily for debugging or testing.
    pub fn get_interpolation_counter(&self) -> u32 {
        self.interpolation_counter
    }

    /// Allows changing the number of interpolation steps dynamically.
    /// Resets the current animation cycle.
    pub fn set_interpolation_steps(&mut self, steps: u32) {
        self.interpolation_steps = steps.max(0); // Ensure non-negative
        self.interpolation_counter = 0; // Reset to start of a new cycle
    }

    /// Updates the animation state for one frame and returns the current display values.
    ///
    /// This method calculates the eased progress, interpolates the values,
    /// and advances the interpolation counter.
    pub fn update_and_get_current_values(&mut self) -> &Vec<u16> {
        let eased_progress = self.calculate_eased_progress();
        self.interpolate_current_display_values(eased_progress);

        // Advance interpolation counter for the next frame.
        // If interpolation_steps is 0, counter remains 0 (progress is always 1.0).
        // If counter reaches interpolation_steps - 1, it stays there until reset by set_new_targets,
        // indicating the animation to the current target is complete.
        if self.interpolation_steps > 0 && self.interpolation_counter < self.interpolation_steps.saturating_sub(1) {
            self.interpolation_counter += 1;
        }
        &self.current_values_display
    }

    /// Checks if the animator is at the start of a new interpolation cycle,
    /// meaning the previous animation has completed or it's the very first frame.
    /// This is the signal for the caller to provide new target values.
    pub fn is_new_cycle_start(&self) -> bool {
        if self.interpolation_steps == 0 {
            // If 0 steps, every frame is effectively a "new cycle start" as values snap instantly.
            return true;
        }
        // A new cycle starts if the counter is at 0 (very first frame, or just reset by set_new_targets)
        // OR if the counter has reached the end of the previous interpolation period.
        self.interpolation_counter == 0 || self.interpolation_counter >= self.interpolation_steps.saturating_sub(1)
    }

    /// Easing function: ease-out quadratic. `t` is progress from 0.0 to 1.0.
    fn ease_out_quad(&self, t: f32) -> f32 {
        let t_clamped = t.max(0.0).min(1.0); // Ensure t is within [0.0, 1.0]
        1.0 - (1.0 - t_clamped) * (1.0 - t_clamped)
    }

    /// Calculates the current interpolation progress using easing.
    /// Ensures progress reaches 1.0 at the end of the interpolation period.
    fn calculate_eased_progress(&self) -> f32 {
        let progress = if self.interpolation_steps == 0 {
            // If 0 steps, jump directly to target. Progress is effectively 1.0.
            1.0
        } else if self.interpolation_steps == 1 {
            // If 1 step, interpolation_counter is 0. (steps-1) is 0.
            // Raw progress is 0/0 (NaN) or 1.0 depending on interpretation.
            // Effectively, it should snap, so progress is 1.0.
             1.0
        }
        else {
            // For interpolation_steps > 1
            // self.interpolation_counter ranges from 0 to self.interpolation_steps - 1.
            // Division by (self.interpolation_steps - 1) makes raw progress span [0.0, 1.0].
            self.interpolation_counter as f32 / (self.interpolation_steps - 1) as f32
        };
        self.ease_out_quad(progress)
    }

    /// Interpolates `self.current_values_display` based on `previous_values_calc`,
    /// `target_values_calc`, and eased progress. Clamps to `max_display_value`.
    fn interpolate_current_display_values(&mut self, eased_progress: f32) {
        if eased_progress >= 1.0 || self.interpolation_steps == 0 {
            // Snap to target values if progress is 1.0 or no interpolation steps.
            for i in 0..self.num_values {
                let display_val = self.target_values_calc[i]
                    .max(0.0) // Ensure non-negative before casting
                    .min(self.max_display_value as f32) as u16; // Clamp and cast
                self.current_values_display[i] = display_val;
            }
            return;
        }

        for i in 0..self.num_values {
            let prev_calc = self.previous_values_calc[i];
            let target_calc = self.target_values_calc[i];
            let interpolated_calc = prev_calc * (1.0 - eased_progress) + target_calc * eased_progress;

            let display_val = interpolated_calc
                .max(0.0) // Ensure non-negative before casting
                .min(self.max_display_value as f32) as u16; // Clamp and cast
            self.current_values_display[i] = display_val;
        }
    }

    /// Returns the configured maximum displayable u16 value.
    pub fn get_max_display_value(&self) -> u16 {
        self.max_display_value
    }

    /// Returns the configured number of interpolation steps.
    pub fn get_interpolation_steps(&self) -> u32 {
        self.interpolation_steps
    }

    // For more detailed logging, if you want to see the animator's internal calculation vectors:
    pub fn get_previous_values_calc(&self) -> &Vec<f32> {
        &self.previous_values_calc
    }

    pub fn get_target_values_calc(&self) -> &Vec<f32> {
        &self.target_values_calc
    }

    pub fn get_current_values_display(&self) -> &Vec<u16> {
        &self.current_values_display
    }

}


#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn test_spectrum_value_animator() {
        let mut animator = SpectrumValueAnimator::new(5, 0.0, 10, 100);

        // Test initial state
        assert_eq!(animator.previous_values_calc, vec![0.0; 5]);
        assert_eq!(animator.target_values_calc, vec![0.0; 5]);
        assert_eq!(animator.current_values_display, vec![0; 5]);

        // Set new targets
        animator.set_new_targets(&[10.0, 20.0, 30.0, 40.0, 50.0]);
        assert_eq!(animator.target_values_calc, vec![10.0, 20.0, 30.0, 40.0, 50.0]);

        // Update and get current values
        let current_values = animator.update_and_get_current_values();
        assert_eq!(current_values.len(), 5);
    }

    #[test]
    fn test_initial_state_and_set_targets() {
        let num_values = 5;
        let initial_val = 0.0;
        let steps = 10;
        let max_display = 100;
        let mut animator =
            SpectrumValueAnimator::new(num_values, initial_val, steps, max_display);

        // Test initial state
        assert_eq!(animator.previous_values_calc, vec![initial_val; num_values]);
        assert_eq!(animator.target_values_calc, vec![initial_val; num_values]);
        assert_eq!(
            animator.current_values_display,
            vec![initial_val as u16; num_values]
        );
        assert_eq!(animator.interpolation_steps, steps);
        assert_eq!(animator.get_interpolation_counter(), 0, "Initial counter should be 0");
        assert_eq!(animator.num_values, num_values);
        assert_eq!(animator.max_display_value, max_display);
        assert!(animator.is_new_cycle_start(), "Should be new cycle at start");


        // Set new targets
        let new_targets = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        animator.set_new_targets(&new_targets);
        assert_eq!(animator.previous_values_calc, vec![initial_val; num_values], "Previous should be old targets after set_new_targets");
        assert_eq!(animator.target_values_calc, new_targets);
        assert_eq!(animator.get_interpolation_counter(), 0, "Counter should reset after set_new_targets");
        assert!(animator.is_new_cycle_start(), "Should be new cycle after set_new_targets");
    }

    #[test]
    fn test_interpolation_counter_increment_normal() {
        let steps = 3; // Test with a small number of steps
        let mut animator = SpectrumValueAnimator::new(1, 0.0, steps, 100);

        assert_eq!(animator.get_interpolation_counter(), 0, "Initial");
        assert!(animator.is_new_cycle_start(), "Initial cycle start");

        animator.set_new_targets(&[100.0]); // Set some targets to start a cycle
        assert_eq!(animator.get_interpolation_counter(), 0, "After set_new_targets");
        assert!(animator.is_new_cycle_start(), "Cycle start after set_new_targets");


        // First update
        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 1, "After 1st update");
        assert!(!animator.is_new_cycle_start(), "Not new cycle after 1st update");


        // Second update (counter should be steps - 1)
        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 2, "After 2nd update (steps - 1)");
        assert!(animator.is_new_cycle_start(), "New cycle should start when counter is at steps - 1");

        // Third update (counter should remain at steps - 1)
        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 2, "After 3rd update (should stay at steps - 1)");
        assert!(animator.is_new_cycle_start(), "Still new cycle as animation is complete");


        // Reset with new targets
        animator.set_new_targets(&[50.0]);
        assert_eq!(animator.get_interpolation_counter(), 0, "Counter reset after 2nd set_new_targets");
        assert!(animator.is_new_cycle_start(), "New cycle after 2nd set_new_targets");
    }

    #[test]
    fn test_interpolation_counter_zero_steps() {
        let steps = 0; // Instant interpolation
        let mut animator = SpectrumValueAnimator::new(1, 0.0, steps, 100);

        assert_eq!(animator.get_interpolation_counter(), 0, "Initial counter for 0 steps");
        assert!(animator.is_new_cycle_start(), "Initial: 0 steps means always new cycle");


        animator.set_new_targets(&[100.0]);
        assert_eq!(animator.get_interpolation_counter(), 0, "After set_new_targets for 0 steps");
        assert!(animator.is_new_cycle_start(), "After set_new_targets: 0 steps means always new cycle");


        animator.update_and_get_current_values();
        // For interpolation_steps == 0, the counter does not increment:
        // `if self.interpolation_steps > 0 && ...` will be false.
        assert_eq!(animator.get_interpolation_counter(), 0, "After update for 0 steps, counter stays 0");
        assert!(animator.is_new_cycle_start(), "After update: 0 steps means always new cycle");


        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 0, "After second update for 0 steps, counter stays 0");
        assert!(animator.is_new_cycle_start(), "After 2nd update: 0 steps means always new cycle");
    }

    #[test]
    fn test_interpolation_counter_one_step() {
        let steps = 1; // Single step interpolation
        let mut animator = SpectrumValueAnimator::new(1, 0.0, steps, 100);

        assert_eq!(animator.get_interpolation_counter(), 0, "Initial counter for 1 step");
        assert!(animator.is_new_cycle_start(), "Initial: 1 step means new cycle (counter 0 >= steps-1 (0))");

        animator.set_new_targets(&[100.0]);
        assert_eq!(animator.get_interpolation_counter(), 0, "After set_new_targets for 1 step");
        assert!(animator.is_new_cycle_start(), "Cycle start for 1 step");

        // In `update_and_get_current_values`:
        // `self.interpolation_steps.saturating_sub(1)` will be `1 - 1 = 0`.
        // The condition `self.interpolation_counter < 0` (since counter is 0) will be false.
        // So, the counter should not increment.
        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 0, "After 1st update for 1 step, counter stays 0");
        // is_new_cycle_start checks `self.interpolation_counter >= self.interpolation_steps.saturating_sub(1)`
        // which is `0 >= (1-1)`, so `0 >= 0`, which is true.
        assert!(animator.is_new_cycle_start(), "After 1st update: 1 step means new cycle (animation complete)");

        animator.update_and_get_current_values();
        assert_eq!(animator.get_interpolation_counter(), 0, "After 2nd update for 1 step, counter stays 0");
        assert!(animator.is_new_cycle_start(), "After 2nd update: 1 step, new cycle");
    }

    #[test]
    fn test_animation_progression() {
        let num_values = 1;
        let steps = 2; // 2 steps: counter goes 0, 1
        let max_display = 100;
        let mut animator = SpectrumValueAnimator::new(num_values, 0.0, steps, max_display);

        animator.set_new_targets(&[100.0]); // Target is 100.0, previous is 0.0

        // Frame 0 (after set_new_targets, before first update_and_get_current_values)
        // current_values_display is still initial
        // interpolation_counter is 0
        // calculate_eased_progress: counter=0, steps=2 => progress = 0 / (2-1) = 0. ease(0)=0.
        // interpolate_current_display_values with progress 0 -> display should be previous (0)
        // THIS IS A SUBTLE POINT: update_and_get_current_values CALCULATES THEN INCREMENTS.
        // So the *returned* value is for counter=0. Then counter becomes 1.

        // Update 1: counter will be 0 for calculation, then becomes 1
        let values1 = animator.update_and_get_current_values();
        // Eased progress for counter 0, steps 2:
        // raw_progress = 0 / (2-1) = 0.0. eased_progress = ease_out_quad(0.0) = 0.0
        // interpolated = 0.0 * (1-0) + 100.0 * 0 = 0
        assert_eq!(values1[0], 0, "Values after 1st update (progress 0)");
        assert_eq!(animator.get_interpolation_counter(), 1, "Counter after 1st update");
        assert!(animator.is_new_cycle_start(), "Cycle is complete after 1st update, counter is at steps-1");


        // Update 2: counter will be 1 for calculation, then becomes 1 (max for steps=2)
        let values2 = animator.update_and_get_current_values();
        // Eased progress for counter 1, steps 2:
        // raw_progress = 1 / (2-1) = 1.0. eased_progress = ease_out_quad(1.0) = 1.0
        // interpolated = 0.0 * (1-1) + 100.0 * 1 = 100
        assert_eq!(values2[0], 100, "Values after 2nd update (progress 1)");
        // For steps=2, counter increments when counter (1) < steps.saturating_sub(1) (which is 1).
        // 1 < 1 is false. So counter should not increment. It remains 1.
        assert_eq!(animator.get_interpolation_counter(), 1, "Counter after 2nd update (max for steps=2)");
        assert!(animator.is_new_cycle_start());

        // Update 3: counter will be 1 for calculation, remains 1
        let values3 = animator.update_and_get_current_values();
        // Eased progress for counter 1, steps 2 -> 1.0
        assert_eq!(values3[0], 100, "Values after 3rd update (still at target)");
        assert_eq!(animator.get_interpolation_counter(), 1, "Counter after 3rd update (still max)");
        assert!(animator.is_new_cycle_start());
    }
}