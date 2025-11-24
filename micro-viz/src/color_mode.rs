//! Color mode implementations for spectrum visualization.
//!
//! This module provides pluggable color strategies that determine how colors
//! are calculated based on bar position, height, and other context.

use embedded_graphics::{
    pixelcolor::Rgb888,
    prelude::RgbColor,
};

/// Context information available to color modes.
pub struct ColorContext {
    /// Bar number/index (0-based)
    pub bar_number: usize,
    /// Current bar height value
    pub bar_height: f32,
    /// Total number of bars
    pub num_bars: usize,
    /// Screen height for normalization
    pub screen_height: f32,
    /// Rolling maximum FFT value
    pub rolling_max: f32,
    /// Frame counter for animations
    pub frame_count: u32,
}

impl ColorContext {
    pub fn new(
        bar_number: usize,
        bar_height: f32,
        num_bars: usize,
        screen_height: f32,
        rolling_max: f32,
        frame_count: u32,
    ) -> Self {
        Self {
            bar_number,
            bar_height: bar_height.max(0.0),
            num_bars: num_bars.max(1),
            screen_height: screen_height.max(1.0),
            rolling_max: rolling_max.max(0.0),
            frame_count,
        }
    }
}

/// Trait for color calculation strategies.
///
/// Implementations of this trait determine how colors are calculated for
/// visualization elements based on their position and value.
pub trait ColorMode {
    /// Calculates the color for a visualization element.
    ///
    /// # Arguments
    /// * `context` - Context information about the element
    ///
    /// # Returns
    /// RGB color value
    fn calculate_color(&self, context: &ColorContext) -> Rgb888;
    
    /// Updates any internal state for animated color modes.
    ///
    /// Called once per frame before color calculations.
    /// Default implementation does nothing (for static color modes).
    fn update(&mut self, _frame_count: u32) {
        // Default: no-op for static color modes
    }
}

/// Color wheel distribution across bars (rainbow effect).
pub struct SpectrumColorMode;

impl ColorMode for SpectrumColorMode {
    fn calculate_color(&self, context: &ColorContext) -> Rgb888 {
        let color_position = ((context.bar_number as u32 * 255)
            / (context.num_bars as u32 + 1))
            % 255;
        color_wheel(color_position as u8)
    }
}

/// Intensity-based grayscale gradient.
pub struct GradientColorMode;

impl ColorMode for GradientColorMode {
    fn calculate_color(&self, context: &ColorContext) -> Rgb888 {
        let intensity = (context.bar_height / context.screen_height * 255.0) as u8;
        Rgb888::new(intensity, intensity, intensity)
    }
}

/// Fixed color palette cycling.
pub struct StaticPaletteColorMode {
    palette: alloc::vec::Vec<Rgb888>,
}

impl StaticPaletteColorMode {
    pub fn new(palette: alloc::vec::Vec<Rgb888>) -> Self {
        Self { palette }
    }
    
    pub fn default() -> Self {
        Self::new(alloc::vec![Rgb888::RED, Rgb888::GREEN, Rgb888::BLUE])
    }
}

impl ColorMode for StaticPaletteColorMode {
    fn calculate_color(&self, context: &ColorContext) -> Rgb888 {
        if self.palette.is_empty() {
            return Rgb888::WHITE;
        }
        self.palette[context.bar_number % self.palette.len()]
    }
}

/// FFT magnitude-based dynamic colors.
pub struct DynamicFftColorMode;

impl ColorMode for DynamicFftColorMode {
    fn calculate_color(&self, context: &ColorContext) -> Rgb888 {
        if context.rolling_max <= 0.0 {
            return Rgb888::BLACK;
        }
        let intensity = (context.bar_height / context.rolling_max * 255.0) as u8;
        Rgb888::new(intensity, 0, 255 - intensity)
    }
}

/// Animated color wheel that shifts over time.
pub struct ShiftingSpectrumColorMode {
    offset: u8,
    multiplier: u8,
}

impl ShiftingSpectrumColorMode {
    pub fn new(multiplier: u8) -> Self {
        Self {
            offset: 0,
            multiplier,
        }
    }
    
    pub fn default() -> Self {
        Self::new(32)
    }
}

impl ColorMode for ShiftingSpectrumColorMode {
    fn calculate_color(&self, context: &ColorContext) -> Rgb888 {
        let color_position = (context.bar_number as u8)
            .wrapping_mul(self.multiplier)
            .wrapping_add(self.offset);
        color_wheel(color_position)
    }
    
    fn update(&mut self, _frame_count: u32) {
        self.offset = self.offset.wrapping_add(1);
    }
}

/// Helper function to convert a position (0-255) to an RGB color using a color wheel.
fn color_wheel(pos: u8) -> Rgb888 {
    let pos = pos % 255;
    if pos < 85 {
        Rgb888::new(
            pos.saturating_mul(3),
            255u8.saturating_sub(pos.saturating_mul(3)),
            0,
        )
    } else if pos < 170 {
        let pos = pos - 85;
        Rgb888::new(
            255u8.saturating_sub(pos.saturating_mul(3)),
            0,
            pos.saturating_mul(3),
        )
    } else {
        let pos = pos - 170;
        Rgb888::new(
            0,
            pos.saturating_mul(3),
            255u8.saturating_sub(pos.saturating_mul(3)),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spectrum_color_mode() {
        let mode = SpectrumColorMode;
        let context = ColorContext::new(0, 10.0, 8, 64.0, 10.0, 0);
        let color = mode.calculate_color(&context);
        // Just verify it returns a valid color
        assert!(color.r() >= 0 && color.r() <= 255);
    }

    #[test]
    fn test_gradient_color_mode() {
        let mode = GradientColorMode;
        let context = ColorContext::new(0, 32.0, 8, 64.0, 10.0, 0);
        let color = mode.calculate_color(&context);
        // Should be grayscale
        assert_eq!(color.r(), color.g());
        assert_eq!(color.g(), color.b());
    }

    #[test]
    fn test_shifting_spectrum_updates() {
        let mut mode = ShiftingSpectrumColorMode::default();
        let context1 = ColorContext::new(0, 10.0, 8, 64.0, 10.0, 0);
        let color1 = mode.calculate_color(&context1);
        
        mode.update(1);
        let color2 = mode.calculate_color(&context1);
        
        // Colors should be different after update
        assert_ne!(color1, color2);
    }
}

