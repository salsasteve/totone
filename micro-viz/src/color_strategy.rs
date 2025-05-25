use embedded_graphics::{
    pixelcolor::Rgb888,
    prelude::*,
};
pub struct ColorContext {
    pub element_index: usize,
    pub num_elements: usize,
    pub element_height: f32,   
    pub max_element_height: f32, 
}

pub trait ColorStrategy {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888;
}

fn map_position_to_rgb_on_wheel(pos: u8) -> Rgb888 {
    let pos = pos % 255; 
    if pos < 85 {
        Rgb888::new(
            pos.saturating_mul(3),
            255u8.saturating_sub(pos.saturating_mul(3)),
            0,
        )
    } else if pos < 170 {
        let pos = pos.saturating_sub(85);
        Rgb888::new(
            255u8.saturating_sub(pos.saturating_mul(3)),
            0,
            pos.saturating_mul(3),
        )
    } else {
        let pos = pos.saturating_sub(170);
        Rgb888::new(
            0,
            pos.saturating_mul(3),
            255u8.saturating_sub(pos.saturating_mul(3)),
        )
    }
}

pub struct SpectrumColor;
impl ColorStrategy for SpectrumColor {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888 {
        let color_position = ((context.element_index as u32 * 255
            / (context.num_elements as u32))
            % 255) as u8;
        map_position_to_rgb_on_wheel(color_position)
    }
}

pub struct GradientColor;
impl ColorStrategy for GradientColor {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888 {
        let intensity = (context.element_height as f32 / context.max_element_height as f32 * 255.0)
            .clamp(0.0, 255.0) as u8;
        Rgb888::new(intensity, intensity, intensity)
    }
}

pub struct StaticPaletteColor {
    palette: [Rgb888; 3],
}
impl StaticPaletteColor {
    pub fn new() -> Self {
        Self {
            palette: [Rgb888::RED, Rgb888::GREEN, Rgb888::BLUE],
        }
    }
}
impl ColorStrategy for StaticPaletteColor {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888 {
        if self.palette.is_empty() {
            return Rgb888::BLACK;
        }
        self.palette[context.element_index % self.palette.len()]
    }
}

pub struct DynamicFFTColor;
impl ColorStrategy for DynamicFFTColor {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888 {
        let intensity = (context.element_height as f32 / context.max_element_height as f32 * 255.0)
            .clamp(0.0, 255.0) as u8;
        Rgb888::new(intensity, 0, 255u8.saturating_sub(intensity))
    }
}

pub struct ShiftingSpectrumColor {
    color_wheel_multiplier: u8,
    wheel_val: u8,
}
impl ShiftingSpectrumColor {
    pub fn new(num_bars: usize) -> Self {
        Self {
            color_wheel_multiplier: if num_bars > 0 { 255 / num_bars as u8 } else { 1 },
            wheel_val: 0,
        }
    }
}
impl ColorStrategy for ShiftingSpectrumColor {
    fn get_color(&mut self, context: &ColorContext) -> Rgb888 {
        map_position_to_rgb_on_wheel(
            (context.element_index as u8)
                .wrapping_mul(self.color_wheel_multiplier)
                .wrapping_add(self.wheel_val),
        )
    }
}