#![no_std]
use embedded_graphics::{
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    Drawable,
};
/// Drawing demo for the LED matrix
pub struct DrawingDemo {
    wheel_val: u8,
    current_step: u8,
    step_counter: u32,
}

impl DrawingDemo {
    /// Creates a new DrawingDemo instance
    pub const fn new() -> Self {
        Self {
            wheel_val: 0,
            current_step: 0,
            step_counter: 0,
        }
    }

    /// Generates a color from the HSV color wheel based on position
    fn color_wheel(&self, pos: u8) -> Rgb888 {
        let pos = pos % 255;
        if pos < 85 {
            Rgb888::new(pos * 3, 255 - pos * 3, 0)
        } else if pos < 170 {
            let pos = pos - 85;
            Rgb888::new(255 - pos * 3, 0, pos * 3)
        } else {
            let pos = pos - 170;
            Rgb888::new(0, pos * 3, 255 - pos * 3)
        }
    }

    /// Updates the drawing on the framebuffer
    pub fn update<D>(&mut self, fb: &mut D, heights: [i32; 8]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        // Draw colorful line at the bottom of the display

        for i in 0..8usize {
            let l_max = i * 8;
            let r_max = (i + 1) * 8 - 1;
            let l_point = Point::new(l_max as i32, heights[i]);
            let r_point = Point::new(r_max as i32, heights[i]);
            let color = self.color_wheel((i as u8 * 32).wrapping_add(self.wheel_val));
            let line = Line::new(l_point, r_point);
            line.into_styled(PrimitiveStyle::with_stroke(color, 8))
                .draw(fb)?;
        }

        // Increment animation counters
        self.wheel_val = self.wheel_val.wrapping_add(1);
        self.step_counter = self.step_counter.wrapping_add(1);

        Ok(())
    }
}