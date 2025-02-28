#![no_std]
use embedded_graphics::{
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    Drawable,
};

pub const BAR_WIDTH: u32 = 8;
pub const NUM_BARS: u32 = 8;
pub const BAR_SPACING: u32 = 0;
pub const FRAME_DELAY_MS: u64 = 16;

// Common colors
const WHITE: Rgb888 = Rgb888::new(255, 255, 255);
const BLACK: Rgb888 = Rgb888::new(0, 0, 0);
const RED: Rgb888 = Rgb888::new(255, 0, 0);
const GREEN: Rgb888 = Rgb888::new(0, 255, 0);
const BLUE: Rgb888 = Rgb888::new(0, 0, 255);
const YELLOW: Rgb888 = Rgb888::new(255, 255, 0);
const VIOLET: Rgb888 = Rgb888::new(255, 0, 255);

/// Drawing demo for the LED matrix
pub struct DrawingDemo {
    wheel_val: u8,
    current_step: u8,
    step_counter: u32,
    bar_width: u8,
    screen_width: u16,
    screen_height: u16,
    stroke_width: u8,
    color_wheel_multiplier: u8,
}

impl DrawingDemo {
    /// Creates a new DrawingDemo instance
    pub const fn new(screen_width:u16, screen_height:u16, stroke_width:u8) -> Self {
        

        Self {
            wheel_val: 0,
            current_step: 0,
            step_counter: 0,
            bar_width: 8,
            stroke_width,
            screen_width,
            screen_height,
            color_wheel_multiplier: 32,
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
            let reverse_height = self.screen_height as i32 - heights[i];
            let l_max = i * self.bar_width as usize;
            let r_max = (i + 1) * self.bar_width as usize - 1;
            let l_point = Point::new(l_max as i32, reverse_height);
            let r_point = Point::new(r_max as i32, reverse_height);
            let color = self.color_wheel((i as u8 * self.color_wheel_multiplier).wrapping_add(self.wheel_val));
            let line = Line::new(l_point, r_point);
            line.into_styled(PrimitiveStyle::with_stroke(color, self.stroke_width as u32))
                .draw(fb)?;
        }

        // Increment animation counters
        // self.wheel_val = self.wheel_val.wrapping_add(1);
        self.step_counter = self.step_counter.wrapping_add(1);

        Ok(())
    }
}