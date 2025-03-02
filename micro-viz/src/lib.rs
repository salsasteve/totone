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
    interpolation_counter: u32,
    bar_width: u8,
    screen_width: u16,
    screen_height: u16,
    stroke_width: u8,
    color_wheel_multiplier: u8,
    previous_heights: [i32; NUM_BARS as usize],
    target_heights: [i32; NUM_BARS as usize],
    current_heights: [f32; NUM_BARS as usize],
    interpolation_steps: u32,
}

impl DrawingDemo {
    /// Creates a new DrawingDemo instance
    pub const fn new(screen_width: u16, screen_height: u16, stroke_width: u8, interpolation_steps:u32) -> Self {
        Self {
            wheel_val: 0,
            current_step: 0,
            step_counter: 0,
            interpolation_counter: 0,
            bar_width: 8,
            stroke_width,
            screen_width,
            screen_height,
            color_wheel_multiplier: 32,
            previous_heights: [0; NUM_BARS as usize],
            target_heights: [0; NUM_BARS as usize],
            current_heights: [0.0; NUM_BARS as usize],
            interpolation_steps,
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
    
    /// Interpolate between two colors for smoother transitions
    fn interpolate_color(&self, color1: Rgb888, color2: Rgb888, t: f32) -> Rgb888 {
        let r = (color1.r() as f32 * (1.0 - t) + color2.r() as f32 * t) as u8;
        let g = (color1.g() as f32 * (1.0 - t) + color2.g() as f32 * t) as u8;
        let b = (color1.b() as f32 * (1.0 - t) + color2.b() as f32 * t) as u8;
        Rgb888::new(r, g, b)
    }
    
    /// Apply easing function for smoother motion
    fn ease_out_quad(&self, t: f32) -> f32 {
        1.0 - (1.0 - t) * (1.0 - t)
    }

    /// Updates the drawing on the framebuffer
    pub fn update<D>(&mut self, fb: &mut D, heights: [i32; 8]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        // Handle new target heights
        if self.interpolation_counter == 0 {
            self.previous_heights = self.target_heights;
            self.target_heights = heights;
            
            for i in 0..NUM_BARS as usize {

                self.current_heights[i] = self.previous_heights[i] as f32;
            }
        }
        
        // Calculate interpolation progress (0.0 to 1.0)
        let progress = self.interpolation_counter as f32 / self.interpolation_steps as f32;
        let eased_progress = self.ease_out_quad(progress);
        
        // Interpolate heights
        for i in 0..NUM_BARS as usize {
            self.current_heights[i] = self.previous_heights[i] as f32 * (1.0 - eased_progress) + 
                                     self.target_heights[i] as f32 * eased_progress;
        }

        // Draw colorful bars
        for i in 0..8usize {
            let height = self.current_heights[i] as i32;
            let l_max = i * self.bar_width as usize;
            let r_max = (i + 1) * self.bar_width as usize - 1;
            let l_point = Point::new(l_max as i32, height);
            let r_point = Point::new(r_max as i32, height);
            
            // Animate color
            let color = self.color_wheel((i as u8 * self.color_wheel_multiplier).wrapping_add(self.wheel_val));
            
            let line = Line::new(l_point, r_point);
            line.into_styled(PrimitiveStyle::with_stroke(color, self.stroke_width as u32))
                .draw(fb)?;
        }

        // Animate colors
        // self.wheel_val = self.wheel_val.wrapping_add(1);
        
        // Increment animation counters
        self.step_counter = self.step_counter.wrapping_add(1);
        self.interpolation_counter = (self.interpolation_counter + 1) % self.interpolation_steps;
        
        Ok(())
    }
}