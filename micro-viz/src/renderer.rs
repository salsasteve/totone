use core::cmp;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};

#[cfg(feature = "logging")]
use defmt::info;
#[cfg(feature = "logging")]
use defmt_rtt as _;

const BAR_SPACING: u32 = 1; 
#[allow(unused_imports)]
use micromath::F32Ext;

use crate::color_strategy::{ColorStrategy, ColorContext};

pub enum ColorMode {
    Spectrum,
    Gradient,
    StaticPalette,
    DynamicFFT,
    ShiftingSpectrum,
}

pub trait Renderer {
    fn draw<D: DrawTarget<Color = Rgb888>>(
        &mut self,
        target: &mut D,
        frame: &[u16],
    ) -> Result<(), D::Error>;
}

pub struct BarGraphRenderer<CS: ColorStrategy> {
    screen_width: u16,
    screen_height: u16,
    bar_width: u16,
    num_bars: usize,
    color_strategy: CS,
}

impl<CS: ColorStrategy> Renderer for BarGraphRenderer<CS> {
    fn draw<D>(&mut self, fb: &mut D, current_values: &[u16]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        for i in 0..self.num_bars {
            // Get current height from the animator's display values
            let current_height = current_values[i];

            let context = ColorContext {
                element_index: i,
                num_elements: self.num_bars, // Total configured bars for context
                element_height: current_height as f32,
                max_element_height: self.screen_height as f32,
            };

            // The y_line_coord should be screen_height - height, assuming 0,0 is top-left
            // And bars grow upwards from the bottom.
            // If y_line_coord was meant to be the height itself (from top):
            let y_pos = self
                .screen_height
                .saturating_sub(current_height)
                .saturating_sub(1);
            let y_line_coord = cmp::max(0, y_pos) as i32;

            let bar_start_x = (i as u32 * (self.bar_width as u32 + BAR_SPACING)) as i32;
            let bar_end_x = bar_start_x + (self.bar_width as i32) - 1;

            let x_left = cmp::max(0, cmp::min((self.screen_width - 1) as i32, bar_start_x));
            let x_right = cmp::max(x_left, cmp::min((self.screen_width - 1) as i32, bar_end_x));

            if x_left > x_right {
                continue;
            }

            let color = self.color_strategy.get_color(&context);

            // Draw line from bottom of screen up to current_height_val
            // Assuming (x, screen_height-1) is bottom, (x, screen_height - 1 - height) is top of bar
            let y_bottom = (self.screen_height - 1) as i32;
            let y_top = (self.screen_height.saturating_sub(current_height).max(0)) as i32;

            for bar_x_offset in 0..self.bar_width {
                let x_coord = bar_start_x + bar_x_offset as i32;
                if x_coord < 0 || x_coord >= self.screen_width as i32 {
                    continue;
                }

                Line::new(Point::new(x_coord, y_top), Point::new(x_coord, y_bottom))
                    .into_styled(PrimitiveStyle::with_stroke(color, 1))
                    .draw(fb)?;
            }
        }
        Ok(())
    }
}

impl<CS: ColorStrategy> BarGraphRenderer<CS> {
    pub fn new(
        screen_width: u16,
        screen_height: u16,
        bar_width: u16,
        num_bars: usize,
        color_strategy: CS,
    ) -> Self {
        let bar_width = cmp::max(1, bar_width);
        Self {
            screen_width,
            screen_height,
            bar_width,
            num_bars,
            color_strategy,
        }
    }
}
