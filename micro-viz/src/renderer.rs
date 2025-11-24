//! Rendering strategies for drawing visualization elements.
//!
//! This module provides low-level rendering functions for drawing different
//! shapes and elements used in visualizations.

use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
};

/// Renders a vertical bar on the display.
///
/// # Arguments
/// * `target` - The drawing target (framebuffer)
/// * `x` - X coordinate (left edge of bar)
/// * `y_top` - Y coordinate of the top of the bar
/// * `y_bottom` - Y coordinate of the bottom of the bar
/// * `width` - Width of the bar in pixels
/// * `color` - Color of the bar
pub fn render_bar<D: DrawTarget<Color = Rgb888>>(
    target: &mut D,
    x: i32,
    y_top: i32,
    y_bottom: i32,
    width: u16,
    color: Rgb888,
) -> Result<(), D::Error> {
    // Draw the bar as vertical lines
    for offset in 0..width {
        let x_coord = x + offset as i32;
        Line::new(
            Point::new(x_coord, y_top),
            Point::new(x_coord, y_bottom),
        )
        .into_styled(PrimitiveStyle::with_stroke(color, 1))
        .draw(target)?;
    }
    Ok(())
}

/// Renders a filled rectangle (for fat bars).
///
/// # Arguments
/// * `target` - The drawing target (framebuffer)
/// * `x` - X coordinate (left edge)
/// * `y` - Y coordinate (top edge)
/// * `width` - Width of the rectangle
/// * `height` - Height of the rectangle
/// * `color` - Fill color
pub fn render_filled_rect<D: DrawTarget<Color = Rgb888>>(
    target: &mut D,
    x: i32,
    y: i32,
    width: u32,
    height: u32,
    color: Rgb888,
) -> Result<(), D::Error> {
    Rectangle::new(
        Point::new(x, y),
        embedded_graphics::geometry::Size::new(width, height),
    )
    .into_styled(PrimitiveStyle::with_fill(color))
    .draw(target)?;
    Ok(())
}

/// Renders a line segment.
///
/// # Arguments
/// * `target` - The drawing target (framebuffer)
/// * `start` - Start point
/// * `end` - End point
/// * `color` - Line color
pub fn render_line<D: DrawTarget<Color = Rgb888>>(
    target: &mut D,
    start: Point,
    end: Point,
    color: Rgb888,
) -> Result<(), D::Error> {
    Line::new(start, end)
        .into_styled(PrimitiveStyle::with_stroke(color, 1))
        .draw(target)?;
    Ok(())
}

/// Renders a smooth curve (waveform) using connected line segments.
///
/// # Arguments
/// * `target` - The drawing target (framebuffer)
/// * `points` - Iterator of points to connect
/// * `color` - Line color
pub fn render_waveform<D: DrawTarget<Color = Rgb888>>(
    target: &mut D,
    mut points: impl Iterator<Item = Point>,
    color: Rgb888,
) -> Result<(), D::Error> {
    let mut prev_point = if let Some(first) = points.next() {
        first
    } else {
        return Ok(());
    };

    for point in points {
        render_line(target, prev_point, point, color)?;
        prev_point = point;
    }
    Ok(())
}

/// Helper to clamp coordinates to screen bounds.
pub fn clamp_coord(coord: i32, min: i32, max: i32) -> i32 {
    coord.max(min).min(max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp_coord() {
        assert_eq!(clamp_coord(5, 0, 10), 5);
        assert_eq!(clamp_coord(-1, 0, 10), 0);
        assert_eq!(clamp_coord(15, 0, 10), 10);
    }
}


