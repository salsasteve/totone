use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};
use micro_viz::DrawingDemo;
use micromath::F32Ext;
use std::{thread, time::Duration};

// Constants for visualization parameters
pub const WIDTH: u32 = 64;
pub const HEIGHT: u32 = 64;
pub const FRAME_DELAY_MS: u64 = 16;

const WHITE: Rgb888 = Rgb888::new(255, 255, 255);
const BLACK: Rgb888 = Rgb888::new(0, 0, 0);
const RED: Rgb888 = Rgb888::new(255, 0, 0);
const GREEN: Rgb888 = Rgb888::new(0, 255, 0);
const BLUE: Rgb888 = Rgb888::new(0, 0, 255);
const YELLOW: Rgb888 = Rgb888::new(255, 255, 0);
const VIOLET: Rgb888 = Rgb888::new(255, 0, 255);

pub fn calculate_amplitude(x: f32, time: f32, frequency: f32) -> f32 {
    let phase = time + x * frequency * 2.0 * core::f32::consts::PI;
    (phase.sin() * 0.5 + 0.5) * 48.0
}

fn main() -> Result<(), std::convert::Infallible> {
    // Create a display with the specified width and height
    let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));
    
    // Create a window with the specified title and size
    let mut window = Window::new(
        "MicroViz Simulator",
        &OutputSettingsBuilder::new().theme(BinaryColorTheme::Default).build(),
    );
    
    // Create a drawing demo instance
    let mut demo = DrawingDemo::new();
    
    // Initialize time variable for animation
    let mut time: f32 = 0.0;
    
    // Main loop
    loop {
        // Clear the display
        display.clear(BinaryColor::Off.into())?;
        
        // Calculate the heights for each bar based on the amplitude function
        let heights: [i32; 8] = [
            calculate_amplitude(0.0, time, 1.0) as i32,
            calculate_amplitude(1.0, time, 1.0) as i32,
            calculate_amplitude(2.0, time, 1.0) as i32,
            calculate_amplitude(3.0, time, 1.0) as i32,
            calculate_amplitude(4.0, time, 1.0) as i32,
            calculate_amplitude(5.0, time, 1.0) as i32,
            calculate_amplitude(6.0, time, 1.0) as i32,
            calculate_amplitude(7.0, time, 1.0) as i32,
        ];
        
        // Update the drawing demo with the calculated heights
        demo.update(&mut display, heights)?;
        
        // Draw a rectangle around the display area
        Rectangle::with_corners(Point::zero(), Point::new(WIDTH as i32 - 1, HEIGHT as i32 - 1))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .stroke_color(BLACK)
                    .stroke_width(2)
                    .build(),
            )
            .draw(&mut display)?;
        
        // Draw the display on the window
        window.update(&display);
        
        // Increment time for animation
        time += 0.05;
        
        // Wait for a frame delay
        thread::sleep(Duration::from_millis(FRAME_DELAY_MS));
        
        // Handle window events
        if let Some(event) = window.events().next() {
            if let SimulatorEvent::Quit = event {
                break;
            }
        }
    }
    
    Ok(())
}