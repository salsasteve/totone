use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};
use micro_viz::DrawingDemo;
use std::{thread, time::Duration};

// Constants for visualization parameters
pub const WIDTH: u32 = 64;
pub const HEIGHT: u32 = 64;
pub const FRAME_DELAY_MS: u64 = 16;

const BLACK: Rgb888 = Rgb888::new(0, 0, 0);

pub fn simulate_fft(time: f32) -> [f32; 8] {
    // Define our 8 core frequencies on a logarithmic scale (20Hz to 20kHz)
    let frequencies = [
        20.0,    // Low bass
        63.0,    // Bass
        200.0,   // Low-mid
        630.0,   // Mid
        2000.0,  // Upper-mid
        6300.0,  // Presence
        12000.0, // Brilliance
        20000.0, // High end
    ];

    // Array to store magnitudes for each frequency bin
    let mut magnitudes = [0.0; 8];

    // Base amplitude for each frequency (can be adjusted to emphasize certain ranges)
    let base_amplitudes = [
        0.9, // Bass frequencies are often stronger
        0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, // High frequencies often have less energy
    ];

    // Generate a different phase for each frequency bin
    for i in 0..8 {
        // Create a sine wave for this frequency
        let freq = frequencies[i];
        let phase = time * freq * 0.01; // Scale time to make animation visible

        // Calculate magnitude with some time-based modulation
        // This creates a more interesting visualization
        let base_mag = (phase.sin() * 0.5 + 0.5) * base_amplitudes[i];

        // Add some cross-talk between adjacent frequency bins
        let neighbor_effect = if i > 0 {
            (time * frequencies[i - 1] * 0.015).sin() * 0.2
        } else {
            0.0
        };

        // Ensure magnitudes stay in proper range
        magnitudes[i] = (base_mag + neighbor_effect).max(0.01).min(1.0) * 63.0;
    }

    magnitudes
}

fn main() -> Result<(), std::convert::Infallible> {
    // Create a display with the specified width and height
    let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(WIDTH, HEIGHT));

    // Create a window with the specified title and size
    let mut window = Window::new(
        "MicroViz Simulator",
        &OutputSettingsBuilder::new()
            .theme(BinaryColorTheme::Default)
            .build(),
    );

    // Create a drawing demo instance
    let mut demo = DrawingDemo::new(WIDTH as u16, HEIGHT as u16);

    // Initialize time variable for animation
    let mut time: f32 = 0.0;

    // Main loop
    loop {
        // Clear the display
        display.clear(BinaryColor::Off.into())?;

        let fft_magnitudes = simulate_fft(time);
        let heights: [i32; 8] = [
            fft_magnitudes[0] as i32,
            fft_magnitudes[1] as i32,
            fft_magnitudes[2] as i32,
            fft_magnitudes[3] as i32,
            fft_magnitudes[4] as i32,
            fft_magnitudes[5] as i32,
            fft_magnitudes[6] as i32,
            fft_magnitudes[7] as i32,
        ];

        // Update the drawing demo with the calculated heights
        demo.update(&mut display, heights)?;

        // Draw a rectangle around the display area
        Rectangle::with_corners(
            Point::zero(),
            Point::new(WIDTH as i32 - 1, HEIGHT as i32 - 1),
        )
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
