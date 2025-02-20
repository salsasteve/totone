use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb888, RgbColor},
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};
use micromath::F32Ext;
use std::{thread, time::Duration};

pub const WIDTH: u32 = 64;
pub const HEIGHT: u32 = 64;
pub const BAR_WIDTH: u32 = 8;
pub const NUM_BARS: u32 = 8;
pub const BAR_SPACING: u32 = 8;
pub const FRAME_DELAY_MS: u64 = 16;

/// Available color modes for visualization
#[derive(Debug, Clone, Copy)]
pub enum ColorMode {
    Binary,
    Rainbow,
    HeatMap,
    BlueScale,
}

/// Configuration for the visualization
#[derive(Debug, Clone)]
pub struct VisualizationConfig {
    pub width: u32,
    pub height: u32,
    pub bar_width: u32,
    pub num_bars: u32,
    pub bar_spacing: u32,
    pub wave_frequency: f32,
    pub color_mode: ColorMode,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            width: WIDTH,
            height: HEIGHT,
            bar_width: BAR_WIDTH,
            num_bars: NUM_BARS,
            bar_spacing: BAR_SPACING,
            wave_frequency: 4.0,
            color_mode: ColorMode::Rainbow,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Bar {
    pub position: Point,
    pub width: u32,
    pub height: u32,
    pub amplitude_ratio: f32, // 0.0 to 1.0, used for coloring
}

/// Color generator based on mode and amplitude
pub struct ColorGenerator {
    mode: ColorMode,
}

impl ColorGenerator {
    pub fn new(mode: ColorMode) -> Self {
        Self { mode }
    }

    pub fn get_color(&self, amplitude_ratio: f32) -> Rgb888 {
        match self.mode {
            ColorMode::Binary => {
                if amplitude_ratio > 0.0 {
                    Rgb888::new(255, 255, 255)
                } else {
                    Rgb888::new(0, 0, 0)
                }
            }
            ColorMode::Rainbow => {
                let hue = amplitude_ratio * 360.0;
                self.hsv_to_rgb(hue, 1.0, 1.0)
            }
            ColorMode::HeatMap => {
                let r = (amplitude_ratio * 255.0) as u8;
                let g = ((1.0 - amplitude_ratio) * 255.0) as u8;
                Rgb888::new(r, g, 0)
            }
            ColorMode::BlueScale => {
                let b = (amplitude_ratio * 255.0) as u8;
                Rgb888::new(0, 0, b)
            }
        }
    }

    fn hsv_to_rgb(&self, h: f32, s: f32, v: f32) -> Rgb888 {
        let c = v * s;
        let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
        let m = v - c;

        let (r, g, b) = match (h as u32) {
            0..=59 => (c, x, 0.0),
            60..=119 => (x, c, 0.0),
            120..=179 => (0.0, c, x),
            180..=239 => (0.0, x, c),
            240..=299 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };

        Rgb888::new(
            ((r + m) * 255.0) as u8,
            ((g + m) * 255.0) as u8,
            ((b + m) * 255.0) as u8,
        )
    }
}

pub fn generate_sine_wave_bars(time: f32, config: &VisualizationConfig) -> Vec<Bar> {
    (0..config.num_bars)
        .map(|i| {
            let x = i as f32 / config.num_bars as f32;
            let amplitude = calculate_amplitude(x, time, config.wave_frequency);
            let amplitude_ratio = amplitude / 48.0; // Normalize to 0.0-1.0
            let bar_x = i * (config.bar_width + config.bar_spacing) + config.bar_spacing;
            let bar_y = config.height - amplitude as u32;
            
            Bar {
                position: Point::new(bar_x as i32, bar_y as i32),
                width: config.bar_width,
                height: amplitude as u32,
                amplitude_ratio,
            }
        })
        .collect()
}

pub fn calculate_amplitude(x: f32, time: f32, frequency: f32) -> f32 {
    let phase = time + x * frequency * 2.0 * core::f32::consts::PI;
    (phase.sin() * 0.5 + 0.5) * 48.0
}

pub fn draw_bars(
    display: &mut SimulatorDisplay<Rgb888>,
    bars: &[Bar],
    color_generator: &ColorGenerator,
) -> Result<(), <SimulatorDisplay<Rgb888> as embedded_graphics::draw_target::DrawTarget>::Error> {
    for bar in bars {
        let color = color_generator.get_color(bar.amplitude_ratio);
        let style = PrimitiveStyleBuilder::new()
            .fill_color(color)
            .build();

        Rectangle::new(
            bar.position,
            Size::new(bar.width, bar.height)
        )
        .into_styled(style)
        .draw(display)?;
    }
    Ok(())
}

pub fn create_window(title: &str) -> Window {
    let output_settings = OutputSettingsBuilder::new()
        .scale(2)
        .build();
    Window::new(title, &output_settings)
}

pub fn run_visualization(config: VisualizationConfig) -> Result<(), std::convert::Infallible> {
    let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(config.width, config.height));
    let color_generator = ColorGenerator::new(config.color_mode);
    let mut window = create_window("FFT Visualization");
    let mut time = 0.0f32;
    
    'running: loop {
        display.clear(Rgb888::new(0, 0, 0))?;

        let bars = generate_sine_wave_bars(time, &config);
        draw_bars(&mut display, &bars, &color_generator)?;

        window.update(&display);
        
        if window.events().any(|e| e == SimulatorEvent::Quit) {
            break 'running Ok(());
        }
        
        time += 0.05;
        thread::sleep(Duration::from_millis(FRAME_DELAY_MS));
    }
}

fn main() -> Result<(), std::convert::Infallible> {
    // Example of running with different color modes
    let mut config = VisualizationConfig::default();
    
    // Change this line to try different color modes:
    config.color_mode = ColorMode::Rainbow;
    // Available modes: ColorMode::Binary, ColorMode::Rainbow, ColorMode::HeatMap, ColorMode::BlueScale
    
    run_visualization(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_color_generator() {
        let generator = ColorGenerator::new(ColorMode::Binary);
        let white = generator.get_color(1.0);
        let black = generator.get_color(0.0);
        assert_eq!(white, Rgb888::new(255, 255, 255));
        assert_eq!(black, Rgb888::new(0, 0, 0));
    }

    #[test]
    fn test_heat_map_colors() {
        let generator = ColorGenerator::new(ColorMode::HeatMap);
        let hot = generator.get_color(1.0);
        let cold = generator.get_color(0.0);
        assert_eq!(hot, Rgb888::new(255, 0, 0));
        assert_eq!(cold, Rgb888::new(0, 255, 0));
    }

    #[test]
    fn test_blue_scale_colors() {
        let generator = ColorGenerator::new(ColorMode::BlueScale);
        let full = generator.get_color(1.0);
        let none = generator.get_color(0.0);
        assert_eq!(full, Rgb888::new(0, 0, 255));
        assert_eq!(none, Rgb888::new(0, 0, 0));
    }

    #[test]
    fn test_bar_generation_with_amplitude_ratio() {
        let config = VisualizationConfig::default();
        let bars = generate_sine_wave_bars(0.0, &config);
        
        assert!(bars.iter().all(|bar| bar.amplitude_ratio >= 0.0 && bar.amplitude_ratio <= 1.0));
    }
}