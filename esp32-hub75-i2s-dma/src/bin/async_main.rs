#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use core::mem::MaybeUninit;
#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_executor::task;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embassy_time::{Duration, Timer};
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::I2s0DmaChannel;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::Pin;
use esp_hal::i2s::parallel::AnyI2s;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::DmaFrameBuffer;
use esp_hub75::i2s_parallel::Hub75;
use esp_hub75::Color;
use esp_hub75::Hub75Pins;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

const GAMMA: [u8; 256] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7,
    7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11,
    11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15,
    15, 15, 15, 15, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19,
    20, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 24, 25, 25,
    25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 29, 29, 29, 29, 30, 30, 30, 30, 31, 31,
    31, 31, 32, 32,
];

type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub struct Hub75Peripherals {
    pub i2s: AnyI2s,
    pub dma_channel: I2s0DmaChannel,
    pub red1: AnyPin,
    pub grn1: AnyPin,
    pub blu1: AnyPin,
    pub red2: AnyPin,
    pub grn2: AnyPin,
    pub blu2: AnyPin,
    pub addr0: AnyPin,
    pub addr1: AnyPin,
    pub addr2: AnyPin,
    pub addr3: AnyPin,
    pub addr4: AnyPin,
    pub blank: AnyPin,
    pub clock: AnyPin,
    pub latch: AnyPin,
}

fn gamma(color: Rgb888) -> Rgb888 {
    let r = GAMMA[color.r() as usize].checked_mul(8).unwrap_or(255);
    let g = GAMMA[color.g() as usize].checked_mul(8).unwrap_or(255);
    let b = GAMMA[color.b() as usize].checked_mul(8).unwrap_or(255);
    Rgb888::new(r, g, b)
}

#[task]
async fn display_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
) {
    info!("display_task: starting!");

    let mut demo = DrawingDemo::new();
    let mut count = 0u32;
    let mut start = Instant::now();

    loop {
        fb.clear();

        // Update drawing demo
        demo.update(fb).unwrap();

        tx.signal(fb);
        fb = rx.wait().await;

        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

// #[task]
// async fn display_task(
//     rx: &'static FrameBufferExchange,
//     tx: &'static FrameBufferExchange,
//     mut fb: &'static mut FBType,
// ) {
//     info!("display_task: starting!");

//     let mut plasma = PlasmaEffect::new();
//     let mut count = 0u32;
//     let mut start = Instant::now();

//     loop {
//         fb.clear();

//         // Update plasma effect
//         plasma.update(fb).unwrap();

//         tx.signal(fb);
//         fb = rx.wait().await;

//         count += 1;
//         const FPS_INTERVAL: Duration = Duration::from_secs(1);
//         if start.elapsed() > FPS_INTERVAL {
//             RENDER_RATE.store(count, Ordering::Relaxed);
//             count = 0;
//             start = Instant::now();
//         }
//     }
// }

// #[task]
// async fn display_task(
//     rx: &'static FrameBufferExchange,
//     tx: &'static FrameBufferExchange,
//     mut fb: &'static mut FBType,
// ) {
//     info!("display_task: starting!");
//     let fps_style = MonoTextStyleBuilder::new()
//         .font(&FONT_5X7)
//         .text_color(Color::YELLOW)
//         .background_color(Color::BLACK)
//         .build();
//     let mut count = 0u32;

//     let mut start = Instant::now();

//     loop {
//         fb.clear();

//         const STEP: u8 = (256 / COLS) as u8 - 1;
//         for x in 0..COLS {
//             let brightness = (x as u8) * STEP + 64;
//             for y in 0..8 {
//                 fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
//             }
//             for y in 8..16 {
//                 fb.set_pixel(Point::new(x as i32, y), Color::new(0, brightness, 0));
//             }
//             for y in 16..24 {
//                 fb.set_pixel(Point::new(x as i32, y), Color::new(0, 0, brightness));
//             }
//         }

//         let mut buffer: String<64> = String::new();

//         fmt::write(
//             &mut buffer,
//             format_args!("Refresh: {:4}", REFRESH_RATE.load(Ordering::Relaxed)),
//         )
//         .unwrap();
//         Text::with_alignment(
//             buffer.as_str(),
//             Point::new(0, 63),
//             fps_style,
//             Alignment::Left,
//         )
//         .draw(fb)
//         .unwrap();

//         buffer.clear();
//         fmt::write(
//             &mut buffer,
//             format_args!("Render: {:5}", RENDER_RATE.load(Ordering::Relaxed)),
//         )
//         .unwrap();

//         Text::with_alignment(
//             buffer.as_str(),
//             Point::new(0, 63 - 8),
//             fps_style,
//             Alignment::Left,
//         )
//         .draw(fb)
//         .unwrap();

//         buffer.clear();
//         fmt::write(
//             &mut buffer,
//             format_args!("Simple: {:5}", SIMPLE_COUNTER.load(Ordering::Relaxed)),
//         )
//         .unwrap();
//         Text::with_alignment(
//             buffer.as_str(),
//             Point::new(0, 63 - 16),
//             fps_style,
//             Alignment::Left,
//         )
//         .draw(fb)
//         .unwrap();

//         tx.signal(fb);

//         fb = rx.wait().await;

//         count += 1;
//         const FPS_INTERVAL: Duration = Duration::from_secs(1);
//         if start.elapsed() > FPS_INTERVAL {
//             RENDER_RATE.store(count, Ordering::Relaxed);
//             count = 0;
//             start = Instant::now();
//         }
//     }
// }

#[task]
async fn hub75_task(
    peripherals: Hub75Peripherals,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    info!("hub75_task: starting!");

    let mut hub75 = create_hub75(peripherals);

    let mut count = 0u32;
    let mut start = Instant::now();

    let mut fb = fb;

    loop {
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        let mut xfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        xfer.wait_for_done()
            .await
            .expect("rendering wait_for_done failed!");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");

        count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Main starting!");
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    info!("ROWS: {}", ROWS);
    info!("COLS: {}", COLS);
    info!("BITS: {}", BITS);
    info!("FRAME_COUNT: {}", FRAME_COUNT);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    info!("init embassy");
    esp_hal_embassy::init(timg0.timer0);

    info!("init framebuffer exchange");
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    info!("init framebuffers");
    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());
    fb0.clear();
    fb1.clear();

    let hub75_peripherals = Hub75Peripherals {
        i2s: peripherals.I2S0.into(),
        dma_channel: peripherals.DMA_I2S0,
        red1: peripherals.GPIO25.degrade(),
        grn1: peripherals.GPIO26.degrade(),
        blu1: peripherals.GPIO27.degrade(),
        red2: peripherals.GPIO14.degrade(),
        grn2: peripherals.GPIO12.degrade(),
        blu2: peripherals.GPIO13.degrade(),
        addr0: peripherals.GPIO23.degrade(),
        addr1: peripherals.GPIO19.degrade(),
        addr2: peripherals.GPIO5.degrade(),
        addr3: peripherals.GPIO17.degrade(),
        addr4: peripherals.GPIO32.degrade(),
        blank: peripherals.GPIO15.degrade(),
        clock: peripherals.GPIO16.degrade(),
        latch: peripherals.GPIO4.degrade(),
    };

    let hp_executor = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(software_interrupt)
    );
    let high_pri_spawner = hp_executor.start(Priority::Priority3);

    high_pri_spawner
        .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
        .ok();
    spawner.spawn(display_task(&TX, &RX, fb0)).ok();

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

fn create_hub75(peripherals: Hub75Peripherals) -> Hub75<'static, esp_hal::Async> {
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, size_of::<FBType>());
    let pins = Hub75Pins {
        red1: peripherals.red1,
        grn1: peripherals.grn1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        grn2: peripherals.grn2,
        blu2: peripherals.blu2,
        addr0: peripherals.addr0,
        addr1: peripherals.addr1,
        addr2: peripherals.addr2,
        addr3: peripherals.addr3,
        addr4: peripherals.addr4,
        blank: peripherals.blank,
        clock: peripherals.clock,
        latch: peripherals.latch,
    };

    Hub75::new(
        peripherals.i2s,
        pins,
        peripherals.dma_channel,
        tx_descriptors,
        Rate::from_mhz(19),
    )
    .expect("failed to create Hub75!")
    .into_async()
}

use core::f32::consts::PI;
use embedded_graphics::prelude::*;
use micromath::F32Ext;

pub struct PlasmaEffect {
    time_counter: u16,
    cycles: u16,
    current_palette: usize,
}

impl PlasmaEffect {
    pub fn new() -> Self {
        Self {
            time_counter: 0,
            cycles: 0,
            current_palette: 0,
        }
    }

    fn sin8(x: u8) -> u8 {
        let angle = (x as f32 / 255.0) * 2.0 * PI;
        ((angle.sin() + 1.0) * 127.5) as u8
    }

    fn sin16(x: i16) -> i16 {
        let angle = (x as f32 / 32768.0) * PI;
        (angle.sin() * 32767.0) as i16
    }

    fn cos16(x: i16) -> i16 {
        let angle = (x as f32 / 32768.0) * PI;
        (angle.cos() * 32767.0) as i16
    }

    fn cos8(x: u8) -> u8 {
        let angle = (x as f32 / 255.0) * 2.0 * PI;
        ((angle.cos() + 1.0) * 127.5) as u8
    }

    fn color_from_palette(&self, index: u8) -> Rgb888 {
        // Simplified palette system - you can extend this
        match self.current_palette {
            0 => self.heat_color(index),
            1 => self.lava_color(index),
            2 => self.rainbow_color(index),
            3 => self.cloud_color(index),
            _ => self.rainbow_color(index),
        }
    }

    fn heat_color(&self, index: u8) -> Rgb888 {
        let x = index as f32 / 255.0;
        let r = (x * 255.0) as u8;
        let g = ((x * x) * 255.0) as u8;
        let b = ((x * x * x) * 255.0) as u8;
        Rgb888::new(r, g, b)
    }

    fn lava_color(&self, index: u8) -> Rgb888 {
        let x = index as f32 / 255.0;
        let r = (x * 255.0) as u8;
        let g = ((x * x * 0.5) * 255.0) as u8;
        let b = ((x * x * x * 0.2) * 255.0) as u8;
        Rgb888::new(r, g, b)
    }

    fn rainbow_color(&self, index: u8) -> Rgb888 {
        let hue = index as f32 / 255.0;
        let i = (hue * 6.0) as u8;
        let f = hue * 6.0 - i as f32;
        let p = 0.0;
        let q = 1.0 - f;
        let t = f;

        let (r, g, b) = match i % 6 {
            0 => (1.0, t, p),
            1 => (q, 1.0, p),
            2 => (p, 1.0, t),
            3 => (p, q, 1.0),
            4 => (t, p, 1.0),
            5 => (1.0, p, q),
            _ => (0.0, 0.0, 0.0),
        };

        Rgb888::new((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
    }

    fn cloud_color(&self, index: u8) -> Rgb888 {
        let x = index as f32 / 255.0;
        let value = (x * 255.0) as u8;
        Rgb888::new(value, value, value)
    }

    // pub fn update<D>(&mut self, fb: &mut D) -> Result<(), D::Error>
    // where
    //     D: DrawTarget<Color = Rgb888>,
    // {
    //     const MAX_PIXELS: usize = 1024; // Adjust this size as needed
    //     let mut pixels: [Option<Pixel<Rgb888>>; MAX_PIXELS] = [None; MAX_PIXELS];
    //     let mut pixel_count = 0;

    //     for x in 0..fb.bounding_box().size.width as i32 {
    //         for y in 0..fb.bounding_box().size.height as i32 {
    //             if pixel_count >= MAX_PIXELS {
    //                 break;
    //             }

    //             let mut v = 128i16;
    //             let wibble = Self::sin8(self.time_counter as u8) as i16;

    //             v += Self::sin16(x as i16 * wibble * 3 + self.time_counter as i16);
    //             v += Self::cos16(y as i16 * (128 - wibble) + self.time_counter as i16);
    //             v += Self::sin16(
    //                 y as i16 * x as i16 * Self::cos8(self.time_counter as u8) as i16 / 8,
    //             );

    //             let color = self.color_from_palette((v >> 8) as u8);
    //             pixels[pixel_count] = Some(Pixel(Point::new(x, y), color));
    //             pixel_count += 1;
    //         }
    //     }

    //     for pixel in pixels.iter().flatten() {
    //         fb.draw_iter(core::iter::once(*pixel))?;
    //     }

    //     self.time_counter = self.time_counter.wrapping_add(1);
    //     self.cycles = self.cycles.wrapping_add(1);

    //     if self.cycles >= 1024 {
    //         self.time_counter = 0;
    //         self.cycles = 0;
    //         // Cycle through palettes
    //         self.current_palette = (self.current_palette + 1) % 4;
    //     }

    //     Ok(())
    // }

    pub fn update<D>(&mut self, fb: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        for x in 0..COLS as i32 {
            for y in 0..ROWS as i32 {
                let mut v = 128i16;
                let wibble = Self::sin8(self.time_counter as u8);

                // Convert x and y to i16 safely
                let x_i16 = x.try_into().unwrap_or(0);
                let y_i16 = y.try_into().unwrap_or(0);

                v += Self::sin16(x_i16 * (wibble as i16) * 3 + self.time_counter as i16);
                v += Self::cos16(y_i16 * (128 - wibble as i16) + self.time_counter as i16);
                v += Self::sin16(y_i16 * x_i16 * Self::cos8(self.time_counter as u8) as i16 / 8);

                let color = self.color_from_palette((v >> 8) as u8);

                // Use draw_iter with a single pixel
                fb.draw_iter(core::iter::once(Pixel(Point::new(x, y), color)))?;
            }
        }

        self.time_counter = self.time_counter.wrapping_add(1);
        self.cycles = self.cycles.wrapping_add(1);

        if self.cycles >= 1024 {
            self.time_counter = 0;
            self.cycles = 0;
            self.current_palette = (self.current_palette + 1) % 4;
        }

        Ok(())
    }
}

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
};

pub struct DrawingDemo {
    wheel_val: u8,
    current_step: u8,
    step_counter: u32,
}

impl DrawingDemo {
    pub const fn new() -> Self {
        Self {
            wheel_val: 0,
            current_step: 0,
            step_counter: 0,
        }
    }

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

    fn draw_rainbow_text<D>(&self, fb: &mut D, base_x: i32, base_y: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        const TEXT: &[u8] = b"ESP32 DMA";

        for (i, &c) in TEXT.iter().enumerate() {
            let color = self.color_wheel((i as u8 * 32).wrapping_add(self.wheel_val));
            let style = MonoTextStyle::new(&FONT_6X10, color);

            // Create a byte buffer that lives long enough
            let char_buf = [c];
            let c_str = core::str::from_utf8(&char_buf).unwrap_or("?");

            let text = Text::new(c_str, Point::new(base_x + (i as i32 * 6), base_y), style);

            text.draw(fb)?;
        }
        Ok(())
    }

    pub fn update<D>(&mut self, fb: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
    {
        // Define colors as constants
        const WHITE: Rgb888 = Rgb888::new(255, 255, 255);
        const BLACK: Rgb888 = Rgb888::new(0, 0, 0);
        const RED: Rgb888 = Rgb888::new(255, 0, 0);
        const GREEN: Rgb888 = Rgb888::new(0, 255, 0);
        const BLUE: Rgb888 = Rgb888::new(0, 0, 255);
        const YELLOW: Rgb888 = Rgb888::new(255, 255, 0);
        const VIOLET: Rgb888 = Rgb888::new(255, 0, 255);

        match self.current_step {
            0 => {
                // Fill screen with green
                let rectangle = Rectangle::new(Point::zero(), Size::new(64, 32));
                rectangle
                    .into_styled(PrimitiveStyle::with_fill(GREEN))
                    .draw(fb)?;
            }
            1 => {
                // Draw yellow border
                let rectangle = Rectangle::new(Point::zero(), Size::new(64, 32));
                rectangle
                    .into_styled(PrimitiveStyle::with_stroke(YELLOW, 1))
                    .draw(fb)?;
            }
            2 => {
                // Draw red X
                let line1 = Line::new(Point::zero(), Point::new(63, 31));
                line1
                    .into_styled(PrimitiveStyle::with_stroke(RED, 1))
                    .draw(fb)?;
                let line2 = Line::new(Point::new(63, 0), Point::new(0, 31));
                line2
                    .into_styled(PrimitiveStyle::with_stroke(RED, 1))
                    .draw(fb)?;
            }
            3 => {
                // Draw blue circle
                let circle = Circle::new(Point::new(10, 10), 10);
                circle
                    .into_styled(PrimitiveStyle::with_stroke(BLUE, 1))
                    .draw(fb)?;
            }
            4 => {
                // Fill violet circle
                let circle = Circle::new(Point::new(40, 21), 10);
                circle
                    .into_styled(PrimitiveStyle::with_fill(VIOLET))
                    .draw(fb)?;
            }
            5 => {
                // Clear screen and draw animated text
                fb.clear(BLACK)?; // Provide a color to clear the framebuffer
                self.draw_rainbow_text(fb, 5, 10)?;

                // Increment wheel value for animation
                self.wheel_val = self.wheel_val.wrapping_add(1);
                return Ok(());
            }
            _ => {
                self.current_step = 0;
                self.step_counter = 0;
                return Ok(());
            }
        }

        // Progress to next step after delay
        self.step_counter = self.step_counter.wrapping_add(1);
        if self.step_counter >= 100 {
            // Adjust this value to control step duration
            self.current_step = self.current_step.wrapping_add(1);
            self.step_counter = 0;
        }

        Ok(())
    }
}
