#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::fmt;
use core::mem::{size_of, MaybeUninit};
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "defmt")]
use defmt::info;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_executor::task;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::{
    geometry::Point,
    mono_font::{
        ascii::{FONT_5X7, FONT_6X10},
        MonoTextStyle, MonoTextStyleBuilder,
    },
    pixelcolor::{Rgb888, RgbColor},
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
    Drawable,
};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::I2s0DmaChannel,
    gpio::{AnyPin, Pin},
    i2s::parallel::AnyI2s,
    interrupt::{software::SoftwareInterruptControl, Priority},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::{
    framebuffer::{compute_frame_count, compute_rows, DmaFrameBuffer},
    i2s_parallel::Hub75,
    Hub75Pins,
};

use micro_viz::DrawingDemo;

use micromath::F32Ext;
use heapless::String;
#[cfg(feature = "log")]
use log::info;

// Constants for LED matrix configuration
const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 4;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);
const FPS_INTERVAL: Duration = Duration::from_secs(1);

// Common colors
const WHITE: Rgb888 = Rgb888::new(255, 255, 255);
const BLACK: Rgb888 = Rgb888::new(0, 0, 0);
const RED: Rgb888 = Rgb888::new(255, 0, 0);
const GREEN: Rgb888 = Rgb888::new(0, 255, 0);
const BLUE: Rgb888 = Rgb888::new(0, 0, 255);
const YELLOW: Rgb888 = Rgb888::new(255, 255, 0);
const VIOLET: Rgb888 = Rgb888::new(255, 0, 255);

// Static atomic counters for performance monitoring
static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

// Type aliases for readability
type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

/// Collection of pins and peripherals for the Hub75 LED matrix
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


/// Initialize the heap for dynamic memory allocation
fn init_heap() {
    const HEAP_SIZE: usize = 3 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

/// Macro to create static variables
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// Creates a Hub75 driver instance
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
    .expect("Failed to create Hub75 driver")
    .into_async()
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
        demo.update(fb, [4,12,20,28,36,44,52,60]).unwrap();

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
            .expect("Failed to start render");

        xfer.wait_for_done()
            .await
            .expect("Rendering wait_for_done failed");

        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("Transfer failed");

        count += 1;
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
    // Initialize logging based on feature flags
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    // Initialize the heap for dynamic memory allocation
    init_heap();

    info!("Main starting!");
    info!("main: stack size: {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });
    info!(
        "ROWS: {}, COLS: {}, BITS: {}, FRAME_COUNT: {}",
        ROWS, COLS, BITS, FRAME_COUNT
    );

    // Initialize ESP32 peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    info!("Initializing Embassy");
    esp_hal_embassy::init(timg0.timer0);

    info!("Initializing framebuffer exchange");
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    info!("Initializing framebuffers");
    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());
    fb0.clear();
    fb1.clear();

    // Configure Hub75 pins and peripherals
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
        addr4: peripherals.GPIO18.degrade(),
        blank: peripherals.GPIO15.degrade(),
        clock: peripherals.GPIO16.degrade(),
        latch: peripherals.GPIO4.degrade(),
    };

    // Create high-priority executor for the Hub75 task
    let hp_executor = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(software_interrupt)
    );
    let high_pri_spawner = hp_executor.start(Priority::Priority3);

    // Spawn tasks
    high_pri_spawner
        .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
        .expect("Failed to spawn hub75_task");

    spawner
        .spawn(display_task(&TX, &RX, fb0))
        .expect("Failed to spawn display_task");

    // Main loop - just increment counter for watchdog
    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}