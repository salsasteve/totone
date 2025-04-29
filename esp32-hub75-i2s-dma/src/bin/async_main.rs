#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use core::mem::{size_of, MaybeUninit};
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use defmt_rtt as _;
use embassy_executor::task;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma_buffers,
    gpio::{AnyPin, Pin},
    i2s::master::{DataFormat, I2s, I2sRx, Standard},
    interrupt::{software::SoftwareInterruptControl, Priority},
    peripherals::LCD_CAM,
    time::Rate,
    timer::timg::TimerGroup,
    Async,
};
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::{
    framebuffer::{compute_frame_count, compute_rows, DmaFrameBuffer},
    lcd_cam::Hub75,
    Hub75Pins,
};

use micro_viz::DrawingDemo;

use embassy_sync::channel::Channel;
use micro_dsp::process_frame;
use micromath::F32Ext;
use static_cell::StaticCell;

// Constants for LED matrix configuration
const ROWS: usize = 64;
const COLS: usize = 64;
const BITS: u8 = 4;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);
const FPS_INTERVAL: Duration = Duration::from_secs(1);
const FFT_SIZE: usize = 1024;
const FFT_CHANNEL_CAPACITY: usize = 1;
static SAMPLES_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>> =
    StaticCell::new();

// Static atomic counters for performance monitoring
static REFRESH_RATE: AtomicU32 = AtomicU32::new(0);
static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);
const HEAP_SIZE: usize = 128 * 1024;

// Type aliases for readability
type Hub75Type = Hub75<'static, esp_hal::Async>;
type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

type FftChannelType = Channel<CriticalSectionRawMutex, [f32; 8], FFT_CHANNEL_CAPACITY>;
static FFT_CHANNEL: StaticCell<FftChannelType> = StaticCell::new();

/// Collection of pins and peripherals for the Hub75 LED matrix
pub struct Hub75Peripherals<'d> {
    pub lcd_cam: LCD_CAM<'d>,
    pub dma_channel: esp_hal::dma::DmaChannel0<'d>,
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub addr0: AnyPin<'d>,
    pub addr1: AnyPin<'d>,
    pub addr2: AnyPin<'d>,
    pub addr3: AnyPin<'d>,
    pub addr4: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
}

//  Initialize the heap for dynamic memory allocation
fn init_heap() {
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

fn create_hub75(peripherals: Hub75Peripherals<'static>) -> Hub75<'static, esp_hal::Async> {
    let channel = peripherals.dma_channel;
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, size_of::<FBType>());

    let pins = Hub75Pins {
        red1: peripherals.red1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        blu2: peripherals.blu2,
        addr0: peripherals.addr0, // A
        addr2: peripherals.addr2, // C
        clock: peripherals.clock,
        blank: peripherals.blank, // Last pin on leftside
        // ---------------------
        grn1: peripherals.grn1,
        // GND pin
        grn2: peripherals.grn2,
        addr4: peripherals.addr4, // E
        addr1: peripherals.addr1, // B
        addr3: peripherals.addr3, // D
        latch: peripherals.latch,
        // GND pin
    };

    return Hub75Type::new_async(
        peripherals.lcd_cam,
        pins,
        channel,
        tx_descriptors,
        Rate::from_mhz(30),
    )
    .expect("failed to create Hub75!");
}



#[task]
async fn display_task(
    tx: &'static FrameBufferExchange, // Swapped TX/RX names for clarity: TX to Hub75, RX from Hub75
    rx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
    fft_receiver: embassy_sync::channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        [f32; 8],
        FFT_CHANNEL_CAPACITY,
    >, // <-- Add receiver
) {
    info!("display_task: starting!");

    let mut demo = DrawingDemo::new(COLS as u16, ROWS as u16, 1, 50); // Adjust params if needed
    let mut count = 0u32;
    let mut start = Instant::now();
    // --- Store the last received FFT data ---
    let mut current_fft_data: [f32; 8] = [0.0; 8];

    // Define the input and output ranges for scaling
    let input_min: f32 = 0.0;
    let input_max: f32 = 10.0;         // Your desired maximum input value
    let output_max_height: f32 = ROWS as f32; // Max pixel height (e.g., 64.0)

    loop {
        if let Ok(new_data) = fft_receiver.try_receive() {
            current_fft_data = new_data;
            // Optional: Log received raw data for debugging the range
            // info!("Raw FFT Data: {:?}", current_fft_data);
        }

        fb.clear();

        // --- Apply the New Scaling Logic ---
        let heights: [i32; 8] = core::array::from_fn(|i| {
            // 1. Get the raw input value for this bin
            let input_value = current_fft_data[i];

            // 2. Clamp the input value to the expected range [0.0, 10.0]
            //    Values below 0 become 0, values above 10 become 10.
            let clamped_input = input_value.max(input_min).min(input_max);

            // 3. Calculate the proportion within the input range (0.0 to 1.0)
            //    Avoid division by zero if input_max could equal input_min (not the case here)
            let proportion = (clamped_input - input_min) / (input_max - input_min); // (input / 10.0)

            // 4. Scale the proportion to the output pixel height range [0.0, 64.0]
            let scaled_height = proportion * output_max_height;

            // 5. Convert to integer height (truncates)
            scaled_height as i32
        });

        // Optional: Log the calculated heights for debugging
        info!("Scaled Heights: {:?}", heights);

        demo.update(fb, heights).unwrap();

        // --- Swap framebuffers with hub75_task ---
        tx.signal(fb);
        fb = rx.wait().await;

        // --- Update counters ---
        count += 1;
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
        }
    }
}

#[task]
async fn hub75_task(
    // Assuming you fixed this to accept the driver instance directly
    // If not, you need to apply the fix from the previous answer first!
    peripherals: Hub75Peripherals<'static>, // Pass hub75 directly if possible, avoid recreating it
    tx_signal_from_display: &'static FrameBufferExchange, // Renamed for clarity: Wait on this
    rx_signal_to_display: &'static FrameBufferExchange,   // Renamed for clarity: Signal on this
    mut fb: &'static mut FBType,                          // Initial buffer (e.g., fb1)
) {
    info!("hub75_task: starting!");
    let mut hub75 = create_hub75(peripherals); // Avoid recreating if possible
    let mut count = 0u32;
    let mut start = Instant::now();

    loop {
        // --- Wait for a rendered frame FROM display_task ON tx_signal_from_display ---
        defmt::trace!("Hub75: Waiting for display frame on TX signal..."); // Use trace for less verbose loops
        let received_fb = tx_signal_from_display.wait().await; // Corrected: Wait on TX signal
        defmt::trace!("Hub75: Received display frame.");

        // --- Signal TO display_task ON rx_signal_to_display that the previous buffer is free ---
        defmt::trace!("Hub75: Signaling back on RX signal.");
        rx_signal_to_display.signal(fb); // Corrected: Signal on RX signal (using the buffer *before* update)

        // Update local buffer variable to the newly received one for rendering
        fb = received_fb;
        defmt::trace!("Hub75: Starting render...");

        // --- Render the current framebuffer (fb) ---
        let mut xfer = hub75.render(fb)
            .map_err(|(e, _hub75)| { defmt::error!("Render start error"); e })
            .expect("Failed to start render");

        defmt::trace!("Hub75: Waiting for render done...");
        xfer.wait_for_done().await
            .map_err(|e| { defmt::error!("Render wait_for_done error"); e })
            .expect("Rendering wait_for_done failed");
        defmt::trace!("Hub75: Render done.");

        // Finalize transfer and get driver back
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75; // Update driver instance
        result.map_err(|e| { defmt::error!("Render result error"); e })
              .expect("Transfer failed");
        defmt::trace!("Hub75: Render cycle complete.");

        // --- Update counters ---
        count += 1;
        if start.elapsed() > FPS_INTERVAL {
            REFRESH_RATE.store(count, Ordering::Relaxed);
            count = 0;
            start = Instant::now();
            // info!("Hub75 FPS: {}", REFRESH_RATE.load(Ordering::Relaxed)); // Optional FPS log
        }
    }
}

// --- Minor changes in main related to hub75_task ---

// In main, when creating hub75:
// let hub75_driver = create_hub75(hub75_peripherals); // Create it once

// In cpu1_fnctn closure:
// high_pri_spawner.spawn(hub75_task(hub75_driver, &RX, &TX, fb1)).ok(); // Pass the driver instance


#[embassy_executor::task]
async fn microphone_reader(
    i2s_rx: I2sRx<'static, Async>,
    buffer: &'static mut [u8],
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
) {
    info!("Starting microphone_reader task");

    const BYTES_PER_SAMPLE: usize = 2;
    const FFT_SIZE: usize = 1024;

    let mut data = [0u8; 4096 * 2];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();

    loop {
        let _avail = transaction.available().await.unwrap();
        let count = transaction.pop(&mut data).await.unwrap();

        if count >= FFT_SIZE * BYTES_PER_SAMPLE {
            let mut samples: [i16; FFT_SIZE] = [0i16; FFT_SIZE];
            for (i, chunk) in data.chunks_exact(4).enumerate().take(FFT_SIZE) {
                samples[i] = i16::from_le_bytes([chunk[0], chunk[1]]);
            }

            // Signal the data is ready for processing
            signal.signal(samples);

            // Simple output of first few values for debugging
            // info!("Samples: {:?}, {:?}, {:?}", samples[0], samples[1], samples[2]);
        }
    }
}

#[embassy_executor::task]
async fn audio_processor(
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
    fft_sender: embassy_sync::channel::Sender<
        'static,
        CriticalSectionRawMutex,
        [f32; 8],
        FFT_CHANNEL_CAPACITY,
    >, // <-- Add sender
) {
    info!("Starting audio_processor task");

    loop {
        let samples = signal.wait().await;
        let samples_clone = samples; // Clone if process_frame consumes it, or pass slice

        match process_frame(&samples_clone) {
            // Assuming process_frame takes &[i16]
            Ok(fft_data) => {
                // --- Process raw FFT data into 8 display values ---
                // Example: Select 8 specific bins or average ranges.
                // This depends heavily on how `fft_data` is structured and what you want to display.
                // Replace this with your actual logic.
                let display_values: [f32; 8] = [
                    fft_data[8].abs(),  // Example: Magnitude of bin 8
                    fft_data[16].abs(), // Example: Magnitude of bin 16
                    fft_data[32].abs(),
                    fft_data[64].abs(),
                    fft_data[128].abs(),
                    fft_data[200].abs(),
                    fft_data[300].abs(),
                    fft_data[400].abs(), // Ensure these indices are valid!
                ];

                // --- Send the processed data ---
                // Use try_send: if the channel is full (display hasn't picked up last value),
                // this value is simply dropped. Prevents audio processing from stalling.
                let _ = fft_sender.try_send(display_values);

                // Optional: Keep your debug logging if needed
                // info!( /* ... */ );
            }
            Err(e) => {
                defmt::error!("FFT Processing Error: {:?}", e);
            }
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

    let dma_channel = peripherals.DMA_CH1;

    let samples_signal = &*SAMPLES_SIGNAL.init(Signal::new());
    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(4096 * 3, 0);
    let ws = peripherals.GPIO5;
    let din = peripherals.GPIO6;
    let bclk = peripherals.GPIO4;

    // --- Initialize the Channel ---
    let fft_channel = FFT_CHANNEL.init(Channel::new());
    let fft_sender = fft_channel.sender(); // For Core 0 (audio_processor)
    let fft_receiver = fft_channel.receiver(); // For Core 1 (display_task)

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        Rate::from_khz(48),
        dma_channel,
    )
    .into_async();
    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(bclk)
        .with_ws(ws)
        .with_din(din)
        .build(rx_descriptors);
    spawner.must_spawn(microphone_reader(i2s_rx, rx_buffer, &samples_signal));
    spawner.must_spawn(audio_processor(samples_signal, fft_sender));

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
        lcd_cam: peripherals.LCD_CAM,
        dma_channel: peripherals.DMA_CH0,
        red1: peripherals.GPIO3.degrade(),
        blu1: peripherals.GPIO46.degrade(),
        red2: peripherals.GPIO9.degrade(),
        blu2: peripherals.GPIO10.degrade(),
        addr0: peripherals.GPIO11.degrade(),
        addr2: peripherals.GPIO12.degrade(),
        clock: peripherals.GPIO13.degrade(),
        blank: peripherals.GPIO14.degrade(),

        grn1: peripherals.GPIO43.degrade(),
        grn2: peripherals.GPIO44.degrade(),
        addr4: peripherals.GPIO1.degrade(),  // E
        addr1: peripherals.GPIO2.degrade(),  // B
        addr3: peripherals.GPIO42.degrade(), // D
        latch: peripherals.GPIO41.degrade(),
    };

    // run hub75 and display on second core
    // let cpu1_fnctn = {
    //     move || {
    //         use esp_hal_embassy::Executor;
    //         let hp_executor = mk_static!(
    //             InterruptExecutor<2>,
    //             InterruptExecutor::new(software_interrupt)
    //         );
    //         let high_pri_spawner = hp_executor.start(Priority::Priority3);

    //         // hub75 runs as high priority task
    //         high_pri_spawner
    //             .spawn(hub75_task(hub75_peripherals, &RX, &TX, fb1))
    //             .ok();

    //         let lp_executor = mk_static!(Executor, Executor::new());
    //         // display task runs as low priority task
    //         lp_executor.run(|spawner| {
    //             spawner.spawn(display_task(&TX, &RX, fb0)).ok();
    //         });
    //     }
    // };

    let cpu1_fnctn = {
        // Capture the receiver end here
        let fft_receiver_clone = fft_receiver.clone(); // Clone receiver to move into closure
        move || {
            use esp_hal_embassy::Executor;
            let hp_executor = mk_static!(
                InterruptExecutor<2>,
                InterruptExecutor::new(software_interrupt)
            );
            let high_pri_spawner = hp_executor.start(Priority::Priority3);
            high_pri_spawner
                .spawn(hub75_task(hub75_peripherals, &TX, &RX, fb1)) // Pass peripherals if needed, check lifetimes
                .ok();

            let lp_executor = mk_static!(Executor, Executor::new());
            lp_executor.run(|spawner| {
                // Pass the receiver end to display_task
                spawner
                    .spawn(display_task(&TX, &RX, fb0, fft_receiver_clone))
                    .ok(); // <-- Pass receiver
            });
        }
    };

    use esp_hal::system::CpuControl;
    use esp_hal::system::Stack;
    const DISPLAY_STACK_SIZE: usize = 8192;
    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());
    let mut _cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    #[allow(static_mut_refs)]
    let _guard = _cpu_control
        .start_app_core(app_core_stack, cpu1_fnctn)
        .unwrap();

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
