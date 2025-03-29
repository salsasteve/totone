#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    aes::dma, dma::DmaChannel0, dma_buffers, i2c::master, i2s::master::{DataFormat, I2s, I2sRx, Standard}, peripherals::SPI3, spi::{master::Spi, Error as SpiError}, time::Rate, timer::{timg::TimerGroup, AnyTimer}, Async
};

use micro_dsp::process_frame;
use static_cell::StaticCell;

const FFT_SIZE: usize = 1024;
static SAMPLES_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>> =
    StaticCell::new();

// Configuration constants
const CHUNK_SIZE: usize = 512 * 4; // 2KB buffer size (could hold 512 float values)
                                   // Ensure the CHUNK_SIZE matches the client code configuration
                                   // to prevent buffer overflow issues. 

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

#[embassy_executor::task]
async fn high_prio() {
    info!("Starting high_prio()");
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        info!("High priority ticks");
        ticker.next().await;
    }
}

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
async fn send_fft_bin_magnitudes(
    master_sck: Output<'static>,
    master_mosi: Output<'static>,
    master_miso: Input<'static>,
    master_cs: Output<'static>,
    dma_channel: DmaChannel0,
    spi_chan: SPI3,
) {

    let mut spi_master = Spi::new(
        peripherals.SPI3,
        esp_hal::spi::master::Config::default()
            .with_frequency(esp_hal::time::Rate::from_mhz(2))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO1)
    .with_mosi(peripherals.GPIO2)
    .with_miso(peripherals.GPIO35) // CS pin managed separately, dont add it here
    .into_async();

    loop {
        
    }
    
}

#[embassy_executor::task]
async fn audio_processor(signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>) {
    info!("Starting audio_processor task");

    loop {
        let samples = signal.wait().await;

        // Process the samples (e.g., FFT analysis or other audio processing)
        // If you have micro_dsp, you could use:
        let samples_clone = samples;
        let fft_data = process_frame(&samples_clone).unwrap();

        // Output the first few values for debugging
        info!(
            "FFT: {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
            fft_data[7], fft_data[8], fft_data[9], fft_data[10], fft_data[11], fft_data[12]
        );
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    info!("Init!");

    // Initialize the heap for dynamic memory allocation
    init_heap();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let timer1: AnyTimer = timg1.timer0.into();

    esp_hal_embassy::init([timer0, timer1]);
    // Launch system monitoring background task
    spawner.spawn(background_task()).unwrap();

    let dma_channel = peripherals.DMA_CH0;

    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(4096 * 3, 0);

    let ws = peripherals.GPIO5;
    let din = peripherals.GPIO6;
    let bclk = peripherals.GPIO4;

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        Rate::from_khz(48),
        dma_channel,
        rx_descriptors,
        tx_descriptors,
    )
    .into_async();

    let i2s_rx = i2s.i2s_rx.with_bclk(bclk).with_ws(ws).with_din(din).build();

    

    let samples_signal = &*SAMPLES_SIGNAL.init(Signal::new());

    spawner.must_spawn(microphone_reader(i2s_rx, rx_buffer, &samples_signal));
    spawner.must_spawn(audio_processor(samples_signal));
}

/// System monitoring task that runs concurrently with main operation
#[embassy_executor::task]
async fn background_task() {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        info!("Background monitor: system active");
    }
}