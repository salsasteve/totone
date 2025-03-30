#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::{
    dma_buffers,
    gpio::{Input, Output, OutputConfig, Level, Pull, InputConfig},
    i2s::master::{DataFormat, I2s, I2sRx, Standard},
    peripherals::SPI3,
    spi::{master::Spi, Error as SpiError},
    time::Rate,
    timer::{timg::TimerGroup, AnyTimer},
    Async,
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
    mut master_cs: Output<'static>,
    spi_chan: SPI3,
) {
    let mut spi_master = Spi::new(
        spi_chan,
        esp_hal::spi::master::Config::default()
            .with_frequency(esp_hal::time::Rate::from_mhz(2))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(master_sck)
    .with_mosi(master_mosi)
    .with_miso(master_miso)
    .into_async();

    let mut start_time = Instant::now();
    let mut last_elapsed_time = 0_u64;

    let mut tx_buffer = [0u8; CHUNK_SIZE];
    let mut total_data_sent = 0;
    tx_buffer[2] = 0xFF;
    tx_buffer[2047] = 0x69;

    loop {
        transmit_data(&mut spi_master, &tx_buffer, &mut master_cs)
            .await
            .unwrap();

        tx_buffer[2] = 0xFF;
        total_data_sent += tx_buffer.len();
        Timer::after(Duration::from_millis(1000)).await;

        let elapsed_time = start_time.elapsed().as_secs();
        if elapsed_time > 0 && elapsed_time % 10 == 0 && last_elapsed_time != elapsed_time {
            let data_rate =
                total_data_sent as f64 / (start_time.elapsed().as_millis() as f64 / 1000.0); // (just for debugging) there might be some imprecision due to floating point division
            info!("Total data sent: {} bytes", total_data_sent);
            // info!("Data rate: {:.2} bytes/second", data_rate);
            info!("Data value: {}", data_rate);
            start_time = Instant::now(); // Reset timing reference
            total_data_sent = 0; // Reset data counter
            last_elapsed_time = elapsed_time;

            tx_buffer[2] = 0xED; // Update marker byte after statistics reset
        }
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

    // Configure GPIO pins
    let config_in = InputConfig::default().with_pull(Pull::Up);
    let config_out = OutputConfig::default();
    

    let master_sck = Output::new(peripherals.GPIO1, Level::Low, config_out);
    let master_miso = Input::new(peripherals.GPIO35, config_in);
    let master_mosi = Output::new(peripherals.GPIO2, Level::Low, config_out);
    let cs_pin = Output::new(peripherals.GPIO21, Level::Low, config_out);

    let i2s_rx = i2s.i2s_rx.with_bclk(bclk).with_ws(ws).with_din(din).build();

    let samples_signal = &*SAMPLES_SIGNAL.init(Signal::new());

    spawner.must_spawn(microphone_reader(i2s_rx, rx_buffer, &samples_signal));
    spawner.must_spawn(audio_processor(samples_signal));
    spawner.must_spawn(send_fft_bin_magnitudes(
        master_sck,
        master_mosi,
        master_miso,
        cs_pin,
        peripherals.SPI3,
    ));
}

/// System monitoring task that runs concurrently with main operation
#[embassy_executor::task]
async fn background_task() {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        info!("Background monitor: system active");
    }
}

/// Handles SPI data transmission with proper chip select signaling protocol
///
/// This function asserts the chip select (CS) pin to initiate communication, performs
/// a non-blocking data transfer via SPI, and then releases the CS pin.
///
/// # Arguments
///
/// * `spi` - A mutable reference to the asynchronous SPI master.
/// * `data` - A slice of bytes to be transmitted.
/// * `cs` - A mutable reference to the chip select GPIO pin.
///
/// # Returns
///
/// * `Result<(), SpiError>` - Returns `Ok(())` if the transmission is successful, or an `SpiError` if it fails.
async fn transmit_data(
    spi: &mut Spi<'_, esp_hal::Async>,
    data: &[u8],
    cs: &mut Output<'_>,
) -> Result<(), SpiError> {
    // Assertion of chip select (active low)
    cs.set_low();

    // Execute non-blocking data transfer
    SpiBus::write(spi, data).await?;

    // Release chip select
    cs.set_high();

    Ok(())
}
