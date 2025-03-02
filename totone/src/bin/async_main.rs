#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use defmt::info;
use defmt_rtt as _;
use embassy_sync::{signal::Signal, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    dma_buffers,
    i2s::master::{I2s, I2sRx, Standard, DataFormat},
    interrupt::{software::SoftwareInterruptControl, Priority},
    time::RateExtU32,
    timer::{timg::TimerGroup, AnyTimer},
    Async,
};
use esp_hal_embassy::InterruptExecutor;
use static_cell::StaticCell;
use micro_dsp::process_frame;


const FFT_SIZE: usize = 1024;

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
async fn audio_processor(
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
) {
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
            fft_data[7],
            fft_data[8],
            fft_data[9],
            fft_data[10],
            fft_data[11],
            fft_data[12]
        );
        
    }
}

#[esp_hal_embassy::main]
async fn main(low_prio_spawner: Spawner) {
    info!("Init!");

    // Initialize the heap for dynamic memory allocation
    init_heap();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let timer1: AnyTimer = timg1.timer0.into();

    esp_hal_embassy::init([timer0, timer1]);

    let dma_channel = peripherals.DMA_CH0;

    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(4096 * 3, 0);

    let mclk = peripherals.GPIO0;
    let bclk = peripherals.GPIO2;
    let ws = peripherals.GPIO4;
    let din = peripherals.GPIO5;

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100.Hz(),
        dma_channel,
        rx_descriptors,
        tx_descriptors,
    )
    .with_mclk(mclk)
    .into_async();

    let i2s_rx = i2s
    .i2s_rx
    .with_bclk(bclk)
    .with_ws(ws)
    .with_din(din)
    .build();

    static SAMPLES_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>> = StaticCell::new();
    let samples_signal = &*SAMPLES_SIGNAL.init(Signal::new());

    static HIGH_PRIO_EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    let high_prio_executor = InterruptExecutor::new(sw_ints.software_interrupt2);
    let executor = HIGH_PRIO_EXECUTOR.init(high_prio_executor);

    let high_prio_spawner = executor.start(Priority::Priority3);
    low_prio_spawner.must_spawn(audio_processor(samples_signal));


    high_prio_spawner.must_spawn(microphone_reader(i2s_rx, rx_buffer, samples_signal));
}
