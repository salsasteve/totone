//! This shows how to continuously receive data via I2S.
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN
//! to read 0 or 0xFF or connect DIN to WS to read two different values.
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer.
//!
//! The following wiring is assumed:
//! - MCLK =>  GPIO0 (not supported on ESP32)
//! - BCLK =>  GPIO2
//! - WS   =>  GPIO4
//! - DIN  =>  GPIO5

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_hal::{
    dma::{Dma, DmaPriority},
    dma_buffers,
    i2s::master::{DataFormat, I2s, Standard},
    prelude::*,
    timer::timg::TimerGroup
};
use esp_alloc as _;
use {defmt_rtt as _, esp_backtrace as _};
use micro_dsp::normalize_samples;
use microdsp::common::apply_window_function;
use core::mem::MaybeUninit;


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


const BYTES_PER_SAMPLE: usize = 2;
const FFT_SIZE: usize = 1024;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    info!("Init!");
    init_heap();

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(4096 * 7, 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        41000u32.Hz(),
        dma_channel.configure(false, DmaPriority::Priority0),
        rx_descriptors,
        tx_descriptors,
    )
    .into_async();

    // For 24-bit data in 32-bit word:
    // [0000_0000][DDDD_DDDD][DDDD_DDDD][DDDD_DDDD]
    // Where D is data bits and 0 is padding

    let i2s = i2s.with_mclk(peripherals.GPIO0);

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(peripherals.GPIO2)
        .with_ws(peripherals.GPIO4)
        .with_din(peripherals.GPIO5)
        .build();

    let buffer = rx_buffer;

    info!("Start");

    let mut data = [0u8; 4096 * 2];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let _avail = transaction.available().await.unwrap();

        let count = transaction.pop(&mut data).await.unwrap();

        // get the first 1024 samples if they are available
        if count >= FFT_SIZE * BYTES_PER_SAMPLE {
            let mut samples = [0i16; FFT_SIZE];
            samples
                .iter_mut()
                .zip(data.chunks_exact(2))
                .for_each(|(sample, chunk)| {
                    *sample = i16::from_le_bytes([chunk[0], chunk[1]]);
                });

            let mut f32_samples: [f32; FFT_SIZE] = [0.0; FFT_SIZE];
            normalize_samples(&samples, &mut f32_samples);

            info!("Samples: {:?}", &f32_samples[0..5]);

            let window_function = microdsp::common::WindowFunctionType::Hann;
            apply_window_function(window_function, &mut f32_samples);
   
            info!("Samples: {:?}", &f32_samples[0..5]);

            
        }
    }
}
