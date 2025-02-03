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

use core::mem::MaybeUninit;
use defmt::info;
use embassy_executor::Spawner;
use esp_alloc as _;
use esp_hal::{
    dma::{Dma, DmaPriority},
    dma_buffers,
    i2s::master::{DataFormat, I2s, Standard},
    prelude::*,
    timer::timg::TimerGroup,
};
use micro_dsp::process_frame;
use {defmt_rtt as _, esp_backtrace as _};

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
const SAMPLE_RATE: u32 = 41000;
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
        SAMPLE_RATE.Hz(),
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

       
        if count >= FFT_SIZE * BYTES_PER_SAMPLE {
            let mut samples: [i16; FFT_SIZE] = [0i16; FFT_SIZE];
            for (i, chunk) in data
                .chunks_exact(4)
                .enumerate()
                .take(FFT_SIZE)
            {
                samples[i] = i16::from_be_bytes([chunk[0], chunk[1]]);
            }

            let fft_data = process_frame(&samples).unwrap();
            info!(
                "FFT: {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                fft_data[6], fft_data[7], fft_data[8], fft_data[9], fft_data[10], fft_data[11], fft_data[12], fft_data[13], fft_data[14]
            );

            // info!("Got {} bytes", count);
            
        }
    }
}
