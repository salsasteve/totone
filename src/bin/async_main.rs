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

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_hal::{
    dma::{Dma, DmaPriority},
    dma_buffers,
    i2s::master::{DataFormat, I2s, Standard},
    prelude::*,
    timer::timg::TimerGroup,
    uart::{Config, Uart},
};
use {defmt_rtt as _, esp_backtrace as _};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    info!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(4096 * 8, 0);

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

    let uart_config = Config::default().baudrate(921_600);
    let uart1 = Uart::new_with_config(
        peripherals.UART1,
        uart_config,
        peripherals.GPIO18,
        peripherals.GPIO17,
    )
    .unwrap();

    let (mut _uart1_rx, _uart1_tx) = uart1.split();

    info!("Start");

    let mut data = [0u8; 4096];
    let mut transaction = i2s_rx.read_dma_circular_async(buffer).unwrap();
    loop {
        let _avail = transaction.available().await.unwrap();
        // info!("available {}", avail);

        let count = transaction.pop(&mut data).await.unwrap();

        // uart0_tx.write_bytes(&data).unwrap();
        for i in (0..10.min(count)).step_by(4) {
            let sample = i16::from_le_bytes([data[i], data[i + 1]]);
            let normalized = sample as f32 / i16::MAX as f32;
            info!("sample: {}", normalized);
        }
    }
}
