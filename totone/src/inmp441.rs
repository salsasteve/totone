use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{
    dma::{DmaChannel, DmaChannelFor},
    gpio::AnyPin,
    i2s::master::{AnyI2s, DataFormat, I2s, I2sRx, Standard},
    peripherals::{I2S0, I2S1},
    Async,
};
use fugit::Hertz;

const DMA_BUFFER_SIZE: usize = 4096 * 3;
const BYTES_PER_SAMPLE: usize = 2;
const FFT_SIZE: usize = 1024;

enum I2S {
    I2S0(I2S0),
    I2S1(I2S1),
}
pub struct Inmp441 {
    i2s_rx: I2sRx<'static, Async>,
    sample_rate: Hertz<u32>,
}

impl Inmp441 {
    pub async fn new<CH>(
        i2s: I2S,
        mclk: AnyPin,
        bclk: AnyPin,
        ws: AnyPin,
        din: AnyPin,
        dma_channel: CH,
        sample_rate: Hertz<u32>,
    ) -> Result<Self, ()>
    where
        CH: DmaChannel + DmaChannelFor<AnyI2s>,
    {
        let i2s = I2s::new(
            i2s,
            Standard::Philips,
            DataFormat::Data16Channel16,
            sample_rate,
            dma_channel,
            rx_descriptors,
            tx_descriptors,
        )
        .with_mclk(mclk)
        .into_async();

        let i2s_rx = i2s.i2s_rx.with_bclk(bclk).with_ws(ws).with_din(din).build();

        Ok(Inmp441 {
            i2s_rx,
            sample_rate,
        })
    }
}

pub async fn start_reading(
    i2s_rx: I2sRx<'static, Async>,
    signal: &'static Signal<CriticalSectionRawMutex, [i16; 1024]>,
    rx_buffer: &'static mut [u8; DMA_BUFFER_SIZE],
) {
    let mut transaction = i2s_rx.read_dma_circular_async(rx_buffer).unwrap();
    let mut data = [0u8; 4096 * 2]; // Buffer to hold read data

    loop {
        let _avail = transaction.available().await.unwrap();
        let count = transaction.pop(&mut data).await.unwrap();

        if count >= FFT_SIZE * BYTES_PER_SAMPLE {
            let mut samples: [i16; FFT_SIZE] = [0i16; FFT_SIZE];
            for (i, chunk) in data.chunks_exact(4).enumerate().take(FFT_SIZE) {
                samples[i] = i16::from_le_bytes([chunk[0], chunk[1]]);
            }
            signal.signal(samples); // Send samples to the processing task
        }
    }
}

#[embassy_executor::task]
pub async fn inmp441_task(
    microphone: Inmp441,
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
    rx_buffer: &'static mut [u8; DMA_BUFFER_SIZE],
) {
    info!("Starting microphone task");
    start_reading(microphone.i2s_rx, signal, rx_buffer).await;
}

pub fn spawn_inmp441_task(
    spawner: &Spawner,
    microphone: Inmp441,
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
    rx_buffer: &'static mut [u8; DMA_BUFFER_SIZE],
) -> Result<(), ()> {
    spawner
        .spawn(inmp441_task(microphone, signal, rx_buffer))
        .map_err(|_| ())
}
