// src/audio.rs
use crate::config::*;
use defmt::{error, info, trace, warn};
use embassy_executor::task;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender},
    signal::Signal,
};
use esp_hal::{i2s::master::I2sRx, Async};
use micro_dsp::process_frame; // Replace if using esp-dsp
                              // use static_cell::StaticCell;

// Statics accessible within the crate (used by main for init)
// pub(crate) static SAMPLES_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>> =
//     StaticCell::new();
// pub(crate) type FftChannelType = Channel<CriticalSectionRawMutex, [f32; 512], FFT_CHANNEL_CAPACITY>;
// pub(crate) static FFT_CHANNEL: StaticCell<FftChannelType> = StaticCell::new();

#[task]
pub async fn microphone_reader(
    mut i2s_rx: I2sRx<'static, Async>,
    buffer: &'static mut [u8], // DMA buffer provided by main
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
) {
    info!("Starting microphone_reader task");
    const BYTES_PER_SAMPLE_STEREO: usize = 4; // 2 bytes/sample * 2 channels
                                              // Buffer to pop data into, should be large enough for at least one FFT frame + margin
    let mut read_buf = [0u8; FFT_SIZE * BYTES_PER_SAMPLE_STEREO]; // e.g., 1024 * 4 = 4096 bytes

    match i2s_rx.read_dma_circular_async(buffer) {
        Ok(mut transaction) => loop {
            match transaction.pop(&mut read_buf).await {
                Ok(count) => {
                    if count > 0 {
                        trace!("I2S read {} bytes", count);
                        // Process full frames available within read_buf
                        let mut processed_offset = 0;
                        while processed_offset + (FFT_SIZE * BYTES_PER_SAMPLE_STEREO) <= count {
                            let frame_start = processed_offset;
                            let frame_end = frame_start + (FFT_SIZE * BYTES_PER_SAMPLE_STEREO);
                            let frame_data = &read_buf[frame_start..frame_end];

                            let mut samples: [i16; FFT_SIZE] = [0i16; FFT_SIZE];
                            for (i, chunk) in frame_data
                                .chunks_exact(BYTES_PER_SAMPLE_STEREO)
                                .enumerate()
                                .take(FFT_SIZE)
                            {
                                // Assuming Philips Standard: Left channel first
                                samples[i] = i16::from_le_bytes([chunk[0], chunk[1]]);
                            }
                            signal.signal(samples);
                            trace!("Sent FFT frame to processor");
                            processed_offset = frame_end;
                        }
                        // Note: Partial frames at the end of read_buf are currently ignored.
                        // More complex handling could carry them over.
                    } else {
                        // Pop returned 0 bytes, wait briefly? Or rely on await to yield.
                    }
                }
                Err(e) => {
                    error!("I2S DMA pop error: {:?}", e);
                    // Consider adding a short delay before retrying after an error
                    embassy_time::Timer::after_millis(100).await;
                }
            }
        },
        Err(e) => error!("I2S DMA read_dma_circular_async failed: {:?}", e),
    }
}

#[task]
pub async fn audio_processor(
    signal: &'static Signal<CriticalSectionRawMutex, [i16; FFT_SIZE]>,
    fft_sender: Sender<'static, CriticalSectionRawMutex, [f32; 512], FFT_CHANNEL_CAPACITY>,
) {
    info!("Starting audio_processor task");
    loop {
        let samples = signal.wait().await;
        trace!("Processing audio frame");
        match process_frame(&samples) {
            Ok(fft_data) => {
                if fft_sender.try_send(fft_data).is_err() {
                    trace!("FFT channel full, frame dropped.");
                } else {
                    trace!("Sent FFT data to display task.");
                }
            }
            Err(e) => {
                error!("FFT Processing Error: {:?}", e);
            }
        }
    }
}
