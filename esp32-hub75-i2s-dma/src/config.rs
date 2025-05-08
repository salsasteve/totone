
use embassy_time::Duration;

// --- Display Config ---
pub const ROWS: usize = 64;
pub const COLS: usize = 128;
pub const BITS: u8 = 4; // Hub75 Bit depth (Lower = Faster refresh, fewer colors)
                        // pub const NROWS: usize = compute_rows(ROWS); // ROWS / 2 = 32
                        // pub const FRAME_COUNT: usize = compute_frame_count(BITS); // 2^(BITS-1) = 8
                        // FRAME_COUNT and NROWS might be internal details of FBType, perhaps not needed here.

pub const HUB75_CLOCK_MHZ: u32 = 20; // Hub75 Clock Rate

// --- Audio Config ---
pub const SAMPLE_RATE_HZ: u32 = 48_000; // I2S Sample Rate (e.g., 48_000, 22_050, 16_000)
pub const FFT_SIZE: usize = 1024; // FFT Resolution

// --- System Config ---
pub const HEAP_SIZE: usize = 128 * 1024; // Increased Heap Size (128KB)
pub const CORE1_STACK_SIZE: usize = 16 * 1024; // Increased Stack for Core 1 (16KB)

// --- Task Timing ---
pub const FPS_INTERVAL: Duration = Duration::from_secs(1); // Interval for FPS counters

// --- Channel Config ---
pub const FFT_CHANNEL_CAPACITY: usize = 1; // Buffer size for FFT results channel
pub const NUM_FFT_BINS: usize = 32; // Number of visualizer bands
