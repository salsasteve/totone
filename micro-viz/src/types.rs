use alloc::{vec::Vec};

pub struct BandData {
    pub index: usize,          // The index of this band (0, 1, 2...)
    pub current_value: f32,    // The raw summarized value from the FFT (e.g., the "max")
    pub normalized_value: f32, // The value scaled from 0.0 to 1.0 based on its rolling max
    pub peak_value: f32,       // The current rolling max for this band
}

// A collection of all bands for one frame of visualization
pub type AudioFrame = Vec<BandData>;