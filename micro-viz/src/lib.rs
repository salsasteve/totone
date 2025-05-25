#![no_std]

extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

pub mod spectral_band_aggregator;
pub use spectral_band_aggregator::*;
pub mod bin_summary_strategy;
pub use bin_summary_strategy::*;
pub mod spectrum_value_animator;
pub use spectrum_value_animator::*;
pub mod viz_engine;
pub use viz_engine::*;
pub mod renderer;
pub use renderer::*;
pub mod audio_processor;
pub use audio_processor::*;
pub mod types;
pub use types::*;
pub mod color_strategy;
pub use color_strategy::*;
