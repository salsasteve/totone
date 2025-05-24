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
pub mod bar_graph;
pub use bar_graph::*;
pub mod renderer;
pub use renderer::*;
