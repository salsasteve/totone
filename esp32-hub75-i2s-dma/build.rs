//! Build script for ESP32-S3 target.
//!
//! This script configures linker scripts required for ESP32-S3:
//! - `linkall.x`: ESP32-S3 linker script (memory layout, sections)
//! - `defmt.x`: defmt logging linker script (logging support)
//!
//! These linker scripts are provided by the ESP Rust toolchain and are
//! required for proper memory layout and logging on ESP32-S3.

fn main() {
    // Link the ESP32-S3 linker script (memory layout, interrupt vectors, etc.)
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    
    // Link the defmt logging script (enables defmt logging support)
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
