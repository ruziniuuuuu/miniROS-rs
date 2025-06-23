//! Optional features and extensions for MiniROS
//!
//! This module contains optional functionality that can be enabled via feature flags,
//! including Python bindings, visualization capabilities, and performance benchmarks.

#[cfg(feature = "python")]
pub mod python;

#[cfg(feature = "visualization")]
pub mod visualization;

pub mod benchmarks;

// Re-export feature types for convenience
#[cfg(feature = "python")]
pub use python::*;

#[cfg(feature = "visualization")]
pub use visualization::*;

pub use benchmarks::*;
