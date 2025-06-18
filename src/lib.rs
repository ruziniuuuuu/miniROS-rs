//! MiniROS - A high-performance, cross-platform ROS2-like middleware
//! 
//! This library provides core robotics communication primitives with
//! ROS2-compatible DDS transport layer for maximum interoperability.

pub mod core;
pub mod error;
pub mod message;
pub mod node;
pub mod publisher;
pub mod subscriber;
pub mod service;
pub mod discovery;

// Transport layer - DDS-based by default for ROS2 compatibility
#[cfg(feature = "dds-transport")]
pub mod dds_transport;
#[cfg(feature = "tcp-transport")]
pub mod transport;
pub mod zenoh_transport;

// Optional features
#[cfg(feature = "visualization")]
pub mod visualization;
#[cfg(feature = "python")]
pub mod python;

// Re-exports for convenience
pub use error::{Result, MiniRosError};
pub use core::Context;
pub use node::Node;
pub use publisher::Publisher;
pub use subscriber::Subscriber;
pub use service::{Service, ServiceClient};
pub use message::{Message, StringMsg, Int32Msg, Float64Msg, BoolMsg, EmptyMsg, Stamped};

/// Prelude module for common imports
pub mod prelude {
    pub use crate::{
        Result, MiniRosError, Context, Node, Publisher, Subscriber, 
        Service, ServiceClient, Message, StringMsg, Int32Msg, 
        Float64Msg, BoolMsg, EmptyMsg, Stamped
    };
    pub use std::time::Duration;
}

// Python module entry point
#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn _core(_py: Python, m: &PyModule) -> PyResult<()> {
    python::init_python_module(m)?;
    Ok(())
}