//! # MiniROS - A high-performance, cross-platform ROS2-like middleware
//! 
//! MiniROS provides core robotics middleware functionality with focus on:
//! - High-performance inter-process communication
//! - Cross-platform compatibility
//! - Simple and clean API
//! - Async/await support

pub mod core;
pub mod node;
pub mod publisher;
pub mod subscriber;
pub mod message;
pub mod service;
pub mod discovery;
pub mod transport;
pub mod error;
pub mod zenoh_transport;
pub mod visualization;

#[cfg(feature = "python")]
pub mod python;

// Prelude module for easy imports
pub mod prelude {
    pub use crate::{
        node::Node,
        publisher::Publisher,
        subscriber::Subscriber,
        message::Message,
        service::{Service, ServiceClient},
        error::{Result, MiniRosError},
        core::Context,
    };
}

// Re-export main types for easy access
pub use crate::{
    node::Node,
    publisher::Publisher,
    subscriber::Subscriber,
    message::Message,
    service::{Service, ServiceClient},
    error::{Result, MiniRosError},
    core::Context,
};

// Python module entry point
#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn _core(_py: Python, m: &PyModule) -> PyResult<()> {
    python::init_python_module(m)?;
    Ok(())
}