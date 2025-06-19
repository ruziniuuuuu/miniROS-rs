//! MiniROS - A high-performance, cross-platform ROS2-like middleware
//!
//! This library provides core robotics communication primitives with
//! ROS2-compatible DDS transport layer for maximum interoperability.

pub mod action;
pub mod core;
pub mod discovery;
pub mod error;
pub mod message;
pub mod node;
pub mod parameter;
pub mod publisher;
pub mod service;
pub mod subscriber;

// Transport layer - DDS-based by default for ROS2 compatibility
#[cfg(feature = "dds-transport")]
pub mod dds_transport;
#[cfg(feature = "tcp-transport")]
pub mod transport;
pub mod zenoh_transport;

// Optional features
#[cfg(feature = "python")]
pub mod python;
#[cfg(feature = "visualization")]
pub mod visualization;

// Re-exports for convenience
pub use action::{ActionClient, ActionGoal, ActionResult, ActionServer, GoalStatus};
pub use core::Context;
pub use error::{MiniRosError, Result};
pub use message::{BoolMsg, EmptyMsg, Float64Msg, Int32Msg, Message, Stamped, StringMsg};
pub use node::Node;
pub use parameter::{ParameterClient, ParameterServer, ParameterValue};
pub use publisher::Publisher;
pub use service::{Service, ServiceClient};
pub use subscriber::Subscriber;

/// Prelude module for common imports
pub mod prelude {
    pub use crate::{
        ActionClient, ActionServer, BoolMsg, Context, EmptyMsg, Float64Msg, GoalStatus, Int32Msg,
        Message, MiniRosError, Node, ParameterClient, ParameterServer, ParameterValue, Publisher,
        Result, Service, ServiceClient, Stamped, StringMsg, Subscriber,
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
