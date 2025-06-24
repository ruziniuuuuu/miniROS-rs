//! MiniROS - A high-performance, cross-platform ROS2-like middleware
//!
//! This library provides core robotics communication primitives with
//! ROS2-compatible DDS transport layer for maximum interoperability.

// Core modules
pub mod communication;
pub mod core;
pub mod system;
pub mod transport;
pub mod types;

// Optional features
pub mod features;

// Re-exports for convenience
pub use core::{BoolMsg, EmptyMsg, Float64Msg, Int32Msg, Message, Stamped, StringMsg};
pub use core::{Context, MiniRosError, Node, Result};

pub use communication::{
    ActionClient, ActionGoal, ActionResult, ActionServer, GoalStatus, Publisher, Service,
    ServiceClient, Subscriber,
};

pub use system::{
    DiscoveryService, LaunchConfig, LaunchDescription, LaunchManager, NodeLaunchConfig, Package,
    PackageManager, ParameterClient, ParameterServer, ParameterValue, Ros2Bridge, Ros2BridgeConfig,
    create_ros2_bridge, create_turtlebot_bridge,
    Plugin, PluginManager, PluginConfig, PluginStatus, CustomTransportPlugin, MonitoringPlugin,
    PluginConfigSchema, PluginContext, ConfigField, ConfigFieldType,
};

// Transport layer exports based on features
#[cfg(feature = "tcp-transport")]
pub use transport::tcp::{MessageBroker, TcpTransport, Transport, TransportManager, UdpTransport};

#[cfg(feature = "dds-transport")]
pub use transport::DdsTransport;

pub use transport::ZenohTransport;

// Feature exports
#[cfg(feature = "visualization")]
pub use features::*;

#[cfg(not(feature = "visualization"))]
pub mod visualization {
    //! Placeholder visualization module when feature is disabled
    pub use crate::features::visualization::*;
}

pub use features::benchmarks::{
    BenchmarkConfig, BenchmarkFramework, BenchmarkSuite, LatencyMetrics, PerformanceMetrics,
    ThroughputMetrics,
};

// Re-export modules for compatibility with examples
pub mod message {
    pub use crate::core::message::*;
}

pub mod node {
    pub use crate::core::node::*;
}

pub mod error {
    pub use crate::core::error::*;
}

pub mod parameter {
    pub use crate::system::parameter::*;
}

pub mod benchmarks {
    pub use crate::features::benchmarks::*;
}

/// Prelude module for common imports
pub mod prelude {
    pub use crate::{
        ActionClient, ActionServer, BoolMsg, Context, EmptyMsg, Float64Msg, GoalStatus, Int32Msg,
        LaunchConfig, LaunchDescription, LaunchManager, Message, MiniRosError, Node,
        NodeLaunchConfig, Package, PackageManager, ParameterClient, ParameterServer,
        ParameterValue, Publisher, Result, Service, ServiceClient, Stamped, StringMsg, Subscriber,
    };
    pub use std::time::Duration;
}

// Python module entry point
#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn _core(_py: Python, m: &PyModule) -> PyResult<()> {
    features::python::init_python_module(m)?;
    Ok(())
}
