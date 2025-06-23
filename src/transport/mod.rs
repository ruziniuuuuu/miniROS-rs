//! Transport layer implementations for MiniROS
//!
//! This module provides different transport layer implementations including
//! TCP/UDP transport, DDS transport for ROS2 compatibility, and Zenoh transport.

#[cfg(feature = "tcp-transport")]
pub mod tcp;

#[cfg(feature = "dds-transport")]
pub mod dds_transport;

pub mod zenoh_transport;

// Re-export transport types based on enabled features
#[cfg(feature = "tcp-transport")]
pub use tcp::{MessageBroker, TcpTransport, Transport, TransportManager, UdpTransport};

#[cfg(feature = "dds-transport")]
pub use dds_transport::DdsTransport;

pub use zenoh_transport::ZenohTransport;
