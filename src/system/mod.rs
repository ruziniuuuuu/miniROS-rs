//! System services and management for MiniROS
//!
//! This module provides system-level services including node discovery,
//! parameter management, launch system, and package management.

pub mod discovery;
pub mod launch;
pub mod packages;
pub mod parameter;

// Re-export system types for convenience
pub use discovery::DiscoveryService;
pub use launch::{LaunchConfig, LaunchDescription, LaunchManager, NodeLaunchConfig};
pub use packages::{Package, PackageManager};
pub use parameter::{ParameterClient, ParameterServer, ParameterValue};
