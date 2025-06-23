//! Communication layer components for MiniROS
//!
//! This module provides the core communication primitives including
//! publishers, subscribers, services, and actions for inter-node communication.

pub mod action;
pub mod publisher;
pub mod service;
pub mod subscriber;

// Re-export communication types for convenience
pub use action::{ActionClient, ActionGoal, ActionResult, ActionServer, GoalStatus};
pub use publisher::Publisher;
pub use service::{Service, ServiceClient};
pub use subscriber::Subscriber;
