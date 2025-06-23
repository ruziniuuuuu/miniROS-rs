//! Core MiniROS components and abstractions
//!
//! This module contains the fundamental building blocks of MiniROS including
//! the context management, node abstraction, error handling, and message definitions.

pub mod context;
pub mod error;
pub mod message;
pub mod node;

// Re-export core types for convenience
pub use context::Context;
pub use error::{MiniRosError, Result};
pub use message::{BoolMsg, EmptyMsg, Float64Msg, Int32Msg, Message, Stamped, StringMsg};
pub use node::Node;
