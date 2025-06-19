//! Error types for MiniROS

use std::fmt;

/// Result type alias for MiniROS operations
pub type Result<T> = std::result::Result<T, MiniRosError>;

/// MiniROS error types
#[derive(Debug)]
pub enum MiniRosError {
    /// Network communication error
    NetworkError(String),
    /// Serialization/deserialization error
    SerializationError(String),
    /// Node not found error
    NodeNotFound(String),
    /// Topic not found error
    TopicNotFound(String),
    /// Service not found error
    ServiceNotFound(String),
    /// Timeout error
    Timeout(String),
    /// General I/O error
    IoError(std::io::Error),
    /// Configuration error
    ConfigError(String),
    /// Other generic error
    Other(String),
}

impl fmt::Display for MiniRosError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MiniRosError::NetworkError(msg) => write!(f, "Network error: {}", msg),
            MiniRosError::SerializationError(msg) => write!(f, "Serialization error: {}", msg),
            MiniRosError::NodeNotFound(name) => write!(f, "Node not found: {}", name),
            MiniRosError::TopicNotFound(topic) => write!(f, "Topic not found: {}", topic),
            MiniRosError::ServiceNotFound(service) => write!(f, "Service not found: {}", service),
            MiniRosError::Timeout(msg) => write!(f, "Timeout: {}", msg),
            MiniRosError::IoError(err) => write!(f, "IO error: {}", err),
            MiniRosError::ConfigError(msg) => write!(f, "Configuration error: {}", msg),
            MiniRosError::Other(msg) => write!(f, "Other error: {}", msg),
        }
    }
}

impl std::error::Error for MiniRosError {}

impl From<std::io::Error> for MiniRosError {
    fn from(err: std::io::Error) -> Self {
        MiniRosError::IoError(err)
    }
}

impl From<serde_json::Error> for MiniRosError {
    fn from(err: serde_json::Error) -> Self {
        MiniRosError::SerializationError(err.to_string())
    }
}

impl From<bincode::Error> for MiniRosError {
    fn from(err: bincode::Error) -> Self {
        MiniRosError::SerializationError(err.to_string())
    }
}
