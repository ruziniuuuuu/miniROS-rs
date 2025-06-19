//! Error types for MiniROS

use std::fmt;

/// Result type for MiniROS operations
pub type Result<T> = std::result::Result<T, MiniRosError>;

/// Error types for MiniROS operations
#[derive(Debug)]
pub enum MiniRosError {
    /// Network communication error
    NetworkError(String),
    /// Serialization/deserialization error
    SerializationError(String),
    /// Service not found error
    ServiceNotFound(String),
    /// Service operation timeout
    ServiceTimeout(String),
    /// Generic timeout error
    Timeout(String),
    /// Configuration error
    ConfigError(String),
    /// IO error
    IoError(std::io::Error),
    /// Custom error
    Custom(String),
}

impl fmt::Display for MiniRosError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MiniRosError::NetworkError(msg) => write!(f, "Network error: {}", msg),
            MiniRosError::SerializationError(msg) => write!(f, "Serialization error: {}", msg),
            MiniRosError::ServiceNotFound(service) => write!(f, "Service not found: {}", service),
            MiniRosError::ServiceTimeout(service) => write!(f, "Service timeout: {}", service),
            MiniRosError::Timeout(msg) => write!(f, "Timeout: {}", msg),
            MiniRosError::ConfigError(msg) => write!(f, "Configuration error: {}", msg),
            MiniRosError::IoError(err) => write!(f, "IO error: {}", err),
            MiniRosError::Custom(msg) => write!(f, "Error: {}", msg),
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

impl From<String> for MiniRosError {
    fn from(msg: String) -> Self {
        MiniRosError::Custom(msg)
    }
}

impl From<&str> for MiniRosError {
    fn from(msg: &str) -> Self {
        MiniRosError::Custom(msg.to_string())
    }
}
