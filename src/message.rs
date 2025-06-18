//! Message types and serialization for MiniROS

use serde::{Deserialize, Serialize};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Trait for all MiniROS messages
/// 
/// Messages must be serializable and deserializable for network transport
pub trait Message: Send + Sync + Serialize + for<'de> Deserialize<'de> + Clone + 'static {}

// Blanket implementation for all types that satisfy the requirements
impl<T> Message for T where T: Send + Sync + Serialize + for<'de> Deserialize<'de> + Clone + 'static {}

/// Standard message header containing metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    /// Sequence number for message ordering
    pub seq: u64,
    /// Timestamp when message was created (nanoseconds since Unix epoch)
    pub stamp: u64,
    /// Frame ID for coordinate transformations
    pub frame_id: String,
}

impl Header {
    /// Create a new header with current timestamp
    pub fn new(seq: u64, frame_id: String) -> Self {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or(Duration::ZERO)
            .as_nanos() as u64;

        Header {
            seq,
            stamp,
            frame_id,
        }
    }

    /// Create a header with default values
    pub fn default_with_seq(seq: u64) -> Self {
        Self::new(seq, String::new())
    }
}

/// Generic stamped message wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Stamped<T> {
    pub header: Header,
    pub data: T,
}

impl<T> Stamped<T> {
    /// Create a new stamped message
    pub fn new(data: T, seq: u64, frame_id: String) -> Self {
        Stamped {
            header: Header::new(seq, frame_id),
            data,
        }
    }

    /// Create a stamped message with default header
    pub fn with_seq(data: T, seq: u64) -> Self {
        Stamped {
            header: Header::default_with_seq(seq),
            data,
        }
    }
}

// Example built-in message types

/// Simple string message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StringMsg {
    pub data: String,
}

/// Simple integer message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Int32Msg {
    pub data: i32,
}

/// Simple float message  
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Float64Msg {
    pub data: f64,
}

/// Boolean message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoolMsg {
    pub data: bool,
}

/// Empty message for triggers/events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmptyMsg;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_creation() {
        let header = Header::new(42, "base_link".to_string());
        assert_eq!(header.seq, 42);
        assert_eq!(header.frame_id, "base_link");
        assert!(header.stamp > 0);
    }

    #[test]
    fn test_stamped_message() {
        let msg = Stamped::new("Hello, World!".to_string(), 1, "test".to_string());
        assert_eq!(msg.data, "Hello, World!");
        assert_eq!(msg.header.seq, 1);
        assert_eq!(msg.header.frame_id, "test");
    }

    #[test]
    fn test_built_in_messages() {
        let string_msg = StringMsg { data: "test".to_string() };
        let int_msg = Int32Msg { data: 42 };
        let float_msg = Float64Msg { data: 3.14 };
        let bool_msg = BoolMsg { data: true };
        let empty_msg = EmptyMsg;

        // Test serialization
        let _json = serde_json::to_string(&string_msg).unwrap();
        let _json = serde_json::to_string(&int_msg).unwrap();
        let _json = serde_json::to_string(&float_msg).unwrap();
        let _json = serde_json::to_string(&bool_msg).unwrap();
        let _json = serde_json::to_string(&empty_msg).unwrap();
    }
}