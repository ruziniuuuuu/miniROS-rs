//! Message types and serialization for MiniROS
//!
//! This module provides both the generic Message trait and re-exports
//! the structured message packages (std_msgs, geometry_msgs, nav_msgs).

use serde::{Deserialize, Serialize};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// Re-export the structured message packages
pub use crate::types::{geometry_msgs, nav_msgs, std_msgs};

/// Trait for all MiniROS messages
///
/// Messages must be serializable and deserializable for network transport
pub trait Message: Send + Sync + Serialize + for<'de> Deserialize<'de> + Clone + 'static {}

// Blanket implementation for all types that satisfy the requirements
impl<T> Message for T where T: Send + Sync + Serialize + for<'de> Deserialize<'de> + Clone + 'static {}

/// Standard message header containing metadata
///
/// Note: This is the legacy header format. For new code, use std_msgs::Header instead.
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

    /// Convert to standard ROS2 header format
    pub fn to_std_header(&self) -> std_msgs::Header {
        std_msgs::Header {
            stamp: self.stamp as i64,
            frame_id: self.frame_id.clone(),
        }
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

// =============================================================================
// Legacy message types for backward compatibility
// =============================================================================

/// Simple string message - use std_msgs::String for new code
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StringMsg {
    pub data: String,
}

/// Simple integer message - use std_msgs::Int32 for new code
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Int32Msg {
    pub data: i32,
}

/// Simple float message - use std_msgs::Float64 for new code
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Float64Msg {
    pub data: f64,
}

/// Boolean message - use std_msgs::Bool for new code
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoolMsg {
    pub data: bool,
}

/// Empty message for triggers/events - use std_msgs::Empty for new code
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmptyMsg;

// =============================================================================
// Convenience re-exports for common types
// =============================================================================

// Re-export common message types for convenience
pub use geometry_msgs::{Point, Pose, PoseStamped, Quaternion, Twist, Vector3};
pub use nav_msgs::{Odometry, Path};
pub use std_msgs::{
    Bool as StdBool, Empty as StdEmpty, Float64 as StdFloat64, Int32 as StdInt32,
    String as StdString,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::MiniRosMessage;

    #[test]
    fn test_header_creation() {
        let header = Header::new(42, "base_link".to_string());
        assert_eq!(header.seq, 42);
        assert_eq!(header.frame_id, "base_link");
        assert!(header.stamp > 0);
    }

    #[test]
    fn test_header_conversion() {
        let legacy_header = Header::new(42, "base_link".to_string());
        let std_header = legacy_header.to_std_header();
        assert_eq!(std_header.frame_id, "base_link");
        assert_eq!(std_header.stamp, legacy_header.stamp as i64);
    }

    #[test]
    fn test_stamped_message() {
        let msg = Stamped::new("Hello, World!".to_string(), 1, "test".to_string());
        assert_eq!(msg.data, "Hello, World!");
        assert_eq!(msg.header.seq, 1);
        assert_eq!(msg.header.frame_id, "test");
    }

    #[test]
    fn test_legacy_messages() {
        let string_msg = StringMsg {
            data: "test".to_string(),
        };
        let int_msg = Int32Msg { data: 42 };
        let float_msg = Float64Msg {
            data: std::f64::consts::PI,
        };
        let bool_msg = BoolMsg { data: true };
        let empty_msg = EmptyMsg;

        // Test serialization
        let _json = serde_json::to_string(&string_msg).unwrap();
        let _json = serde_json::to_string(&int_msg).unwrap();
        let _json = serde_json::to_string(&float_msg).unwrap();
        let _json = serde_json::to_string(&bool_msg).unwrap();
        let _json = serde_json::to_string(&empty_msg).unwrap();
    }

    #[test]
    fn test_new_std_messages() {
        let string_msg = std_msgs::String {
            data: "Hello miniROS!".to_string(),
        };

        let point = geometry_msgs::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        let twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        };

        // Test message validation
        assert!(string_msg.validate().is_ok());
        assert!(point.validate().is_ok());
        assert!(twist.validate().is_ok());

        // Test serialization
        assert!(string_msg.to_bytes().is_ok());
        assert!(point.to_bytes().is_ok());
        assert!(twist.to_bytes().is_ok());
    }

    #[test]
    fn test_convenience_exports() {
        // Test that convenience re-exports work
        let _string = StdString {
            data: "test".to_string(),
        };
        let _int = StdInt32 { data: 42 };
        let _point = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let _twist = Twist {
            linear: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };
    }
}
