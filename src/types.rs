//! Unified type system for miniROS
//!
//! Provides type-safe message definitions and cross-language serialization
//! compatible with both Rust and Python interfaces. Organized into ROS2-compatible packages.

use crate::error::{MiniRosError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Core message trait that all miniROS messages must implement
pub trait MiniRosMessage: Send + Sync + Clone + Serialize + for<'de> Deserialize<'de> {
    /// Get the message type name for type checking
    fn message_type() -> &'static str;

    /// Get the message schema for validation
    fn schema() -> MessageSchema;

    /// Validate message contents
    fn validate(&self) -> Result<()> {
        Ok(()) // Default implementation - no validation
    }

    /// Serialize to bytes using efficient binary format
    fn to_bytes(&self) -> Result<Vec<u8>> {
        bincode::serialize(self)
            .map_err(|e| MiniRosError::Custom(format!("Serialization failed: {}", e)))
    }

    /// Deserialize from bytes
    fn from_bytes(data: &[u8]) -> Result<Self> {
        bincode::deserialize(data)
            .map_err(|e| MiniRosError::Custom(format!("Deserialization failed: {}", e)))
    }
}

/// Message schema for runtime type checking and validation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageSchema {
    pub name: String,
    pub fields: Vec<MessageField>,
    pub version: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageField {
    pub name: String,
    pub field_type: FieldType,
    pub required: bool,
    pub description: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FieldType {
    String,
    Int32,
    Int64,
    Uint32,
    Uint8,
    Float32,
    Float64,
    Bool,
    Bytes,
    Array(Box<FieldType>),
    FixedArray(Box<FieldType>, usize),
    Struct(String), // Reference to another message type
}

// ============================================================================
// std_msgs package - Standard message types
// ============================================================================
pub mod std_msgs {
    use super::*;

    /// String message type
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct String {
        pub data: std::string::String,
    }

    impl MiniRosMessage for String {
        fn message_type() -> &'static str {
            "std_msgs/String"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/String".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::String,
                    required: true,
                    description: Some("String data content".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if self.data.len() > 1024 * 1024 {
                return Err(MiniRosError::Custom("String too large (>1MB)".to_string()));
            }
            Ok(())
        }
    }

    /// 32-bit integer message
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Int32 {
        pub data: i32,
    }

    impl MiniRosMessage for Int32 {
        fn message_type() -> &'static str {
            "std_msgs/Int32"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Int32".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::Int32,
                    required: true,
                    description: Some("32-bit signed integer".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }
    }

    /// 64-bit integer message
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Int64 {
        pub data: i64,
    }

    impl MiniRosMessage for Int64 {
        fn message_type() -> &'static str {
            "std_msgs/Int64"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Int64".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::Int64,
                    required: true,
                    description: Some("64-bit signed integer".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }
    }

    /// 32-bit float message
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Float32 {
        pub data: f32,
    }

    impl MiniRosMessage for Float32 {
        fn message_type() -> &'static str {
            "std_msgs/Float32"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Float32".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::Float32,
                    required: true,
                    description: Some("32-bit floating point number".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }
    }

    /// 64-bit float message
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Float64 {
        pub data: f64,
    }

    impl MiniRosMessage for Float64 {
        fn message_type() -> &'static str {
            "std_msgs/Float64"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Float64".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::Float64,
                    required: true,
                    description: Some("64-bit floating point number".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }
    }

    /// Boolean message
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Bool {
        pub data: bool,
    }

    impl MiniRosMessage for Bool {
        fn message_type() -> &'static str {
            "std_msgs/Bool"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Bool".to_string(),
                fields: vec![MessageField {
                    name: "data".to_string(),
                    field_type: FieldType::Bool,
                    required: true,
                    description: Some("Boolean value".to_string()),
                }],
                version: "1.0".to_string(),
            }
        }
    }

    /// Empty message for triggers and events
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Empty {}

    impl MiniRosMessage for Empty {
        fn message_type() -> &'static str {
            "std_msgs/Empty"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Empty".to_string(),
                fields: vec![],
                version: "1.0".to_string(),
            }
        }
    }

    /// Standard header with timestamp and frame info
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Header {
        pub stamp: i64,    // nanoseconds since epoch
        pub frame_id: std::string::String,
    }

    impl MiniRosMessage for Header {
        fn message_type() -> &'static str {
            "std_msgs/Header"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "std_msgs/Header".to_string(),
                fields: vec![
                    MessageField {
                        name: "stamp".to_string(),
                        field_type: FieldType::Int64,
                        required: true,
                        description: Some("Timestamp in nanoseconds since Unix epoch".to_string()),
                    },
                    MessageField {
                        name: "frame_id".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Frame ID for coordinate system reference".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }
    }
}

// ============================================================================
// geometry_msgs package - Geometric message types
// ============================================================================
pub mod geometry_msgs {
    use super::*;

    /// 3D Point
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl MiniRosMessage for Point {
        fn message_type() -> &'static str {
            "geometry_msgs/Point"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/Point".to_string(),
                fields: vec![
                    MessageField {
                        name: "x".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("X coordinate".to_string()),
                    },
                    MessageField {
                        name: "y".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Y coordinate".to_string()),
                    },
                    MessageField {
                        name: "z".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Z coordinate".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() {
                return Err(MiniRosError::Custom("Point coordinates must be finite".to_string()));
            }
            Ok(())
        }
    }

    /// 3D Vector
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl MiniRosMessage for Vector3 {
        fn message_type() -> &'static str {
            "geometry_msgs/Vector3"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/Vector3".to_string(),
                fields: vec![
                    MessageField {
                        name: "x".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("X component".to_string()),
                    },
                    MessageField {
                        name: "y".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Y component".to_string()),
                    },
                    MessageField {
                        name: "z".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Z component".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() {
                return Err(MiniRosError::Custom("Vector3 components must be finite".to_string()));
            }
            Ok(())
        }
    }

    /// Quaternion for orientation representation
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    impl MiniRosMessage for Quaternion {
        fn message_type() -> &'static str {
            "geometry_msgs/Quaternion"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/Quaternion".to_string(),
                fields: vec![
                    MessageField {
                        name: "x".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("X component of quaternion".to_string()),
                    },
                    MessageField {
                        name: "y".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Y component of quaternion".to_string()),
                    },
                    MessageField {
                        name: "z".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("Z component of quaternion".to_string()),
                    },
                    MessageField {
                        name: "w".to_string(),
                        field_type: FieldType::Float64,
                        required: true,
                        description: Some("W component of quaternion".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check if components are finite
            if !self.x.is_finite() || !self.y.is_finite() || !self.z.is_finite() || !self.w.is_finite() {
                return Err(MiniRosError::Custom("Quaternion components must be finite".to_string()));
            }

            // Check normalization
            let norm = (self.x.powi(2) + self.y.powi(2) + self.z.powi(2) + self.w.powi(2)).sqrt();
            if (norm - 1.0).abs() > 0.1 {
                return Err(MiniRosError::Custom("Quaternion must be normalized".to_string()));
            }

            Ok(())
        }
    }

    /// Pose (position + orientation)
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }

    impl MiniRosMessage for Pose {
        fn message_type() -> &'static str {
            "geometry_msgs/Pose"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/Pose".to_string(),
                fields: vec![
                    MessageField {
                        name: "position".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Point".to_string()),
                        required: true,
                        description: Some("Position component".to_string()),
                    },
                    MessageField {
                        name: "orientation".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Quaternion".to_string()),
                        required: true,
                        description: Some("Orientation component".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.position.validate()?;
            self.orientation.validate()?;
            Ok(())
        }
    }

    /// Pose with timestamp and frame
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct PoseStamped {
        pub header: crate::types::std_msgs::Header,
        pub pose: Pose,
    }

    impl MiniRosMessage for PoseStamped {
        fn message_type() -> &'static str {
            "geometry_msgs/PoseStamped"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/PoseStamped".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "pose".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Pose".to_string()),
                        required: true,
                        description: Some("Pose in the specified frame".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.pose.validate()?;
            Ok(())
        }
    }

    /// Velocity (linear + angular)
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    impl MiniRosMessage for Twist {
        fn message_type() -> &'static str {
            "geometry_msgs/Twist"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/Twist".to_string(),
                fields: vec![
                    MessageField {
                        name: "linear".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Vector3".to_string()),
                        required: true,
                        description: Some("Linear velocity (m/s)".to_string()),
                    },
                    MessageField {
                        name: "angular".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Vector3".to_string()),
                        required: true,
                        description: Some("Angular velocity (rad/s)".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.linear.validate()?;
            self.angular.validate()?;

            // Safety limits for typical robots
            const MAX_LINEAR_VEL: f64 = 2.0;  // 2 m/s max
            const MAX_ANGULAR_VEL: f64 = 4.0; // 4 rad/s max

            if self.linear.x.abs() > MAX_LINEAR_VEL 
                || self.linear.y.abs() > MAX_LINEAR_VEL 
                || self.linear.z.abs() > MAX_LINEAR_VEL {
                return Err(MiniRosError::Custom(format!(
                    "Linear velocity too high (max: {} m/s)", MAX_LINEAR_VEL
                )));
            }

            if self.angular.x.abs() > MAX_ANGULAR_VEL 
                || self.angular.y.abs() > MAX_ANGULAR_VEL 
                || self.angular.z.abs() > MAX_ANGULAR_VEL {
                return Err(MiniRosError::Custom(format!(
                    "Angular velocity too high (max: {} rad/s)", MAX_ANGULAR_VEL
                )));
            }

            Ok(())
        }
    }

    /// Pose with covariance matrix
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct PoseWithCovariance {
        pub pose: Pose,
        pub covariance: Vec<f64>, // 6x6 covariance matrix (36 elements)
    }

    impl MiniRosMessage for PoseWithCovariance {
        fn message_type() -> &'static str {
            "geometry_msgs/PoseWithCovariance"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/PoseWithCovariance".to_string(),
                fields: vec![
                    MessageField {
                        name: "pose".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Pose".to_string()),
                        required: true,
                        description: Some("Pose estimate".to_string()),
                    },
                    MessageField {
                        name: "covariance".to_string(),
                        field_type: FieldType::FixedArray(Box::new(FieldType::Float64), 36),
                        required: true,
                        description: Some("Row-major representation of 6x6 covariance matrix".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.pose.validate()?;
            // Check covariance matrix size and values
            if self.covariance.len() != 36 {
                return Err(MiniRosError::Custom("Covariance matrix must have exactly 36 elements (6x6)".to_string()));
            }
            for &val in &self.covariance {
                if !val.is_finite() {
                    return Err(MiniRosError::Custom("Covariance values must be finite".to_string()));
                }
            }
            Ok(())
        }
    }

    /// Twist with covariance matrix
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct TwistWithCovariance {
        pub twist: Twist,
        pub covariance: Vec<f64>, // 6x6 covariance matrix (36 elements)
    }

    impl MiniRosMessage for TwistWithCovariance {
        fn message_type() -> &'static str {
            "geometry_msgs/TwistWithCovariance"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "geometry_msgs/TwistWithCovariance".to_string(),
                fields: vec![
                    MessageField {
                        name: "twist".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Twist".to_string()),
                        required: true,
                        description: Some("Velocity estimate".to_string()),
                    },
                    MessageField {
                        name: "covariance".to_string(),
                        field_type: FieldType::FixedArray(Box::new(FieldType::Float64), 36),
                        required: true,
                        description: Some("Row-major representation of 6x6 covariance matrix".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.twist.validate()?;
            // Check covariance matrix size and values
            if self.covariance.len() != 36 {
                return Err(MiniRosError::Custom("Covariance matrix must have exactly 36 elements (6x6)".to_string()));
            }
            for &val in &self.covariance {
                if !val.is_finite() {
                    return Err(MiniRosError::Custom("Covariance values must be finite".to_string()));
                }
            }
            Ok(())
        }
    }
}

// ============================================================================
// nav_msgs package - Navigation message types
// ============================================================================
pub mod nav_msgs {
    use super::*;

    /// Odometry message for robot pose and velocity feedback
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Odometry {
        pub header: crate::types::std_msgs::Header,
        pub child_frame_id: std::string::String,
        pub pose: crate::types::geometry_msgs::PoseWithCovariance,
        pub twist: crate::types::geometry_msgs::TwistWithCovariance,
    }

    impl MiniRosMessage for Odometry {
        fn message_type() -> &'static str {
            "nav_msgs/Odometry"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "nav_msgs/Odometry".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "child_frame_id".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Frame id of the child frame (usually base_link)".to_string()),
                    },
                    MessageField {
                        name: "pose".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/PoseWithCovariance".to_string()),
                        required: true,
                        description: Some("Pose of the robot with uncertainty".to_string()),
                    },
                    MessageField {
                        name: "twist".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/TwistWithCovariance".to_string()),
                        required: true,
                        description: Some("Velocity of the robot with uncertainty".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            self.pose.validate()?;
            self.twist.validate()?;
            Ok(())
        }
    }

    /// Navigation path as sequence of poses
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Path {
        pub header: crate::types::std_msgs::Header,
        pub poses: Vec<crate::types::geometry_msgs::PoseStamped>,
    }

    impl MiniRosMessage for Path {
        fn message_type() -> &'static str {
            "nav_msgs/Path"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "nav_msgs/Path".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "poses".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Struct("geometry_msgs/PoseStamped".to_string()))),
                        required: true,
                        description: Some("Path as sequence of poses".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if self.poses.len() > 10000 {
                return Err(MiniRosError::Custom("Path too long (>10000 poses)".to_string()));
            }
            for pose in &self.poses {
                pose.validate()?;
            }
            Ok(())
        }
    }
}

// ============================================================================
// Legacy compatibility types
// ============================================================================

/// Legacy string message - redirects to std_msgs::String for compatibility
pub type StringMessage = std_msgs::String;
/// Legacy int32 message - redirects to std_msgs::Int32 for compatibility  
pub type Int32Message = std_msgs::Int32;
/// Legacy int64 message - redirects to std_msgs::Int64 for compatibility
pub type Int64Message = std_msgs::Int64;
/// Legacy float32 message - redirects to std_msgs::Float32 for compatibility
pub type Float32Message = std_msgs::Float32;
/// Legacy float64 message - redirects to std_msgs::Float64 for compatibility
pub type Float64Message = std_msgs::Float64;
/// Legacy bool message - redirects to std_msgs::Bool for compatibility
pub type BoolMessage = std_msgs::Bool;

/// Legacy pose message - use geometry_msgs::PoseStamped instead
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseMessage {
    pub position: geometry_msgs::Point,
    pub orientation: geometry_msgs::Quaternion,
    pub header: std_msgs::Header,
}

impl MiniRosMessage for PoseMessage {
    fn message_type() -> &'static str {
        "legacy/Pose"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "legacy/Pose".to_string(),
            fields: vec![
                MessageField {
                    name: "position".to_string(),
                    field_type: FieldType::Struct("geometry_msgs/Point".to_string()),
                    required: true,
                    description: Some("3D position".to_string()),
                },
                MessageField {
                    name: "orientation".to_string(),
                    field_type: FieldType::Struct("geometry_msgs/Quaternion".to_string()),
                    required: true,
                    description: Some("Orientation as quaternion".to_string()),
                },
                MessageField {
                    name: "header".to_string(),
                    field_type: FieldType::Struct("std_msgs/Header".to_string()),
                    required: true,
                    description: Some("Message header".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        self.position.validate()?;
        self.orientation.validate()?;
        Ok(())
    }
}

/// Legacy twist message - use geometry_msgs::Twist instead
pub type TwistMessage = geometry_msgs::Twist;

/// Legacy odometry message - use nav_msgs::Odometry instead
pub type OdometryMessage = nav_msgs::Odometry;

// Re-export common types for backward compatibility and convenience
pub type Header = std_msgs::Header;
pub type Point3D = geometry_msgs::Point;
pub type Vector3 = geometry_msgs::Vector3;
pub type Quaternion = geometry_msgs::Quaternion;

/// Raw bytes message (no standard ROS2 equivalent, miniROS-specific)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BytesMessage {
    pub data: Vec<u8>,
}

impl MiniRosMessage for BytesMessage {
    fn message_type() -> &'static str {
        "miniROS/Bytes"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "miniROS/Bytes".to_string(),
            fields: vec![MessageField {
                name: "data".to_string(),
                field_type: FieldType::Bytes,
                required: true,
                description: Some("Raw byte data".to_string()),
            }],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        if self.data.len() > 10 * 1024 * 1024 {
            return Err(MiniRosError::Custom("Bytes too large (>10MB)".to_string()));
        }
        Ok(())
    }
}

/// 3D Point Cloud message (simplified version for miniROS)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloudMessage {
    pub points: Vec<geometry_msgs::Point>,
    pub header: std_msgs::Header,
}

impl MiniRosMessage for PointCloudMessage {
    fn message_type() -> &'static str {
        "miniROS/PointCloud"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "miniROS/PointCloud".to_string(),
            fields: vec![
                MessageField {
                    name: "points".to_string(),
                    field_type: FieldType::Array(Box::new(FieldType::Struct("geometry_msgs/Point".to_string()))),
                    required: true,
                    description: Some("Array of 3D points".to_string()),
                },
                MessageField {
                    name: "header".to_string(),
                    field_type: FieldType::Struct("std_msgs/Header".to_string()),
                    required: true,
                    description: Some("Message header with timestamp and frame".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        if self.points.len() > 1_000_000 {
            return Err(MiniRosError::Custom("Too many points (>1M)".to_string()));
        }

        for point in &self.points {
            point.validate()?;
        }

        Ok(())
    }
}

/// Registry for all message types in the system
pub struct TypeRegistry {
    schemas: HashMap<String, MessageSchema>,
}

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TypeRegistry {
    pub fn new() -> Self {
        let mut registry = Self {
            schemas: HashMap::new(),
        };

        // Register built-in types
        registry.register_builtin_types();
        registry
    }

    /// Register a new message type
    pub fn register<T: MiniRosMessage>(&mut self) {
        let schema = T::schema();
        self.schemas.insert(T::message_type().to_string(), schema);
    }

    /// Get schema for a message type
    pub fn get_schema(&self, message_type: &str) -> Option<&MessageSchema> {
        self.schemas.get(message_type)
    }

    /// Validate message against its schema
    pub fn validate_message(&self, message_type: &str, data: &[u8]) -> Result<()> {
        let _schema = self.get_schema(message_type).ok_or_else(|| {
            MiniRosError::Custom(format!("Unknown message type: {}", message_type))
        })?;

        // Basic validation - check if data can be deserialized
        match message_type {
            // std_msgs
            "std_msgs/String" => {
                let _: std_msgs::String = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid String message: {}", e)))?;
            }
            "std_msgs/Int32" => {
                let _: std_msgs::Int32 = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Int32 message: {}", e)))?;
            }
            "std_msgs/Float64" => {
                let _: std_msgs::Float64 = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Float64 message: {}", e)))?;
            }
            "std_msgs/Bool" => {
                let _: std_msgs::Bool = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Bool message: {}", e)))?;
            }
            // geometry_msgs
            "geometry_msgs/Twist" => {
                let _: geometry_msgs::Twist = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Twist message: {}", e)))?;
            }
            "geometry_msgs/Pose" => {
                let _: geometry_msgs::Pose = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Pose message: {}", e)))?;
            }
            // nav_msgs  
            "nav_msgs/Odometry" => {
                let _: nav_msgs::Odometry = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Odometry message: {}", e)))?;
            }
            // Legacy support
            "String" => {
                let _: StringMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid String message: {}", e)))?;
            }
            "Twist" => {
                let _: TwistMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Twist message: {}", e)))?;
            }
            "Odometry" => {
                let _: OdometryMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Odometry message: {}", e)))?;
            }
            _ => {
                return Err(MiniRosError::Custom(format!(
                    "Validation not implemented for type: {}",
                    message_type
                )));
            }
        }

        Ok(())
    }

    /// Register all built-in message types
    fn register_builtin_types(&mut self) {
        // std_msgs
        self.register::<std_msgs::String>();
        self.register::<std_msgs::Int32>();
        self.register::<std_msgs::Int64>();
        self.register::<std_msgs::Float32>();
        self.register::<std_msgs::Float64>();
        self.register::<std_msgs::Bool>();
        self.register::<std_msgs::Empty>();
        self.register::<std_msgs::Header>();

        // geometry_msgs
        self.register::<geometry_msgs::Point>();
        self.register::<geometry_msgs::Vector3>();
        self.register::<geometry_msgs::Quaternion>();
        self.register::<geometry_msgs::Pose>();
        self.register::<geometry_msgs::PoseStamped>();
        self.register::<geometry_msgs::Twist>();
        self.register::<geometry_msgs::PoseWithCovariance>();
        self.register::<geometry_msgs::TwistWithCovariance>();

        // nav_msgs
        self.register::<nav_msgs::Odometry>();
        self.register::<nav_msgs::Path>();

        // miniROS-specific
        self.register::<BytesMessage>();
        self.register::<PointCloudMessage>();

        // Legacy compatibility
        self.register::<PoseMessage>();
    }
}

// Create a global type registry instance
lazy_static::lazy_static! {
    pub static ref GLOBAL_TYPE_REGISTRY: std::sync::Mutex<TypeRegistry> =
        std::sync::Mutex::new(TypeRegistry::new());
}

/// Helper function to get global type registry
pub fn get_type_registry() -> std::sync::MutexGuard<'static, TypeRegistry> {
    GLOBAL_TYPE_REGISTRY.lock().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_std_msgs_string_serialization() {
        let msg = std_msgs::String {
            data: "Hello, miniROS!".to_string(),
        };

        let bytes = msg.to_bytes().unwrap();
        let restored = std_msgs::String::from_bytes(&bytes).unwrap();

        assert_eq!(msg.data, restored.data);
    }

    #[test]
    fn test_geometry_msgs_point_validation() {
        let point = geometry_msgs::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert!(point.validate().is_ok());

        // Test invalid point with NaN
        let bad_point = geometry_msgs::Point {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
        };

        assert!(bad_point.validate().is_err());
    }

    #[test]
    fn test_geometry_msgs_quaternion_validation() {
        // Valid normalized quaternion
        let quat = geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };

        assert!(quat.validate().is_ok());

        // Invalid non-normalized quaternion
        let bad_quat = geometry_msgs::Quaternion {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            w: 1.0,
        };

        assert!(bad_quat.validate().is_err());
    }

    #[test]
    fn test_geometry_msgs_twist_validation() {
        let twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 { x: 0.5, y: 0.0, z: 0.0 },
            angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 1.0 },
        };

        assert!(twist.validate().is_ok());

        // Test velocity limits
        let bad_twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 { x: 10.0, y: 0.0, z: 0.0 }, // Too fast
            angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        };

        assert!(bad_twist.validate().is_err());
    }

    #[test]
    fn test_type_registry() {
        let registry = TypeRegistry::new();

        // Test that built-in types are registered
        assert!(registry.get_schema("std_msgs/String").is_some());
        assert!(registry.get_schema("geometry_msgs/Point").is_some());
        assert!(registry.get_schema("nav_msgs/Odometry").is_some());
        assert!(registry.get_schema("NonexistentType").is_none());
    }

    #[test]
    fn test_legacy_compatibility() {
        // Test legacy type aliases work
        let legacy_string = StringMessage {
            data: "test".to_string(),
        };

        let std_string = std_msgs::String {
            data: "test".to_string(),
        };

        // Should be the same type (StringMessage is a type alias for std_msgs::String)
        assert_eq!(legacy_string.data, std_string.data);
        assert_eq!(StringMessage::message_type(), std_msgs::String::message_type());
    }
}
