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
            "nav_msgs/Path" => {
                let _: nav_msgs::Path = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Path message: {}", e)))?;
            }
            // sensor_msgs
            "sensor_msgs/LaserScan" => {
                let _: sensor_msgs::LaserScan = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid LaserScan message: {}", e)))?;
            }
            "sensor_msgs/PointCloud2" => {
                let _: sensor_msgs::PointCloud2 = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid PointCloud2 message: {}", e)))?;
            }
            "sensor_msgs/Imu" => {
                let _: sensor_msgs::Imu = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Imu message: {}", e)))?;
            }
            "sensor_msgs/Image" => {
                let _: sensor_msgs::Image = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Image message: {}", e)))?;
            }
            // action_msgs
            "action_msgs/GoalStatus" => {
                let _: action_msgs::GoalStatus = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid GoalStatus message: {}", e)))?;
            }
            // diagnostic_msgs
            "diagnostic_msgs/DiagnosticArray" => {
                let _: diagnostic_msgs::DiagnosticArray = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid DiagnosticArray message: {}", e)))?;
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

        // sensor_msgs
        self.register::<sensor_msgs::LaserScan>();
        self.register::<sensor_msgs::PointField>();
        self.register::<sensor_msgs::PointCloud2>();
        self.register::<sensor_msgs::Imu>();
        self.register::<sensor_msgs::Image>();

        // action_msgs
        self.register::<action_msgs::GoalInfo>();
        self.register::<action_msgs::GoalStatus>();
        self.register::<action_msgs::GoalStatusArray>();

        // diagnostic_msgs
        self.register::<diagnostic_msgs::KeyValue>();
        self.register::<diagnostic_msgs::DiagnosticStatus>();
        self.register::<diagnostic_msgs::DiagnosticArray>();

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
    fn test_sensor_msgs_laser_scan_validation() {
        let laser_scan = sensor_msgs::LaserScan {
            header: std_msgs::Header {
                stamp: 1234567890,
                frame_id: "laser".to_string(),
            },
            angle_min: -1.57,
            angle_max: 1.57,
            angle_increment: 0.01,
            time_increment: 0.001,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges: vec![1.0, 2.0, 3.0],
            intensities: vec![],
        };

        assert!(laser_scan.validate().is_ok());

        // Test invalid angle range
        let bad_scan = sensor_msgs::LaserScan {
            angle_min: 1.57,
            angle_max: -1.57, // Invalid: min > max
            ..laser_scan.clone()
        };

        assert!(bad_scan.validate().is_err());
    }

    #[test]
    fn test_sensor_msgs_imu_validation() {
        let imu = sensor_msgs::Imu {
            header: std_msgs::Header {
                stamp: 1234567890,
                frame_id: "imu".to_string(),
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            orientation_covariance: vec![0.0; 9],
            angular_velocity: geometry_msgs::Vector3 {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            },
            angular_velocity_covariance: vec![0.0; 9],
            linear_acceleration: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 9.8,
            },
            linear_acceleration_covariance: vec![0.0; 9],
        };

        assert!(imu.validate().is_ok());

        // Test invalid covariance matrix size
        let bad_imu = sensor_msgs::Imu {
            orientation_covariance: vec![0.0; 8], // Should be 9 elements
            ..imu.clone()
        };

        assert!(bad_imu.validate().is_err());
    }

    #[test]
    fn test_action_msgs_goal_status_validation() {
        use action_msgs::*;

        let goal_status = GoalStatus {
            goal_info: GoalInfo {
                goal_id: "test_goal".to_string(),
                stamp: 1234567890,
            },
            status: STATUS_EXECUTING,
        };

        assert!(goal_status.validate().is_ok());

        // Test invalid status code
        let bad_status = GoalStatus {
            status: 99, // Invalid status code
            ..goal_status.clone()
        };

        assert!(bad_status.validate().is_err());
    }

    #[test]
    fn test_diagnostic_msgs_validation() {
        use diagnostic_msgs::*;

        let diagnostic = DiagnosticStatus {
            level: OK,
            name: "Test Component".to_string(),
            message: "All good".to_string(),
            hardware_id: "hw_001".to_string(),
            values: vec![KeyValue {
                key: "temperature".to_string(),
                value: "25.0".to_string(),
            }],
        };

        assert!(diagnostic.validate().is_ok());

        // Test invalid diagnostic level
        let bad_diagnostic = DiagnosticStatus {
            level: 99, // Invalid level
            ..diagnostic.clone()
        };

        assert!(bad_diagnostic.validate().is_err());
    }

    #[test]
    fn test_type_registry() {
        let registry = TypeRegistry::new();

        // Test that built-in types are registered
        assert!(registry.get_schema("std_msgs/String").is_some());
        assert!(registry.get_schema("geometry_msgs/Point").is_some());
        assert!(registry.get_schema("nav_msgs/Odometry").is_some());
        assert!(registry.get_schema("sensor_msgs/LaserScan").is_some());
        assert!(registry.get_schema("action_msgs/GoalStatus").is_some());
        assert!(registry.get_schema("diagnostic_msgs/DiagnosticArray").is_some());
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

    #[test]
    fn test_message_serialization_roundtrip() {
        // Test various message types for serialization/deserialization
        
        // LaserScan
        let laser_scan = sensor_msgs::LaserScan {
            header: std_msgs::Header {
                stamp: 1234567890,
                frame_id: "laser".to_string(),
            },
            angle_min: -1.57,
            angle_max: 1.57,
            angle_increment: 0.01,
            time_increment: 0.001,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges: vec![1.0, 2.0, 3.0],
            intensities: vec![],
        };
        
        let bytes = laser_scan.to_bytes().unwrap();
        let restored = sensor_msgs::LaserScan::from_bytes(&bytes).unwrap();
        assert_eq!(laser_scan.ranges, restored.ranges);
        
        // DiagnosticArray
        let diagnostic_array = diagnostic_msgs::DiagnosticArray {
            header: std_msgs::Header {
                stamp: 1234567890,
                frame_id: "".to_string(),
            },
            status: vec![diagnostic_msgs::DiagnosticStatus {
                level: diagnostic_msgs::OK,
                name: "Test".to_string(),
                message: "OK".to_string(),
                hardware_id: "hw_001".to_string(),
                values: vec![],
            }],
        };
        
        let bytes = diagnostic_array.to_bytes().unwrap();
        let restored = diagnostic_msgs::DiagnosticArray::from_bytes(&bytes).unwrap();
        assert_eq!(diagnostic_array.status.len(), restored.status.len());
    }
}

// ============================================================================
// sensor_msgs package - Sensor message types
// ============================================================================
pub mod sensor_msgs {
    use super::*;

    /// Laser range finder scan data - single scan from a planar laser scanner
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct LaserScan {
        pub header: crate::types::std_msgs::Header,
        pub angle_min: f32,        // Start angle of the scan [rad]
        pub angle_max: f32,        // End angle of the scan [rad]
        pub angle_increment: f32,  // Angular distance between measurements [rad]
        pub time_increment: f32,   // Time between measurements [seconds]
        pub scan_time: f32,        // Time between scans [seconds]
        pub range_min: f32,        // Minimum range value [m]
        pub range_max: f32,        // Maximum range value [m]
        pub ranges: Vec<f32>,      // Range data [m] (NaN for invalid measurements)
        pub intensities: Vec<f32>, // Intensity data (optional, same length as ranges)
    }

    impl MiniRosMessage for LaserScan {
        fn message_type() -> &'static str {
            "sensor_msgs/LaserScan"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "sensor_msgs/LaserScan".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "angle_min".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Start angle of the scan [rad]".to_string()),
                    },
                    MessageField {
                        name: "angle_max".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("End angle of the scan [rad]".to_string()),
                    },
                    MessageField {
                        name: "angle_increment".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Angular distance between measurements [rad]".to_string()),
                    },
                    MessageField {
                        name: "time_increment".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Time between measurements [seconds]".to_string()),
                    },
                    MessageField {
                        name: "scan_time".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Time between scans [seconds]".to_string()),
                    },
                    MessageField {
                        name: "range_min".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Minimum range value [m]".to_string()),
                    },
                    MessageField {
                        name: "range_max".to_string(),
                        field_type: FieldType::Float32,
                        required: true,
                        description: Some("Maximum range value [m]".to_string()),
                    },
                    MessageField {
                        name: "ranges".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Float32)),
                        required: true,
                        description: Some("Range data [m] (NaN for invalid measurements)".to_string()),
                    },
                    MessageField {
                        name: "intensities".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Float32)),
                        required: false,
                        description: Some("Intensity data (optional, same length as ranges)".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check angle range
            if self.angle_min >= self.angle_max {
                return Err(MiniRosError::Custom("angle_min must be less than angle_max".to_string()));
            }

            // Check angle increment
            if self.angle_increment <= 0.0 {
                return Err(MiniRosError::Custom("angle_increment must be positive".to_string()));
            }

            // Check time values
            if self.time_increment < 0.0 || self.scan_time <= 0.0 {
                return Err(MiniRosError::Custom("time values must be positive".to_string()));
            }

            // Check range limits
            if self.range_min < 0.0 || self.range_max <= self.range_min {
                return Err(MiniRosError::Custom("Invalid range limits".to_string()));
            }

            // Check data size limits
            if self.ranges.len() > 10000 {
                return Err(MiniRosError::Custom("Too many range measurements (>10000)".to_string()));
            }

            // If intensities exist, check they match ranges
            if !self.intensities.is_empty() && self.intensities.len() != self.ranges.len() {
                return Err(MiniRosError::Custom("Intensities length must match ranges length".to_string()));
            }

            Ok(())
        }
    }

    /// Point field descriptor for PointCloud2
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct PointField {
        pub name: std::string::String,  // Name of field
        pub offset: u32,                // Offset from start of point struct
        pub datatype: u8,               // Datatype enumeration (see constants)
        pub count: u32,                 // How many elements in the field
    }

    impl MiniRosMessage for PointField {
        fn message_type() -> &'static str {
            "sensor_msgs/PointField"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "sensor_msgs/PointField".to_string(),
                fields: vec![
                    MessageField {
                        name: "name".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Name of field".to_string()),
                    },
                    MessageField {
                        name: "offset".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Offset from start of point struct".to_string()),
                    },
                    MessageField {
                        name: "datatype".to_string(),
                        field_type: FieldType::Uint8,
                        required: true,
                        description: Some("Datatype enumeration".to_string()),
                    },
                    MessageField {
                        name: "count".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("How many elements in the field".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }
    }

    /// Point cloud data structure for 3D sensor data
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct PointCloud2 {
        pub header: crate::types::std_msgs::Header,
        pub height: u32,                // Height of point cloud (1 for unordered)
        pub width: u32,                 // Width of point cloud (total points for unordered)
        pub fields: Vec<PointField>,    // Describes the channels and their layout
        pub is_bigendian: bool,         // Is this data bigendian?
        pub point_step: u32,            // Length of a point in bytes
        pub row_step: u32,              // Length of a row in bytes
        pub data: Vec<u8>,              // Actual point data
        pub is_dense: bool,             // True if there are no invalid points
    }

    impl MiniRosMessage for PointCloud2 {
        fn message_type() -> &'static str {
            "sensor_msgs/PointCloud2"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "sensor_msgs/PointCloud2".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "height".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Height of the point cloud".to_string()),
                    },
                    MessageField {
                        name: "width".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Width of the point cloud".to_string()),
                    },
                    MessageField {
                        name: "fields".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Struct("sensor_msgs/PointField".to_string()))),
                        required: true,
                        description: Some("Describes the channels and their layout".to_string()),
                    },
                    MessageField {
                        name: "is_bigendian".to_string(),
                        field_type: FieldType::Bool,
                        required: true,
                        description: Some("Is this data bigendian?".to_string()),
                    },
                    MessageField {
                        name: "point_step".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Length of a point in bytes".to_string()),
                    },
                    MessageField {
                        name: "row_step".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Length of a row in bytes".to_string()),
                    },
                    MessageField {
                        name: "data".to_string(),
                        field_type: FieldType::Bytes,
                        required: true,
                        description: Some("Actual point data".to_string()),
                    },
                    MessageField {
                        name: "is_dense".to_string(),
                        field_type: FieldType::Bool,
                        required: true,
                        description: Some("True if there are no invalid points".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check dimensions
            if self.height == 0 || self.width == 0 {
                return Err(MiniRosError::Custom("Point cloud dimensions must be positive".to_string()));
            }

            // Check data size limits
            let total_points = self.height * self.width;
            if total_points > 1_000_000 {
                return Err(MiniRosError::Custom("Point cloud too large (>1M points)".to_string()));
            }

            // Check step sizes
            if self.point_step == 0 || self.row_step == 0 {
                return Err(MiniRosError::Custom("Step sizes must be positive".to_string()));
            }

            // Check data size consistency
            let expected_size = (self.row_step * self.height) as usize;
            if self.data.len() != expected_size {
                return Err(MiniRosError::Custom("Data size doesn't match expected size".to_string()));
            }

            Ok(())
        }
    }

    /// Inertial Measurement Unit data
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Imu {
        pub header: crate::types::std_msgs::Header,
        pub orientation: crate::types::geometry_msgs::Quaternion,
        pub orientation_covariance: Vec<f64>,     // 3x3 covariance matrix (9 elements)
        pub angular_velocity: crate::types::geometry_msgs::Vector3,
        pub angular_velocity_covariance: Vec<f64>, // 3x3 covariance matrix (9 elements)
        pub linear_acceleration: crate::types::geometry_msgs::Vector3,
        pub linear_acceleration_covariance: Vec<f64>, // 3x3 covariance matrix (9 elements)
    }

    impl MiniRosMessage for Imu {
        fn message_type() -> &'static str {
            "sensor_msgs/Imu"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "sensor_msgs/Imu".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "orientation".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Quaternion".to_string()),
                        required: true,
                        description: Some("Orientation estimate".to_string()),
                    },
                    MessageField {
                        name: "orientation_covariance".to_string(),
                        field_type: FieldType::FixedArray(Box::new(FieldType::Float64), 9),
                        required: true,
                        description: Some("3x3 orientation covariance matrix".to_string()),
                    },
                    MessageField {
                        name: "angular_velocity".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Vector3".to_string()),
                        required: true,
                        description: Some("Angular velocity measurement".to_string()),
                    },
                    MessageField {
                        name: "angular_velocity_covariance".to_string(),
                        field_type: FieldType::FixedArray(Box::new(FieldType::Float64), 9),
                        required: true,
                        description: Some("3x3 angular velocity covariance matrix".to_string()),
                    },
                    MessageField {
                        name: "linear_acceleration".to_string(),
                        field_type: FieldType::Struct("geometry_msgs/Vector3".to_string()),
                        required: true,
                        description: Some("Linear acceleration measurement".to_string()),
                    },
                    MessageField {
                        name: "linear_acceleration_covariance".to_string(),
                        field_type: FieldType::FixedArray(Box::new(FieldType::Float64), 9),
                        required: true,
                        description: Some("3x3 linear acceleration covariance matrix".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Validate embedded messages
            self.orientation.validate()?;
            self.angular_velocity.validate()?;
            self.linear_acceleration.validate()?;

            // Check covariance matrix sizes
            if self.orientation_covariance.len() != 9 {
                return Err(MiniRosError::Custom("Orientation covariance must have 9 elements (3x3)".to_string()));
            }
            if self.angular_velocity_covariance.len() != 9 {
                return Err(MiniRosError::Custom("Angular velocity covariance must have 9 elements (3x3)".to_string()));
            }
            if self.linear_acceleration_covariance.len() != 9 {
                return Err(MiniRosError::Custom("Linear acceleration covariance must have 9 elements (3x3)".to_string()));
            }

            // Check for finite values in covariance matrices
            for &val in &self.orientation_covariance {
                if !val.is_finite() {
                    return Err(MiniRosError::Custom("Covariance values must be finite".to_string()));
                }
            }

            Ok(())
        }
    }

    /// Image sensor data
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Image {
        pub header: crate::types::std_msgs::Header,
        pub height: u32,                // Image height (pixels)
        pub width: u32,                 // Image width (pixels)
        pub encoding: std::string::String, // Encoding of pixels (mono8, rgb8, bgr8, etc.)
        pub is_bigendian: u8,           // Is data bigendian?
        pub step: u32,                  // Full row length in bytes
        pub data: Vec<u8>,              // Actual image data
    }

    impl MiniRosMessage for Image {
        fn message_type() -> &'static str {
            "sensor_msgs/Image"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "sensor_msgs/Image".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "height".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Image height (pixels)".to_string()),
                    },
                    MessageField {
                        name: "width".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Image width (pixels)".to_string()),
                    },
                    MessageField {
                        name: "encoding".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Encoding of pixels".to_string()),
                    },
                    MessageField {
                        name: "is_bigendian".to_string(),
                        field_type: FieldType::Uint8,
                        required: true,
                        description: Some("Is data bigendian?".to_string()),
                    },
                    MessageField {
                        name: "step".to_string(),
                        field_type: FieldType::Uint32,
                        required: true,
                        description: Some("Full row length in bytes".to_string()),
                    },
                    MessageField {
                        name: "data".to_string(),
                        field_type: FieldType::Bytes,
                        required: true,
                        description: Some("Actual image data".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check image dimensions
            if self.height == 0 || self.width == 0 {
                return Err(MiniRosError::Custom("Image dimensions must be positive".to_string()));
            }

            // Check reasonable size limits
            if self.height > 4096 || self.width > 4096 {
                return Err(MiniRosError::Custom("Image too large (max 4096x4096)".to_string()));
            }

            // Check step size
            if self.step == 0 {
                return Err(MiniRosError::Custom("Step size must be positive".to_string()));
            }

            // Check data size
            let expected_size = (self.step * self.height) as usize;
            if self.data.len() != expected_size {
                return Err(MiniRosError::Custom("Data size doesn't match expected size".to_string()));
            }

            // Validate encoding format
            match self.encoding.as_str() {
                "mono8" | "mono16" | "rgb8" | "rgba8" | "bgr8" | "bgra8" | 
                "rgb16" | "rgba16" | "bgr16" | "bgra16" | "8UC1" | "8UC3" | "8UC4" |
                "16UC1" | "16UC3" | "16UC4" | "32FC1" | "32FC3" | "32FC4" => Ok(()),
                _ => Err(MiniRosError::Custom(format!("Unknown image encoding: {}", self.encoding))),
            }
        }
    }
}

// ============================================================================
// action_msgs package - Action system message types  
// ============================================================================
pub mod action_msgs {
    use super::*;

    /// Goal info for action system
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct GoalInfo {
        pub goal_id: std::string::String,     // Unique identifier for this goal
        pub stamp: i64,                       // Timestamp when goal was created
    }

    impl MiniRosMessage for GoalInfo {
        fn message_type() -> &'static str {
            "action_msgs/GoalInfo"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "action_msgs/GoalInfo".to_string(),
                fields: vec![
                    MessageField {
                        name: "goal_id".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Unique identifier for this goal".to_string()),
                    },
                    MessageField {
                        name: "stamp".to_string(),
                        field_type: FieldType::Int64,
                        required: true,
                        description: Some("Timestamp when goal was created".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }
    }

    /// Goal status enumeration values
    pub const STATUS_UNKNOWN: i8 = 0;
    pub const STATUS_ACCEPTED: i8 = 1;
    pub const STATUS_EXECUTING: i8 = 2;
    pub const STATUS_CANCELING: i8 = 3;
    pub const STATUS_SUCCEEDED: i8 = 4;
    pub const STATUS_CANCELED: i8 = 5;
    pub const STATUS_ABORTED: i8 = 6;

    /// Goal status information
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct GoalStatus {
        pub goal_info: GoalInfo,
        pub status: i8,                       // Status code (see constants above)
    }

    impl MiniRosMessage for GoalStatus {
        fn message_type() -> &'static str {
            "action_msgs/GoalStatus"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "action_msgs/GoalStatus".to_string(),
                fields: vec![
                    MessageField {
                        name: "goal_info".to_string(),
                        field_type: FieldType::Struct("action_msgs/GoalInfo".to_string()),
                        required: true,
                        description: Some("Goal information".to_string()),
                    },
                    MessageField {
                        name: "status".to_string(),
                        field_type: FieldType::Int32, // Using Int32 for compatibility
                        required: true,
                        description: Some("Status code".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check valid status codes
            match self.status {
                STATUS_UNKNOWN | STATUS_ACCEPTED | STATUS_EXECUTING | 
                STATUS_CANCELING | STATUS_SUCCEEDED | STATUS_CANCELED | STATUS_ABORTED => Ok(()),
                _ => Err(MiniRosError::Custom(format!("Invalid status code: {}", self.status))),
            }
        }
    }

    /// Array of goal statuses
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct GoalStatusArray {
        pub status_list: Vec<GoalStatus>,
    }

    impl MiniRosMessage for GoalStatusArray {
        fn message_type() -> &'static str {
            "action_msgs/GoalStatusArray"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "action_msgs/GoalStatusArray".to_string(),
                fields: vec![
                    MessageField {
                        name: "status_list".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Struct("action_msgs/GoalStatus".to_string()))),
                        required: true,
                        description: Some("Array of goal statuses".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if self.status_list.len() > 1000 {
                return Err(MiniRosError::Custom("Too many goals in status array (>1000)".to_string()));
            }

            for status in &self.status_list {
                status.validate()?;
            }

            Ok(())
        }
    }
}

// ============================================================================
// diagnostic_msgs package - System diagnostic message types
// ============================================================================
pub mod diagnostic_msgs {
    use super::*;

    /// Diagnostic severity levels
    pub const OK: u8 = 0;
    pub const WARN: u8 = 1;
    pub const ERROR: u8 = 2;
    pub const STALE: u8 = 3;

    /// Key-value pair for diagnostic data
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct KeyValue {
        pub key: std::string::String,    // Parameter name
        pub value: std::string::String,  // Parameter value
    }

    impl MiniRosMessage for KeyValue {
        fn message_type() -> &'static str {
            "diagnostic_msgs/KeyValue"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "diagnostic_msgs/KeyValue".to_string(),
                fields: vec![
                    MessageField {
                        name: "key".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Parameter name".to_string()),
                    },
                    MessageField {
                        name: "value".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Parameter value".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }
    }

    /// Status information for a single diagnostic item
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct DiagnosticStatus {
        pub level: u8,                          // Severity level (see constants)
        pub name: std::string::String,          // Component name (e.g., "Motor Controller")
        pub message: std::string::String,       // Human-readable status message
        pub hardware_id: std::string::String,   // Hardware identifier
        pub values: Vec<KeyValue>,              // Key-value pairs for diagnostic data
    }

    impl MiniRosMessage for DiagnosticStatus {
        fn message_type() -> &'static str {
            "diagnostic_msgs/DiagnosticStatus"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "diagnostic_msgs/DiagnosticStatus".to_string(),
                fields: vec![
                    MessageField {
                        name: "level".to_string(),
                        field_type: FieldType::Uint8,
                        required: true,
                        description: Some("Severity level".to_string()),
                    },
                    MessageField {
                        name: "name".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Component name".to_string()),
                    },
                    MessageField {
                        name: "message".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Human-readable status message".to_string()),
                    },
                    MessageField {
                        name: "hardware_id".to_string(),
                        field_type: FieldType::String,
                        required: true,
                        description: Some("Hardware identifier".to_string()),
                    },
                    MessageField {
                        name: "values".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Struct("diagnostic_msgs/KeyValue".to_string()))),
                        required: true,
                        description: Some("Key-value pairs for diagnostic data".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            // Check valid level
            match self.level {
                OK | WARN | ERROR | STALE => Ok(()),
                _ => Err(MiniRosError::Custom(format!("Invalid diagnostic level: {}", self.level))),
            }?;

            // Check reasonable limits
            if self.values.len() > 100 {
                return Err(MiniRosError::Custom("Too many diagnostic values (>100)".to_string()));
            }

            Ok(())
        }
    }

    /// Array of diagnostic status messages
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct DiagnosticArray {
        pub header: crate::types::std_msgs::Header,
        pub status: Vec<DiagnosticStatus>,
    }

    impl MiniRosMessage for DiagnosticArray {
        fn message_type() -> &'static str {
            "diagnostic_msgs/DiagnosticArray"
        }

        fn schema() -> MessageSchema {
            MessageSchema {
                name: "diagnostic_msgs/DiagnosticArray".to_string(),
                fields: vec![
                    MessageField {
                        name: "header".to_string(),
                        field_type: FieldType::Struct("std_msgs/Header".to_string()),
                        required: true,
                        description: Some("Header with timestamp and frame info".to_string()),
                    },
                    MessageField {
                        name: "status".to_string(),
                        field_type: FieldType::Array(Box::new(FieldType::Struct("diagnostic_msgs/DiagnosticStatus".to_string()))),
                        required: true,
                        description: Some("Array of diagnostic status messages".to_string()),
                    },
                ],
                version: "1.0".to_string(),
            }
        }

        fn validate(&self) -> Result<()> {
            if self.status.len() > 100 {
                return Err(MiniRosError::Custom("Too many diagnostic statuses (>100)".to_string()));
            }

            for status in &self.status {
                status.validate()?;
            }

            Ok(())
        }
    }
}
