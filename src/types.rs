//! Unified type system for miniROS
//!
//! Provides type-safe message definitions and cross-language serialization
//! compatible with both Rust and Python interfaces.

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
    Float32,
    Float64,
    Bool,
    Bytes,
    Array(Box<FieldType>),
    Struct(String), // Reference to another message type
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
            "String" => {
                let _: StringMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid String message: {}", e)))?;
            }
            "Int32" => {
                let _: Int32Message = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Int32 message: {}", e)))?;
            }
            "Float64" => {
                let _: Float64Message = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Float64 message: {}", e)))?;
            }
            "Bool" => {
                let _: BoolMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Bool message: {}", e)))?;
            }
            "Twist" => {
                let _: TwistMessage = bincode::deserialize(data)
                    .map_err(|e| MiniRosError::Custom(format!("Invalid Twist message: {}", e)))?;
            }
            "Odometry" => {
                let _: OdometryMessage = bincode::deserialize(data).map_err(|e| {
                    MiniRosError::Custom(format!("Invalid Odometry message: {}", e))
                })?;
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
        self.register::<StringMessage>();
        self.register::<Int32Message>();
        self.register::<Int64Message>();
        self.register::<Float32Message>();
        self.register::<Float64Message>();
        self.register::<BoolMessage>();
        self.register::<BytesMessage>();
        self.register::<PointCloudMessage>();
        self.register::<PoseMessage>();
        self.register::<TwistMessage>();
        self.register::<OdometryMessage>();
    }
}

// ============================================================================
// Built-in Message Types
// ============================================================================

/// String message type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StringMessage {
    pub data: String,
}

impl MiniRosMessage for StringMessage {
    fn message_type() -> &'static str {
        "String"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "String".to_string(),
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
            // 1MB limit
            return Err(MiniRosError::Custom("String too large (>1MB)".to_string()));
        }
        Ok(())
    }
}

/// 32-bit integer message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Int32Message {
    pub data: i32,
}

impl MiniRosMessage for Int32Message {
    fn message_type() -> &'static str {
        "Int32"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Int32".to_string(),
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
pub struct Int64Message {
    pub data: i64,
}

impl MiniRosMessage for Int64Message {
    fn message_type() -> &'static str {
        "Int64"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Int64".to_string(),
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
pub struct Float32Message {
    pub data: f32,
}

impl MiniRosMessage for Float32Message {
    fn message_type() -> &'static str {
        "Float32"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Float32".to_string(),
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
pub struct Float64Message {
    pub data: f64,
}

impl MiniRosMessage for Float64Message {
    fn message_type() -> &'static str {
        "Float64"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Float64".to_string(),
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
pub struct BoolMessage {
    pub data: bool,
}

impl MiniRosMessage for BoolMessage {
    fn message_type() -> &'static str {
        "Bool"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Bool".to_string(),
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

/// Raw bytes message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BytesMessage {
    pub data: Vec<u8>,
}

impl MiniRosMessage for BytesMessage {
    fn message_type() -> &'static str {
        "Bytes"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Bytes".to_string(),
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
            // 10MB limit
            return Err(MiniRosError::Custom("Bytes too large (>10MB)".to_string()));
        }
        Ok(())
    }
}

/// 3D Point Cloud message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloudMessage {
    pub points: Vec<Point3D>,
    pub header: Header,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub timestamp: i64, // nanoseconds since epoch
    pub frame_id: String,
}

impl MiniRosMessage for PointCloudMessage {
    fn message_type() -> &'static str {
        "PointCloud"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "PointCloud".to_string(),
            fields: vec![
                MessageField {
                    name: "points".to_string(),
                    field_type: FieldType::Array(Box::new(FieldType::Struct(
                        "Point3D".to_string(),
                    ))),
                    required: true,
                    description: Some("Array of 3D points".to_string()),
                },
                MessageField {
                    name: "header".to_string(),
                    field_type: FieldType::Struct("Header".to_string()),
                    required: true,
                    description: Some("Message header with timestamp and frame".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        if self.points.len() > 1_000_000 {
            // 1M points limit
            return Err(MiniRosError::Custom("Too many points (>1M)".to_string()));
        }

        // Validate finite coordinates
        for point in &self.points {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(MiniRosError::Custom(
                    "Point coordinates must be finite".to_string(),
                ));
            }
        }

        Ok(())
    }
}

/// 3D Pose message (position + orientation)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseMessage {
    pub position: Point3D,
    pub orientation: Quaternion,
    pub header: Header,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl MiniRosMessage for PoseMessage {
    fn message_type() -> &'static str {
        "Pose"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Pose".to_string(),
            fields: vec![
                MessageField {
                    name: "position".to_string(),
                    field_type: FieldType::Struct("Point3D".to_string()),
                    required: true,
                    description: Some("3D position".to_string()),
                },
                MessageField {
                    name: "orientation".to_string(),
                    field_type: FieldType::Struct("Quaternion".to_string()),
                    required: true,
                    description: Some("Orientation as quaternion".to_string()),
                },
                MessageField {
                    name: "header".to_string(),
                    field_type: FieldType::Struct("Header".to_string()),
                    required: true,
                    description: Some("Message header".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        // Validate position
        if !self.position.x.is_finite()
            || !self.position.y.is_finite()
            || !self.position.z.is_finite()
        {
            return Err(MiniRosError::Custom(
                "Position coordinates must be finite".to_string(),
            ));
        }

        // Validate quaternion normalization
        let norm = (self.orientation.x.powi(2)
            + self.orientation.y.powi(2)
            + self.orientation.z.powi(2)
            + self.orientation.w.powi(2))
        .sqrt();

        if (norm - 1.0).abs() > 0.1 {
            // Allow some tolerance
            return Err(MiniRosError::Custom(
                "Quaternion must be normalized".to_string(),
            ));
        }

        Ok(())
    }
}

/// Velocity command message (twist) for robot motion control
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwistMessage {
    pub linear: Vector3,  // Linear velocity (m/s)
    pub angular: Vector3, // Angular velocity (rad/s)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl MiniRosMessage for TwistMessage {
    fn message_type() -> &'static str {
        "Twist"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Twist".to_string(),
            fields: vec![
                MessageField {
                    name: "linear".to_string(),
                    field_type: FieldType::Struct("Vector3".to_string()),
                    required: true,
                    description: Some("Linear velocity in m/s".to_string()),
                },
                MessageField {
                    name: "angular".to_string(),
                    field_type: FieldType::Struct("Vector3".to_string()),
                    required: true,
                    description: Some("Angular velocity in rad/s".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        // Validate linear velocity
        if !self.linear.x.is_finite() || !self.linear.y.is_finite() || !self.linear.z.is_finite() {
            return Err(MiniRosError::Custom(
                "Linear velocity components must be finite".to_string(),
            ));
        }

        // Validate angular velocity
        if !self.angular.x.is_finite() || !self.angular.y.is_finite() || !self.angular.z.is_finite()
        {
            return Err(MiniRosError::Custom(
                "Angular velocity components must be finite".to_string(),
            ));
        }

        // Safety limits for turtlebot-style robots
        const MAX_LINEAR_VEL: f32 = 2.0; // 2 m/s max
        const MAX_ANGULAR_VEL: f32 = 4.0; // 4 rad/s max

        if self.linear.x.abs() > MAX_LINEAR_VEL
            || self.linear.y.abs() > MAX_LINEAR_VEL
            || self.linear.z.abs() > MAX_LINEAR_VEL
        {
            return Err(MiniRosError::Custom(format!(
                "Linear velocity too high (max: {} m/s)",
                MAX_LINEAR_VEL
            )));
        }

        if self.angular.x.abs() > MAX_ANGULAR_VEL
            || self.angular.y.abs() > MAX_ANGULAR_VEL
            || self.angular.z.abs() > MAX_ANGULAR_VEL
        {
            return Err(MiniRosError::Custom(format!(
                "Angular velocity too high (max: {} rad/s)",
                MAX_ANGULAR_VEL
            )));
        }

        Ok(())
    }
}

/// Odometry message for robot pose and velocity feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OdometryMessage {
    pub pose: PoseMessage,
    pub twist: TwistMessage,
    pub header: Header,
}

impl MiniRosMessage for OdometryMessage {
    fn message_type() -> &'static str {
        "Odometry"
    }

    fn schema() -> MessageSchema {
        MessageSchema {
            name: "Odometry".to_string(),
            fields: vec![
                MessageField {
                    name: "pose".to_string(),
                    field_type: FieldType::Struct("Pose".to_string()),
                    required: true,
                    description: Some("Robot pose (position and orientation)".to_string()),
                },
                MessageField {
                    name: "twist".to_string(),
                    field_type: FieldType::Struct("Twist".to_string()),
                    required: true,
                    description: Some("Robot velocity (linear and angular)".to_string()),
                },
                MessageField {
                    name: "header".to_string(),
                    field_type: FieldType::Struct("Header".to_string()),
                    required: true,
                    description: Some("Message header".to_string()),
                },
            ],
            version: "1.0".to_string(),
        }
    }

    fn validate(&self) -> Result<()> {
        // Validate pose and twist components
        self.pose.validate()?;
        self.twist.validate()?;
        Ok(())
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
    fn test_string_message_serialization() {
        let msg = StringMessage {
            data: "Hello, miniROS!".to_string(),
        };

        let bytes = msg.to_bytes().unwrap();
        let restored = StringMessage::from_bytes(&bytes).unwrap();

        assert_eq!(msg.data, restored.data);
    }

    #[test]
    fn test_point_cloud_validation() {
        let msg = PointCloudMessage {
            points: vec![
                Point3D {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                Point3D {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0,
                },
            ],
            header: Header {
                timestamp: 123456789,
                frame_id: "base_link".to_string(),
            },
        };

        assert!(msg.validate().is_ok());
    }

    #[test]
    fn test_pose_quaternion_validation() {
        // Valid normalized quaternion
        let msg = PoseMessage {
            position: Point3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            header: Header {
                timestamp: 123456789,
                frame_id: "base_link".to_string(),
            },
        };

        assert!(msg.validate().is_ok());

        // Invalid non-normalized quaternion
        let bad_msg = PoseMessage {
            position: Point3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 1.0,
                y: 1.0,
                z: 1.0,
                w: 1.0,
            }, // Not normalized
            header: Header {
                timestamp: 123456789,
                frame_id: "base_link".to_string(),
            },
        };

        assert!(bad_msg.validate().is_err());
    }

    #[test]
    fn test_type_registry() {
        let registry = TypeRegistry::new();

        // Test that built-in types are registered
        assert!(registry.get_schema("String").is_some());
        assert!(registry.get_schema("Int32").is_some());
        assert!(registry.get_schema("PointCloud").is_some());
        assert!(registry.get_schema("NonexistentType").is_none());
    }
}
