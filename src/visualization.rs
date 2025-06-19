//! Rerun visualization module for miniROS
//! 
//! Provides data visualization capabilities using Rerun viewer.

use crate::error::{Result, MiniRosError};
use rerun::{RecordingStream, RecordingStreamBuilder};
use serde::{Serialize, Deserialize};
use std::sync::Arc;

/// Visualization configuration
#[derive(Clone, Debug)]
pub struct VisualizationConfig {
    pub application_id: String,
    pub spawn_viewer: bool,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            application_id: "miniROS".to_string(),
            spawn_viewer: false,  // Use memory recording by default
        }
    }
}

/// Rerun visualization client
#[derive(Clone)]
pub struct VisualizationClient {
    recording: Arc<RecordingStream>,
}

impl VisualizationClient {
    /// Create a new visualization client
    pub fn new(config: VisualizationConfig) -> Result<Self> {
        let recording = if config.spawn_viewer {
            RecordingStreamBuilder::new(config.application_id)
                .spawn()
                .map_err(|e| MiniRosError::Other(format!("Failed to spawn viewer: {}", e)))?
        } else {
            let (recording, _storage) = RecordingStreamBuilder::new(config.application_id)
                .memory()
                .map_err(|e| MiniRosError::Other(format!("Failed to create memory recording: {}", e)))?;
            recording
        };

        Ok(Self {
            recording: Arc::new(recording),
        })
    }

    /// Log scalar value using the correct rerun 0.20 API
    pub fn log_scalar(&self, entity_path: &str, value: f64) -> Result<()> {
        // Use the Scalar archetype directly which is the simplest approach
        self.recording
            .log(entity_path, &rerun::Scalar::new(value))
            .map_err(|e| MiniRosError::Other(format!("Failed to log scalar: {}", e)))?;
        
        tracing::debug!("Logged scalar {} to entity: {}", value, entity_path);
        Ok(())
    }

    /// Log point cloud
    pub fn log_points(&self, entity_path: &str, points: Vec<[f32; 3]>) -> Result<()> {
        let points_len = points.len();
        self.recording
            .log(entity_path, &rerun::Points3D::new(points))
            .map_err(|e| MiniRosError::Other(format!("Failed to log points: {}", e)))?;
        
        tracing::debug!("Logged {} points to entity: {}", points_len, entity_path);
        Ok(())
    }

    /// Log transform 
    pub fn log_transform(
        &self, 
        entity_path: &str, 
        translation: [f32; 3],
        rotation_quat: [f32; 4]
    ) -> Result<()> {
        let transform = rerun::Transform3D::from_translation_rotation(
            translation,
            rerun::Rotation3D::Quaternion(rotation_quat.into())
        );

        self.recording
            .log(entity_path, &transform)
            .map_err(|e| MiniRosError::Other(format!("Failed to log transform: {}", e)))?;
        
        tracing::debug!("Logged transform to entity: {}", entity_path);
        Ok(())
    }

    /// Log image
    pub fn log_image(&self, entity_path: &str, image_data: &[u8], width: u32, height: u32) -> Result<()> {
        // Create a simple RGB image using the correct API
        let image = rerun::Image::from_rgb24(image_data.to_vec(), [width, height]);

        self.recording
            .log(entity_path, &image)
            .map_err(|e| MiniRosError::Other(format!("Failed to log image: {}", e)))?;
        
        tracing::debug!("Logged image {}x{} to entity: {}", width, height, entity_path);
        Ok(())
    }

    /// Set recording time
    pub fn set_time(&self, timeline: &str, nanos: i64) -> Result<()> {
        self.recording.set_time_nanos(timeline, nanos);
        Ok(())
    }

    /// Log text message
    pub fn log_text(&self, entity_path: &str, message: &str) -> Result<()> {
        self.recording
            .log(entity_path, &rerun::TextLog::new(message))
            .map_err(|e| MiniRosError::Other(format!("Failed to log text: {}", e)))?;
        
        tracing::debug!("Logged text to entity: {}", entity_path);
        Ok(())
    }
}

/// A simple visualization data type
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationData {
    pub entity_path: String,
    pub timestamp: i64,
    pub data_type: String,
    pub metadata: std::collections::HashMap<String, String>,
}

impl VisualizationData {
    pub fn new(entity_path: String, data_type: String) -> Self {
        Self {
            entity_path,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            data_type,
            metadata: std::collections::HashMap::new(),
        }
    }
}

/// Robot pose representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotPose {
    pub position: [f32; 3],      // [x, y, z]
    pub orientation: [f32; 4],   // quaternion [x, y, z, w]
}

/// Point cloud data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloud {
    pub points: Vec<[f32; 3]>,   // Vector of 3D points
}

/// Laser scan data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserScan {
    pub ranges: Vec<f32>,        // Distance measurements
    pub angle_min: f32,          // Start angle
    pub angle_max: f32,          // End angle
}

/// Trait for types that can be visualized
pub trait Visualizable {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()>;
}

impl Visualizable for RobotPose {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Log the robot pose as a transform
        client.log_transform(entity_path, self.position, self.orientation)?;
        Ok(())
    }
}

impl Visualizable for PointCloud {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Log the point cloud
        client.log_points(entity_path, self.points.clone())?;
        Ok(())
    }
}

impl Visualizable for LaserScan {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Convert laser scan to 3D points for visualization
        let mut points = Vec::new();
        let angle_increment = (self.angle_max - self.angle_min) / self.ranges.len() as f32;
        
        for (i, &range) in self.ranges.iter().enumerate() {
            if range > 0.0 && range < 100.0 {  // Filter out invalid readings
                let angle = self.angle_min + i as f32 * angle_increment;
                let x = range * angle.cos();
                let y = range * angle.sin();
                points.push([x, y, 0.0]);  // 2D scan at z=0
            }
        }
        
        client.log_points(entity_path, points)?;
        Ok(())
    }
}



#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_visualization_config_default() {
        let config = VisualizationConfig::default();
        assert_eq!(config.application_id, "miniROS");
        assert!(!config.spawn_viewer);
    }

    #[test]
    fn test_visualization_data_creation() {
        let data = VisualizationData::new("test/entity".to_string(), "scalar".to_string());
        assert_eq!(data.entity_path, "test/entity");
        assert_eq!(data.data_type, "scalar");
        assert!(data.timestamp > 0);
    }
} 