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
            spawn_viewer: false,  // Use buffered mode by default to avoid requiring external viewer
        }
    }
}

/// Rerun visualization client
#[derive(Clone)]
pub struct VisualizationClient {
    recording: Arc<RecordingStream>,
}

impl VisualizationClient {
    /// Create new visualization client
    pub fn new(config: VisualizationConfig) -> Result<Self> {
        let recording = if config.spawn_viewer {
            RecordingStreamBuilder::new(config.application_id.clone())
                .spawn()
                .map_err(|e| MiniRosError::Other(format!("Failed to spawn viewer: {}", e)))?
        } else {
            RecordingStreamBuilder::new(config.application_id.clone())
                .buffered()
                .map_err(|e| MiniRosError::Other(format!("Failed to create recording stream: {}", e)))?
        };

        Ok(Self {
            recording: Arc::new(recording),
        })
    }

    /// Log scalar value
    pub fn log_scalar(&self, entity_path: &str, value: f64) -> Result<()> {
        self.recording
            .log(entity_path, &rerun::Scalar::new(value))
            .map_err(|e| MiniRosError::Other(format!("Failed to log scalar: {}", e)))?;
        
        tracing::debug!("Logged scalar {} to entity: {}", value, entity_path);
        Ok(())
    }

    /// Log 2D points
    pub fn log_points_2d(&self, entity_path: &str, points: Vec<[f32; 2]>) -> Result<()> {
        self.recording
            .log(entity_path, &rerun::Points2D::new(points))
            .map_err(|e| MiniRosError::Other(format!("Failed to log 2D points: {}", e)))?;
        
        tracing::debug!("Logged 2D points to entity: {}", entity_path);
        Ok(())
    }

    /// Log 3D points
    pub fn log_points_3d(&self, entity_path: &str, points: Vec<[f32; 3]>) -> Result<()> {
        self.recording
            .log(entity_path, &rerun::Points3D::new(points))
            .map_err(|e| MiniRosError::Other(format!("Failed to log 3D points: {}", e)))?;
        
        tracing::debug!("Logged 3D points to entity: {}", entity_path);
        Ok(())
    }

    /// Log text message
    pub fn log_text(&self, entity_path: &str, text: &str) -> Result<()> {
        self.recording
            .log(entity_path, &rerun::TextLog::new(text))
            .map_err(|e| MiniRosError::Other(format!("Failed to log text: {}", e)))?;
        
        tracing::debug!("Logged text to entity: {}", entity_path);
        Ok(())
    }

    /// Log transform (position and rotation)
    pub fn log_transform_3d(
        &self, 
        entity_path: &str, 
        translation: [f32; 3], 
        rotation_quat: [f32; 4]
    ) -> Result<()> {
        use rerun::Transform3D;
        
        let transform = Transform3D::from_translation_rotation(
            translation,
            rerun::Rotation3D::Quaternion(rerun::components::RotationQuat::from(rotation_quat))
        );

        self.recording
            .log(entity_path, &transform)
            .map_err(|e| MiniRosError::Other(format!("Failed to log transform: {}", e)))?;
        
        tracing::debug!("Logged transform to entity: {}", entity_path);
        Ok(())
    }

    /// Get recording stream reference
    pub fn recording(&self) -> Arc<RecordingStream> {
        self.recording.clone()
    }

    /// Flush any pending logs
    pub fn flush(&self) -> Result<()> {
        // Rerun handles flushing automatically in most cases
        tracing::debug!("Visualization client flush requested");
        Ok(())
    }
}

/// Helper trait for visualizable data types
pub trait Visualizable {
    /// Visualize data using Rerun client
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()>;
}

/// Robot pose visualization data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotPose {
    pub position: [f32; 3],
    pub orientation: [f32; 4], // quaternion [x, y, z, w]
}

impl Visualizable for RobotPose {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        client.log_transform_3d(entity_path, self.position, self.orientation)
    }
}

/// Point cloud visualization data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloud {
    pub points: Vec<[f32; 3]>,
}

impl Visualizable for PointCloud {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        client.log_points_3d(entity_path, self.points.clone())
    }
}

/// Laser scan visualization data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserScan {
    pub ranges: Vec<f32>,
    pub angle_min: f32,
    pub angle_max: f32,
}

impl Visualizable for LaserScan {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Convert laser scan to 2D points
        let mut points = Vec::new();
        let angle_increment = (self.angle_max - self.angle_min) / (self.ranges.len() as f32 - 1.0);
        
        for (i, &range) in self.ranges.iter().enumerate() {
            if range > 0.0 && range.is_finite() {
                let angle = self.angle_min + (i as f32) * angle_increment;
                let x = range * angle.cos();
                let y = range * angle.sin();
                points.push([x, y]);
            }
        }
        
        client.log_points_2d(entity_path, points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_visualization_config() {
        let config = VisualizationConfig::default();
        assert_eq!(config.application_id, "miniROS");
        assert!(!config.spawn_viewer);  // Default is buffered mode
    }

    #[test]
    fn test_robot_pose_creation() {
        let pose = RobotPose {
            position: [1.0, 2.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(pose.position, [1.0, 2.0, 0.0]);
    }
} 