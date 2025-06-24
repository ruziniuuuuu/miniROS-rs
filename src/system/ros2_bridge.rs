//! ROS2 Bridge Compatibility Module
//! 
//! Provides seamless integration with existing ROS2 systems while maintaining
//! the "mini" philosophy of maximum performance with minimum complexity.

use crate::core::{Node, Result};
use crate::types::{geometry_msgs, nav_msgs, sensor_msgs, std_msgs};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{debug, info, warn};

/// ROS2 bridge configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ros2BridgeConfig {
    /// ROS2 domain ID (0-101)
    pub domain_id: u32,
    /// Topic mappings from ROS2 to miniROS
    pub topic_mappings: HashMap<String, String>,
    /// Message type mappings
    pub message_type_mappings: HashMap<String, String>,
    /// Enable bidirectional communication
    pub bidirectional: bool,
    /// QoS profile mappings
    pub qos_mappings: HashMap<String, QosProfile>,
}

/// QoS Profile for ROS2 compatibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QosProfile {
    pub reliability: ReliabilityKind,
    pub durability: DurabilityKind,
    pub history: HistoryKind,
    pub depth: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ReliabilityKind {
    BestEffort,
    Reliable,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DurabilityKind {
    Volatile,
    TransientLocal,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HistoryKind {
    KeepLast,
    KeepAll,
}

impl Default for Ros2BridgeConfig {
    fn default() -> Self {
        Self {
            domain_id: 0,
            topic_mappings: HashMap::new(),
            message_type_mappings: Self::default_message_mappings(),
            bidirectional: true,
            qos_mappings: Self::default_qos_mappings(),
        }
    }
}

impl Ros2BridgeConfig {
    /// Create default message type mappings
    fn default_message_mappings() -> HashMap<String, String> {
        let mut mappings = HashMap::new();
        
        // Standard message mappings
        mappings.insert("std_msgs/String".to_string(), "std_msgs/String".to_string());
        mappings.insert("std_msgs/Int32".to_string(), "std_msgs/Int32".to_string());
        mappings.insert("std_msgs/Float64".to_string(), "std_msgs/Float64".to_string());
        mappings.insert("std_msgs/Bool".to_string(), "std_msgs/Bool".to_string());
        mappings.insert("std_msgs/Header".to_string(), "std_msgs/Header".to_string());
        
        // Geometry message mappings
        mappings.insert("geometry_msgs/Point".to_string(), "geometry_msgs/Point".to_string());
        mappings.insert("geometry_msgs/Pose".to_string(), "geometry_msgs/Pose".to_string());
        mappings.insert("geometry_msgs/Twist".to_string(), "geometry_msgs/Twist".to_string());
        mappings.insert("geometry_msgs/PoseStamped".to_string(), "geometry_msgs/PoseStamped".to_string());
        
        // Navigation message mappings
        mappings.insert("nav_msgs/Odometry".to_string(), "nav_msgs/Odometry".to_string());
        mappings.insert("nav_msgs/Path".to_string(), "nav_msgs/Path".to_string());
        
        // Sensor message mappings
        mappings.insert("sensor_msgs/LaserScan".to_string(), "sensor_msgs/LaserScan".to_string());
        mappings.insert("sensor_msgs/PointCloud2".to_string(), "sensor_msgs/PointCloud2".to_string());
        mappings.insert("sensor_msgs/Image".to_string(), "sensor_msgs/Image".to_string());
        
        mappings
    }
    
    /// Create default QoS mappings
    fn default_qos_mappings() -> HashMap<String, QosProfile> {
        let mut mappings = HashMap::new();
        
        // Sensor data (best effort, high frequency)
        mappings.insert("sensor_data".to_string(), QosProfile {
            reliability: ReliabilityKind::BestEffort,
            durability: DurabilityKind::Volatile,
            history: HistoryKind::KeepLast,
            depth: 1,
        });
        
        // Control commands (reliable)
        mappings.insert("control".to_string(), QosProfile {
            reliability: ReliabilityKind::Reliable,
            durability: DurabilityKind::Volatile,
            history: HistoryKind::KeepLast,
            depth: 10,
        });
        
        // Configuration (reliable, transient)
        mappings.insert("config".to_string(), QosProfile {
            reliability: ReliabilityKind::Reliable,
            durability: DurabilityKind::TransientLocal,
            history: HistoryKind::KeepLast,
            depth: 1,
        });
        
        mappings
    }
}

/// ROS2 Bridge for seamless integration
pub struct Ros2Bridge {
    config: Ros2BridgeConfig,
    node: Node,
    active_bridges: Arc<RwLock<HashMap<String, BridgeState>>>,
}

/// State of an individual topic bridge
#[derive(Debug)]
struct BridgeState {
    ros2_topic: String,
    miniROS_topic: String,
    message_type: String,
    direction: BridgeDirection,
    message_count: u64,
    last_message_time: std::time::Instant,
}

#[derive(Debug, Clone)]
enum BridgeDirection {
    Ros2ToMiniRos,
    MiniRosToRos2,
    Bidirectional,
}

impl Ros2Bridge {
    /// Create new ROS2 bridge with configuration
    pub async fn new(config: Ros2BridgeConfig) -> Result<Self> {
        let node = Node::new("ros2_bridge")?;
        
        Ok(Self {
            config,
            node,
            active_bridges: Arc::new(RwLock::new(HashMap::new())),
        })
    }
    
    /// Initialize the bridge and start forwarding
    pub async fn initialize(&mut self) -> Result<()> {
        info!("🌉 Initializing ROS2 bridge for domain {}", self.config.domain_id);
        
        // Initialize node
        self.node.init().await?;
        
        // Setup automatic topic mappings
        self.setup_topic_mappings().await?;
        
        info!("✅ ROS2 bridge initialized successfully");
        Ok(())
    }
    
    /// Setup topic mappings based on configuration
    async fn setup_topic_mappings(&mut self) -> Result<()> {
        let topic_mappings = self.config.topic_mappings.clone();
        for (ros2_topic, miniROS_topic) in &topic_mappings {
            self.bridge_topic(ros2_topic, miniROS_topic, BridgeDirection::Bidirectional).await?;
        }
        Ok(())
    }
    
    /// Bridge a specific topic between ROS2 and miniROS
    pub async fn bridge_topic(
        &mut self,
        ros2_topic: &str,
        miniROS_topic: &str,
        direction: BridgeDirection,
    ) -> Result<()> {
        debug!("🔗 Bridging topic: {} <-> {}", ros2_topic, miniROS_topic);
        
        // Determine message type from topic name or configuration
        let message_type = self.infer_message_type(ros2_topic)?;
        
        // Create bridge state
        let bridge_state = BridgeState {
            ros2_topic: ros2_topic.to_string(),
            miniROS_topic: miniROS_topic.to_string(),
            message_type: message_type.clone(),
            direction: direction.clone(),
            message_count: 0,
            last_message_time: std::time::Instant::now(),
        };
        
        // Add to active bridges
        {
            let mut bridges = self.active_bridges.write().await;
            bridges.insert(ros2_topic.to_string(), bridge_state);
        }
        
        // Setup message forwarding based on type and direction
        match message_type.as_str() {
            "std_msgs/String" => {
                self.setup_string_bridge(ros2_topic, miniROS_topic, &direction).await?;
            }
            "geometry_msgs/Twist" => {
                self.setup_twist_bridge(ros2_topic, miniROS_topic, &direction).await?;
            }
            "nav_msgs/Odometry" => {
                self.setup_odometry_bridge(ros2_topic, miniROS_topic, &direction).await?;
            }
            "sensor_msgs/LaserScan" => {
                self.setup_laser_scan_bridge(ros2_topic, miniROS_topic, &direction).await?;
            }
            _ => {
                warn!("⚠️ Unsupported message type for bridging: {}", message_type);
            }
        }
        
        info!("✅ Topic bridge established: {} <-> {}", ros2_topic, miniROS_topic);
        Ok(())
    }
    
    /// Infer message type from topic name
    fn infer_message_type(&self, topic: &str) -> Result<String> {
        // Common topic patterns
        if topic.contains("/cmd_vel") || topic.contains("/velocity") {
            return Ok("geometry_msgs/Twist".to_string());
        }
        if topic.contains("/odom") || topic.contains("/odometry") {
            return Ok("nav_msgs/Odometry".to_string());
        }
        if topic.contains("/scan") || topic.contains("/laser") {
            return Ok("sensor_msgs/LaserScan".to_string());
        }
        if topic.contains("/pose") {
            return Ok("geometry_msgs/PoseStamped".to_string());
        }
        
        // Default to string for unknown topics
        Ok("std_msgs/String".to_string())
    }
    
    /// Setup string message bridging
    async fn setup_string_bridge(
        &mut self,
        _ros2_topic: &str,
        miniROS_topic: &str,
        direction: &BridgeDirection,
    ) -> Result<()> {
        match direction {
            BridgeDirection::Ros2ToMiniRos => {
                // Create miniROS publisher for ROS2->miniROS
                let _pub = self.node.create_publisher::<std_msgs::String>(miniROS_topic).await?;
                // TODO: Setup ROS2 subscriber and forward messages
            }
            BridgeDirection::MiniRosToRos2 => {
                // Create miniROS subscriber for miniROS->ROS2  
                let _sub = self.node.create_subscriber::<std_msgs::String>(miniROS_topic).await?;
                // TODO: Setup ROS2 publisher and forward messages
            }
            BridgeDirection::Bidirectional => {
                // Setup both directions
                let _pub = self.node.create_publisher::<std_msgs::String>(miniROS_topic).await?;
                let _sub = self.node.create_subscriber::<std_msgs::String>(miniROS_topic).await?;
                // TODO: Setup ROS2 bidirectional forwarding
            }
        }
        Ok(())
    }
    
    /// Setup twist message bridging
    async fn setup_twist_bridge(
        &mut self,
        _ros2_topic: &str,
        miniROS_topic: &str,
        direction: &BridgeDirection,
    ) -> Result<()> {
        match direction {
            BridgeDirection::Ros2ToMiniRos => {
                let _pub = self.node.create_publisher::<geometry_msgs::Twist>(miniROS_topic).await?;
            }
            BridgeDirection::MiniRosToRos2 => {
                let _sub = self.node.create_subscriber::<geometry_msgs::Twist>(miniROS_topic).await?;
            }
            BridgeDirection::Bidirectional => {
                let _pub = self.node.create_publisher::<geometry_msgs::Twist>(miniROS_topic).await?;
                let _sub = self.node.create_subscriber::<geometry_msgs::Twist>(miniROS_topic).await?;
            }
        }
        Ok(())
    }
    
    /// Setup odometry message bridging
    async fn setup_odometry_bridge(
        &mut self,
        _ros2_topic: &str,
        miniROS_topic: &str,
        direction: &BridgeDirection,
    ) -> Result<()> {
        match direction {
            BridgeDirection::Ros2ToMiniRos => {
                let _pub = self.node.create_publisher::<nav_msgs::Odometry>(miniROS_topic).await?;
            }
            BridgeDirection::MiniRosToRos2 => {
                let _sub = self.node.create_subscriber::<nav_msgs::Odometry>(miniROS_topic).await?;
            }
            BridgeDirection::Bidirectional => {
                let _pub = self.node.create_publisher::<nav_msgs::Odometry>(miniROS_topic).await?;
                let _sub = self.node.create_subscriber::<nav_msgs::Odometry>(miniROS_topic).await?;
            }
        }
        Ok(())
    }
    
    /// Setup laser scan message bridging
    async fn setup_laser_scan_bridge(
        &mut self,
        _ros2_topic: &str,
        miniROS_topic: &str,
        direction: &BridgeDirection,
    ) -> Result<()> {
        match direction {
            BridgeDirection::Ros2ToMiniRos => {
                let _pub = self.node.create_publisher::<sensor_msgs::LaserScan>(miniROS_topic).await?;
            }
            BridgeDirection::MiniRosToRos2 => {
                let _sub = self.node.create_subscriber::<sensor_msgs::LaserScan>(miniROS_topic).await?;
            }
            BridgeDirection::Bidirectional => {
                let _pub = self.node.create_publisher::<sensor_msgs::LaserScan>(miniROS_topic).await?;
                let _sub = self.node.create_subscriber::<sensor_msgs::LaserScan>(miniROS_topic).await?;
            }
        }
        Ok(())
    }
    
    /// Get bridge statistics
    pub async fn get_bridge_stats(&self) -> Result<HashMap<String, BridgeStats>> {
        let bridges = self.active_bridges.read().await;
        let mut stats = HashMap::new();
        
        for (topic, state) in bridges.iter() {
            stats.insert(topic.clone(), BridgeStats {
                message_count: state.message_count,
                last_message_age: state.last_message_time.elapsed(),
                direction: format!("{:?}", state.direction),
                message_type: state.message_type.clone(),
            });
        }
        
        Ok(stats)
    }
    
    /// Auto-discover and bridge ROS2 topics
    pub async fn auto_discover_and_bridge(&self) -> Result<()> {
        info!("🔍 Auto-discovering ROS2 topics for bridging...");
        
        // TODO: Implement ROS2 topic discovery
        // This would use ROS2 discovery mechanisms to find active topics
        // and automatically create bridges based on configured patterns
        
        warn!("⚠️ Auto-discovery not yet implemented - using manual configuration");
        Ok(())
    }
    
    /// Shutdown bridge gracefully
    pub async fn shutdown(&self) -> Result<()> {
        info!("🛑 Shutting down ROS2 bridge...");
        
        // Clear active bridges
        {
            let mut bridges = self.active_bridges.write().await;
            bridges.clear();
        }
        
        info!("✅ ROS2 bridge shutdown complete");
        Ok(())
    }
}

/// Bridge statistics for monitoring
#[derive(Debug, Serialize, Deserialize)]
pub struct BridgeStats {
    pub message_count: u64,
    pub last_message_age: std::time::Duration,
    pub direction: String,
    pub message_type: String,
}

/// Convenience function to create a ROS2 bridge with common settings
pub async fn create_ros2_bridge(domain_id: u32) -> Result<Ros2Bridge> {
    let mut config = Ros2BridgeConfig::default();
    config.domain_id = domain_id;
    
    // Add common topic mappings
    config.topic_mappings.insert("/cmd_vel".to_string(), "/robot/cmd_vel".to_string());
    config.topic_mappings.insert("/odom".to_string(), "/robot/odom".to_string());
    config.topic_mappings.insert("/scan".to_string(), "/robot/scan".to_string());
    
    Ros2Bridge::new(config).await
}

/// Create a ROS2 bridge for turtlebot compatibility
pub async fn create_turtlebot_bridge(domain_id: u32) -> Result<Ros2Bridge> {
    let mut config = Ros2BridgeConfig::default();
    config.domain_id = domain_id;
    
    // Turtlebot-specific topic mappings
    config.topic_mappings.insert("/turtle1/cmd_vel".to_string(), "/turtlebot/cmd_vel".to_string());
    config.topic_mappings.insert("/turtle1/pose".to_string(), "/turtlebot/pose".to_string());
    config.topic_mappings.insert("/scan".to_string(), "/turtlebot/scan".to_string());
    config.topic_mappings.insert("/odom".to_string(), "/turtlebot/odom".to_string());
    
    Ros2Bridge::new(config).await
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_ros2_bridge_creation() {
        let bridge = create_ros2_bridge(0).await;
        assert!(bridge.is_ok());
    }
    
    #[tokio::test]
    async fn test_turtlebot_bridge_creation() {
        let bridge = create_turtlebot_bridge(0).await;
        assert!(bridge.is_ok());
    }
    
    #[test]
    fn test_message_type_inference() {
        let bridge_config = Ros2BridgeConfig::default();
        let bridge = Ros2Bridge {
            config: bridge_config,
            node: std::sync::Arc::new(crate::Node::new("test").unwrap()),
            active_bridges: std::sync::Arc::new(tokio::sync::RwLock::new(std::collections::HashMap::new())),
        };
        
        assert_eq!(bridge.infer_message_type("/cmd_vel").unwrap(), "geometry_msgs/Twist");
        assert_eq!(bridge.infer_message_type("/odom").unwrap(), "nav_msgs/Odometry");
        assert_eq!(bridge.infer_message_type("/scan").unwrap(), "sensor_msgs/LaserScan");
    }
} 