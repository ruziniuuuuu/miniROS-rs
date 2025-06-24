//! ROS2 Bridge Demo - Phase 3 Feature
//!
//! This example demonstrates the ROS2 bridge compatibility system that provides
//! seamless integration with existing ROS2 systems while maintaining the 
//! "mini" philosophy of maximum performance with minimum complexity.
//!
//! Features demonstrated:
//! - Automatic topic bridging between ROS2 and miniROS
//! - Message type compatibility  
//! - QoS profile mapping
//! - Bidirectional communication
//! - Real-time monitoring and statistics
//!
//! Run with: cargo run --example 18_ros2_bridge_demo

use mini_ros::prelude::*;
use mini_ros::types::{geometry_msgs, nav_msgs, sensor_msgs, std_msgs};
use mini_ros::{create_ros2_bridge, create_turtlebot_bridge, Ros2BridgeConfig};
use std::time::Duration;
use tokio::time::{interval, sleep};
use tracing::info;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    info!("🌉 miniROS ROS2 Bridge Demo - Phase 3 Feature");
    info!("=" = 50);
    info!("Demonstrating seamless ROS2 integration with mini philosophy!");
    
    // Show bridge configurations
    demo_bridge_configurations().await?;
    
    // Demo 1: Basic ROS2 bridge
    demo_basic_ros2_bridge().await?;
    
    // Demo 2: Turtlebot-specific bridge 
    demo_turtlebot_bridge().await?;
    
    // Demo 3: Custom bridge configuration
    demo_custom_bridge_configuration().await?;
    
    // Demo 4: Bridge monitoring and statistics
    demo_bridge_monitoring().await?;
    
    info!("🎉 ROS2 Bridge demo completed successfully!");
    info!("💡 This demonstrates Phase 3 ecosystem features with mini philosophy:");
    info!("   • Maximum performance: Zero-copy message bridging");
    info!("   • Minimum complexity: Auto-configuration and discovery");
    info!("   • ROS2 compatibility: Seamless integration");
    info!("   • Production ready: Monitoring and error handling");
    
    Ok(())
}

/// Demonstrate different bridge configurations
async fn demo_bridge_configurations() -> Result<()> {
    info!("📋 === Bridge Configuration Examples ===");
    
    // Show default configuration
    let default_config = Ros2BridgeConfig::default();
    info!("🔧 Default bridge configuration:");
    info!("   Domain ID: {}", default_config.domain_id);
    info!("   Bidirectional: {}", default_config.bidirectional);
    info!("   Message types: {} mappings", default_config.message_type_mappings.len());
    info!("   QoS profiles: {} profiles", default_config.qos_mappings.len());
    
    // Show some key message mappings
    info!("📦 Key message type mappings:");
    for (ros2_type, mini_type) in default_config.message_type_mappings.iter().take(5) {
        info!("   {} → {}", ros2_type, mini_type);
    }
    
    sleep(Duration::from_secs(2)).await;
    Ok(())
}

/// Demo basic ROS2 bridge functionality
async fn demo_basic_ros2_bridge() -> Result<()> {
    info!("🚀 === Demo 1: Basic ROS2 Bridge ===");
    
    // Create and initialize bridge
    let mut bridge = create_ros2_bridge(0).await?;
    bridge.initialize().await?;
    
    // Create miniROS node for testing
    let mut node = Node::new("bridge_test_node")?;
    node.init().await?;
    
    // Setup publishers and subscribers for testing
    let cmd_vel_pub = node.create_publisher::<geometry_msgs::Twist>("/robot/cmd_vel").await?;
    let _odom_sub = node.create_subscriber::<nav_msgs::Odometry>("/robot/odom").await?;
    
    // Simulate robot control through bridge
    info!("🎮 Publishing velocity commands through bridge...");
    
    let mut timer = interval(Duration::from_millis(500));
    for i in 0..5 {
        timer.tick().await;
        
        let twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 {
                x: 0.5 + (i as f64 * 0.1),
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.2 * (i as f64).sin(),
            },
        };
        
        cmd_vel_pub.publish(&twist).await?;
        info!("📤 Published cmd_vel: linear.x={:.2}, angular.z={:.2}", 
              twist.linear.x, twist.angular.z);
    }
    
    // Show bridge statistics
    let stats = bridge.get_bridge_stats().await?;
    info!("📊 Bridge statistics:");
    for (topic, stat) in stats {
        info!("   {}: {} messages, type: {}", 
              topic, stat.message_count, stat.message_type);
    }
    
    bridge.shutdown().await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demo turtlebot-specific bridge configuration
async fn demo_turtlebot_bridge() -> Result<()> {
    info!("🐢 === Demo 2: Turtlebot Bridge ===");
    
    // Create turtlebot-optimized bridge
    let mut bridge = create_turtlebot_bridge(0).await?;
    bridge.initialize().await?;
    
    // Create miniROS node
    let mut node = Node::new("turtlebot_bridge_test")?;
    node.init().await?;
    
    // Setup turtlebot-specific topics
    let turtle_cmd_pub = node.create_publisher::<geometry_msgs::Twist>("/turtlebot/cmd_vel").await?;
    let _turtle_pose_sub = node.create_subscriber::<geometry_msgs::PoseStamped>("/turtlebot/pose").await?;
    
    info!("🎯 Simulating turtlebot control through ROS2 bridge...");
    
    // Simulate turtlebot movements
    let movements = vec![
        ("Forward", 0.5, 0.0),
        ("Turn Left", 0.0, 1.0),
        ("Backward", -0.3, 0.0),
        ("Turn Right", 0.0, -1.0),
        ("Stop", 0.0, 0.0),
    ];
    
    for (action, linear, angular) in movements {
        let twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 { x: linear, y: 0.0, z: 0.0 },
            angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: angular },
        };
        
        turtle_cmd_pub.publish(&twist).await?;
        info!("🐢 Turtlebot {}: linear={:.1}, angular={:.1}", action, linear, angular);
        
        sleep(Duration::from_millis(800)).await;
    }
    
    bridge.shutdown().await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demo custom bridge configuration
async fn demo_custom_bridge_configuration() -> Result<()> {
    info!("⚙️  === Demo 3: Custom Bridge Configuration ===");
    
    // Create custom configuration
    let mut config = Ros2BridgeConfig::default();
    config.domain_id = 42; // Custom domain
    
    // Add custom topic mappings
    config.topic_mappings.insert("/custom/sensors/lidar".to_string(), "/robot/lidar".to_string());
    config.topic_mappings.insert("/custom/actuators/gripper".to_string(), "/robot/gripper".to_string());
    config.topic_mappings.insert("/custom/navigation/goal".to_string(), "/robot/nav_goal".to_string());
    
    info!("🔧 Custom configuration:");
    info!("   Domain ID: {}", config.domain_id);
    info!("   Custom topic mappings: {}", config.topic_mappings.len());
    
    // Create bridge with custom config
    let mut bridge = mini_ros::Ros2Bridge::new(config).await?;
    bridge.initialize().await?;
    
    // Demonstrate custom topic bridging
    let mut node = Node::new("custom_bridge_test")?;
    node.init().await?;
    
    // Setup custom topics
    let lidar_pub = node.create_publisher::<sensor_msgs::LaserScan>("/robot/lidar").await?;
    let gripper_pub = node.create_publisher::<std_msgs::Float64>("/robot/gripper").await?;
    
    info!("🔬 Publishing to custom bridged topics...");
    
    // Simulate sensor data
    for i in 0..3 {
        // Simulate lidar scan
        let laser_scan = sensor_msgs::LaserScan {
            header: std_msgs::Header {
                stamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as i64,
                frame_id: "lidar_frame".to_string(),
            },
            angle_min: -1.57,
            angle_max: 1.57,
            angle_increment: 0.01,
            time_increment: 0.0,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges: (0..314).map(|j| 1.0 + (i + j) as f32 * 0.01).collect(),
            intensities: Vec::new(),
        };
        
        lidar_pub.publish(&laser_scan).await?;
        
        // Simulate gripper position
        let gripper_pos = std_msgs::Float64 {
            data: 0.5 + 0.3 * (i as f64 * 0.5).sin(),
        };
        
        gripper_pub.publish(&gripper_pos).await?;
        
        info!("📡 Published sensor data - iteration {}", i + 1);
        sleep(Duration::from_millis(300)).await;
    }
    
    bridge.shutdown().await?;
    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demo bridge monitoring and statistics
async fn demo_bridge_monitoring() -> Result<()> {
    info!("📊 === Demo 4: Bridge Monitoring ===");
    
    let mut bridge = create_ros2_bridge(0).await?;
    bridge.initialize().await?;
    
    // Create test node
    let mut node = Node::new("monitoring_test")?;
    node.init().await?;
    
    // Setup multiple publishers for monitoring
    let status_pub = node.create_publisher::<std_msgs::String>("/robot/status").await?;
    let cmd_pub = node.create_publisher::<geometry_msgs::Twist>("/robot/cmd_vel").await?;
    let odom_pub = node.create_publisher::<nav_msgs::Odometry>("/robot/odom").await?;
    
    info!("🔍 Starting monitored message publishing...");
    
    // Publish messages at different rates
    let mut timer = interval(Duration::from_millis(200));
    for i in 0..10 {
        timer.tick().await;
        
        // Status updates (low frequency)
        if i % 5 == 0 {
            let status = std_msgs::String {
                data: format!("Robot operational - cycle {}", i / 5 + 1),
            };
            status_pub.publish(&status).await?;
        }
        
        // Command updates (medium frequency)
        if i % 2 == 0 {
            let cmd = geometry_msgs::Twist {
                linear: geometry_msgs::Vector3 { x: 0.3, y: 0.0, z: 0.0 },
                angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.1 },
            };
            cmd_pub.publish(&cmd).await?;
        }
        
        // Odometry updates (high frequency)
        let odom = nav_msgs::Odometry {
            header: std_msgs::Header {
                stamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as i64,
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            pose: geometry_msgs::PoseWithCovariance {
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: i as f64 * 0.1,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: geometry_msgs::Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
                covariance: vec![0.0; 36],
            },
            twist: geometry_msgs::TwistWithCovariance {
                twist: geometry_msgs::Twist {
                    linear: geometry_msgs::Vector3 { x: 0.3, y: 0.0, z: 0.0 },
                    angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.1 },
                },
                covariance: vec![0.0; 36],
            },
        };
        odom_pub.publish(&odom).await?;
        
        // Show periodic statistics
        if i % 3 == 0 {
            let stats = bridge.get_bridge_stats().await?;
            info!("📈 Bridge stats at step {}:", i);
            for (topic, stat) in stats.iter().take(3) {
                info!("   {}: {} msgs, age: {:?}", 
                      topic, stat.message_count, stat.last_message_age);
            }
        }
    }
    
    // Final statistics report
    let final_stats = bridge.get_bridge_stats().await?;
    info!("📊 Final bridge statistics:");
    info!("   Total bridged topics: {}", final_stats.len());
    
    let total_messages: u64 = final_stats.values().map(|s| s.message_count).sum();
    info!("   Total messages bridged: {}", total_messages);
    
    // Show performance characteristics
    info!("⚡ Performance characteristics:");
    info!("   • Zero-copy message bridging");
    info!("   • Automatic type inference");
    info!("   • Real-time statistics");
    info!("   • Graceful error handling");
    
    bridge.shutdown().await?;
    Ok(())
}

/// Utility function to get current timestamp
fn get_current_time_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
} 