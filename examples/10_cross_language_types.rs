//! Example 10: Cross-Language Type System Demo
//!
//! This example demonstrates miniROS unified type system:
//! - ROS2-compatible message types that work across Rust and Python
//! - Message validation and serialization
//! - Standard robotics message packages (geometry_msgs, nav_msgs)
//! - Type safety with performance focus
//!
//! Following miniROS philosophy: Essential robotics functionality with minimal complexity
//!
//! Run with: cargo run --example 10_cross_language_types

use mini_ros::prelude::*;
use mini_ros::types::{geometry_msgs, nav_msgs, std_msgs, MiniRosMessage};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::time::sleep;
use tracing::info;

/// Get current timestamp in nanoseconds
fn get_current_time_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS Cross-Language Type System Demo ===");
    info!("🌐 Demonstrating ROS2-compatible message types");

    // Create and initialize node with minimal setup
    let mut node = Node::new("type_system_demo")?;
    node.init().await?;

    info!("✅ Node initialized successfully");

    // ========================================================================
    // Demonstrate Pose Message with Validation
    // ========================================================================
    
    info!("\n📍 Demonstrating geometry_msgs::Pose with validation:");
    
    let pose_pub = node.create_publisher::<geometry_msgs::PoseStamped>("/robot/pose").await?;
    let pose_sub = node.create_subscriber::<geometry_msgs::PoseStamped>("/robot/pose").await?;

    // Track received poses for verification
    let received_poses = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let poses_clone = received_poses.clone();

    pose_sub.on_message(move |pose: geometry_msgs::PoseStamped| {
        poses_clone.lock().unwrap().push(pose.clone());
        info!("📨 Received pose: position=({:.2}, {:.2}, {:.2}), frame='{}'",
            pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z,
            pose.header.frame_id);
    })?;

    // Publish valid poses demonstrating proper message construction
    for i in 0..3 {
        let pose_stamped = geometry_msgs::PoseStamped {
            header: std_msgs::Header {
                stamp: get_current_time_ns(),
                frame_id: "base_link".to_string(),
            },
            pose: geometry_msgs::Pose {
                position: geometry_msgs::Point {
                    x: i as f64,
                    y: (i * 2) as f64,
                    z: 0.5,
                },
                orientation: geometry_msgs::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0, // Valid unit quaternion
                },
            },
        };

        // Validate before publishing - demonstrates type safety
        match pose_stamped.validate() {
            Ok(_) => {
                pose_pub.publish(&pose_stamped).await?;
                info!("✅ Published valid pose #{}", i + 1);
            }
            Err(e) => {
                info!("❌ Pose validation failed: {}", e);
            }
        }

        sleep(Duration::from_millis(200)).await;
    }

    // ========================================================================
    // Demonstrate Validation with Invalid Data
    // ========================================================================
    
    info!("\n🔍 Testing message validation:");
    
    let invalid_pose = geometry_msgs::PoseStamped {
        header: std_msgs::Header {
            stamp: get_current_time_ns(),
            frame_id: "test".to_string(),
        },
        pose: geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: 1.0,
                y: 1.0,
                z: 1.0,
                w: 1.0, // Invalid: not normalized
            },
        },
    };

    match invalid_pose.validate() {
        Ok(_) => info!("❓ Invalid pose somehow passed validation"),
        Err(e) => info!("✅ Validation correctly rejected invalid quaternion: {}", e),
    }

    // ========================================================================
    // Demonstrate Navigation Messages (Odometry)
    // ========================================================================
    
    info!("\n🧭 Demonstrating nav_msgs::Odometry:");
    
    let odom_pub = node.create_publisher::<nav_msgs::Odometry>("/robot/odom").await?;
    let odom_sub = node.create_subscriber::<nav_msgs::Odometry>("/robot/odom").await?;

    let received_odoms = std::sync::Arc::new(std::sync::Mutex::new(0u32));
    let odoms_clone = received_odoms.clone();

    odom_sub.on_message(move |odom: nav_msgs::Odometry| {
        let mut count = odoms_clone.lock().unwrap();
        *count += 1;
        info!("📊 Received odometry #{}: child_frame='{}'", 
            *count, odom.child_frame_id);
    })?;

    // Create and publish odometry messages
    for i in 0..2 {
        let odometry = nav_msgs::Odometry {
            header: std_msgs::Header {
                stamp: get_current_time_ns(),
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            pose: geometry_msgs::PoseWithCovariance {
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: i as f64 * 0.5,
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
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
            twist: geometry_msgs::TwistWithCovariance {
                twist: geometry_msgs::Twist {
                    linear: geometry_msgs::Vector3 {
                        x: 0.5,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: geometry_msgs::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.1,
                    },
                },
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
        };

        // Validate and publish
        match odometry.validate() {
            Ok(_) => {
                odom_pub.publish(&odometry).await?;
                info!("✅ Published odometry message #{}", i + 1);
            }
            Err(e) => {
                info!("❌ Odometry validation failed: {}", e);
            }
        }

        sleep(Duration::from_millis(300)).await;
    }

    // ========================================================================
    // Demonstrate Serialization/Deserialization
    // ========================================================================
    
    info!("\n🔄 Testing serialization/deserialization:");
    
    let test_pose = geometry_msgs::PoseStamped {
        header: std_msgs::Header {
            stamp: get_current_time_ns(),
            frame_id: "test_frame".to_string(),
        },
        pose: geometry_msgs::Pose {
            position: geometry_msgs::Point {
                x: 1.5,
                y: 2.5,
                z: 3.5,
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        },
    };

    // Test binary serialization
    let serialized = test_pose.to_bytes()?;
    info!("📦 Serialized pose: {} bytes", serialized.len());

    let deserialized = geometry_msgs::PoseStamped::from_bytes(&serialized)?;
    info!("📦 Deserialized pose: frame='{}', position=({:.1}, {:.1}, {:.1})",
        deserialized.header.frame_id,
        deserialized.pose.position.x,
        deserialized.pose.position.y,
        deserialized.pose.position.z);

    // Wait for message processing
    sleep(Duration::from_millis(300)).await;

    // ========================================================================
    // Summary
    // ========================================================================
    
    let pose_count = received_poses.lock().unwrap().len();
    let odom_count = *received_odoms.lock().unwrap();
    
    info!("\n✅ Cross-language type system demo completed!");
    info!("📊 Results: {} poses and {} odometry messages processed", pose_count, odom_count);
    info!("💡 Key features demonstrated:");
    info!("   • ROS2-compatible message types");
    info!("   • Automatic validation and type safety");
    info!("   • Cross-language serialization");
    info!("   • Standard robotics message packages");
    info!("   • Performance-focused minimal complexity");

    Ok(())
}
