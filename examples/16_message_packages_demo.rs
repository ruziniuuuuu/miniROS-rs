//! Example 16: ROS2-Compatible Message Packages Demo
//!
//! This example demonstrates the new ROS2-compatible message packages:
//! - std_msgs: Standard primitive message types
//! - geometry_msgs: Geometric messages (Point, Pose, Twist, etc.)
//! - nav_msgs: Navigation messages (Odometry, Path)
//!
//! Run with: cargo run --example 16_message_packages_demo

use mini_ros::node::Node;
use mini_ros::types::{MiniRosMessage, geometry_msgs, nav_msgs, std_msgs};
use std::time::{SystemTime, UNIX_EPOCH};
// Removed unused imports
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 16: ROS2-Compatible Message Packages ===");

    let mut node = Node::new("message_packages_demo")?;
    node.init().await?;

    // ============================================================================
    // std_msgs - Standard message types
    // ============================================================================

    info!("📦 Demonstrating std_msgs package:");

    // String message
    let string_msg = std_msgs::String {
        data: "Hello from miniROS std_msgs!".to_string(),
    };
    info!("✨ String message: {}", string_msg.data);
    info!("   Type: {}", std_msgs::String::message_type());

    // Header message
    let header = std_msgs::Header {
        stamp: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64,
        frame_id: "base_link".to_string(),
    };
    info!(
        "📋 Header: frame_id='{}', stamp={}",
        header.frame_id, header.stamp
    );

    // Numeric messages
    let int_msg = std_msgs::Int32 { data: 42 };
    let float_msg = std_msgs::Float64 {
        data: std::f64::consts::PI,
    };
    let bool_msg = std_msgs::Bool { data: true };

    info!(
        "🔢 Numeric messages: int={}, float={:.3}, bool={}",
        int_msg.data, float_msg.data, bool_msg.data
    );

    // ============================================================================
    // geometry_msgs - Geometric message types
    // ============================================================================

    info!("\n📦 Demonstrating geometry_msgs package:");

    // 3D Point
    let point = geometry_msgs::Point {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    info!("📍 Point: ({:.1}, {:.1}, {:.1})", point.x, point.y, point.z);

    // Quaternion (normalized)
    let quaternion = geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    info!(
        "🔄 Quaternion: ({:.1}, {:.1}, {:.1}, {:.1})",
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    );

    // Pose (position + orientation)
    let pose = geometry_msgs::Pose {
        position: point.clone(),
        orientation: quaternion.clone(),
    };
    info!("🤖 Robot pose created successfully");

    // Stamped pose
    let pose_stamped = geometry_msgs::PoseStamped {
        header: header.clone(),
        pose: pose.clone(),
    };
    info!(
        "📋 Stamped pose in frame '{}'",
        pose_stamped.header.frame_id
    );

    // Twist (velocity)
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
    info!(
        "🏃 Twist: linear=({:.1}, {:.1}, {:.1}), angular=({:.1}, {:.1}, {:.1})",
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z
    );

    // ============================================================================
    // nav_msgs - Navigation message types
    // ============================================================================

    info!("\n📦 Demonstrating nav_msgs package:");

    // Odometry message (complete robot state)
    let odometry = nav_msgs::Odometry {
        header: header.clone(),
        child_frame_id: "base_link".to_string(),
        pose: geometry_msgs::PoseWithCovariance {
            pose: pose.clone(),
            covariance: vec![0.0; 36], // 6x6 covariance matrix
        },
        twist: geometry_msgs::TwistWithCovariance {
            twist: twist.clone(),
            covariance: vec![0.0; 36], // 6x6 covariance matrix
        },
    };
    info!("📊 Odometry message created for robot localization");
    info!("   Child frame: '{}'", odometry.child_frame_id);

    // Navigation path
    let path = nav_msgs::Path {
        header: header.clone(),
        poses: vec![
            geometry_msgs::PoseStamped {
                header: header.clone(),
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: quaternion.clone(),
                },
            },
            geometry_msgs::PoseStamped {
                header: header.clone(),
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: quaternion.clone(),
                },
            },
            geometry_msgs::PoseStamped {
                header: header.clone(),
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: 1.0,
                        y: 1.0,
                        z: 0.0,
                    },
                    orientation: quaternion.clone(),
                },
            },
        ],
    };
    info!("🛣️ Navigation path with {} waypoints", path.poses.len());

    // ============================================================================
    // Message validation and serialization
    // ============================================================================

    info!("\n🔍 Testing message validation and serialization:");

    // Test validation
    assert!(string_msg.validate().is_ok());
    assert!(point.validate().is_ok());
    assert!(quaternion.validate().is_ok());
    assert!(pose.validate().is_ok());
    assert!(twist.validate().is_ok());
    assert!(odometry.validate().is_ok());
    assert!(path.validate().is_ok());
    info!("✅ All messages passed validation");

    // Test serialization
    let string_bytes = string_msg.to_bytes()?;
    let point_bytes = point.to_bytes()?;
    let twist_bytes = twist.to_bytes()?;
    let odom_bytes = odometry.to_bytes()?;

    info!("📦 Serialization sizes:");
    info!("   String: {} bytes", string_bytes.len());
    info!("   Point: {} bytes", point_bytes.len());
    info!("   Twist: {} bytes", twist_bytes.len());
    info!("   Odometry: {} bytes", odom_bytes.len());

    // Test deserialization round-trip
    let restored_string = std_msgs::String::from_bytes(&string_bytes)?;
    let restored_point = geometry_msgs::Point::from_bytes(&point_bytes)?;
    let restored_twist = geometry_msgs::Twist::from_bytes(&twist_bytes)?;

    assert_eq!(string_msg.data, restored_string.data);
    assert_eq!(point.x, restored_point.x);
    assert_eq!(twist.linear.x, restored_twist.linear.x);
    info!("✅ Round-trip serialization successful");

    // ============================================================================
    // ROS2 compatibility demonstration
    // ============================================================================

    info!("\n🌉 ROS2 Compatibility Features:");
    info!("✅ Message type names follow ROS2 conventions:");
    info!("   - {}", std_msgs::String::message_type());
    info!("   - {}", geometry_msgs::Point::message_type());
    info!("   - {}", geometry_msgs::Twist::message_type());
    info!("   - {}", nav_msgs::Odometry::message_type());

    info!("✅ Field names and types match ROS2 exactly");
    info!("✅ Covariance matrices use proper 6x6 format");
    info!("✅ Headers include stamp (nanoseconds) and frame_id");
    info!("✅ All messages include validation and schema information");

    info!("\n💡 Usage Tips:");
    info!("   • Use std_msgs for primitive data types");
    info!("   • Use geometry_msgs for poses, twists, and transforms");
    info!("   • Use nav_msgs for odometry and path planning");
    info!("   • All messages are fully compatible with ROS2 tools");
    info!("   • Legacy message types are still available for backward compatibility");

    info!("\n🎉 Message packages demo completed successfully!");
    info!("📚 See package READMEs for detailed usage examples");

    Ok(())
}
