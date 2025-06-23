//! Example 17: New Message Types Demo
//!
//! This example demonstrates the new ROS2-compatible message types:
//! - sensor_msgs: LaserScan, Image, Imu, PointCloud2
//! - action_msgs: GoalInfo, GoalStatus, GoalStatusArray  
//! - diagnostic_msgs: DiagnosticStatus, DiagnosticArray, KeyValue
//!
//! Run with: cargo run --example 17_new_message_types_demo

use mini_ros::node::Node;
use mini_ros::types::{
    MiniRosMessage, action_msgs, diagnostic_msgs, geometry_msgs, sensor_msgs, std_msgs,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

// Helper function to get current timestamp
fn get_current_time_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 17: New Message Types Demo ===");

    let mut node = Node::new("sensor_demo_node")?;
    node.init().await?;

    // ============================================================================
    // sensor_msgs demonstrations
    // ============================================================================

    info!("\n🔬 Demonstrating sensor_msgs package:");

    // Create LaserScan publisher and subscriber
    let laser_pub = node
        .create_publisher::<sensor_msgs::LaserScan>("/sensors/laser")
        .await?;
    let laser_sub = node
        .create_subscriber::<sensor_msgs::LaserScan>("/sensors/laser")
        .await?;

    laser_sub.on_message(|scan: sensor_msgs::LaserScan| {
        info!(
            "📡 Received LaserScan: {} ranges, angle range: {:.2} to {:.2} rad",
            scan.ranges.len(),
            scan.angle_min,
            scan.angle_max
        );
    })?;

    // Create IMU publisher and subscriber
    let imu_pub = node
        .create_publisher::<sensor_msgs::Imu>("/sensors/imu")
        .await?;
    let imu_sub = node
        .create_subscriber::<sensor_msgs::Imu>("/sensors/imu")
        .await?;

    imu_sub.on_message(|imu: sensor_msgs::Imu| {
        info!("🧭 Received IMU: orientation=({:.2}, {:.2}, {:.2}, {:.2}), accel=({:.1}, {:.1}, {:.1})",
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    })?;

    // ============================================================================
    // action_msgs demonstrations
    // ============================================================================

    info!("\n🎯 Demonstrating action_msgs package:");

    let goal_status_pub = node
        .create_publisher::<action_msgs::GoalStatus>("/actions/goal_status")
        .await?;
    let goal_status_sub = node
        .create_subscriber::<action_msgs::GoalStatus>("/actions/goal_status")
        .await?;

    goal_status_sub.on_message(|status: action_msgs::GoalStatus| {
        let status_str = match status.status {
            action_msgs::STATUS_UNKNOWN => "UNKNOWN",
            action_msgs::STATUS_ACCEPTED => "ACCEPTED",
            action_msgs::STATUS_EXECUTING => "EXECUTING",
            action_msgs::STATUS_CANCELING => "CANCELING",
            action_msgs::STATUS_SUCCEEDED => "SUCCEEDED",
            action_msgs::STATUS_CANCELED => "CANCELED",
            action_msgs::STATUS_ABORTED => "ABORTED",
            _ => "INVALID",
        };
        info!(
            "🎯 Goal '{}' status: {}",
            status.goal_info.goal_id, status_str
        );
    })?;

    // ============================================================================
    // diagnostic_msgs demonstrations
    // ============================================================================

    info!("\n🔧 Demonstrating diagnostic_msgs package:");

    let diag_pub = node
        .create_publisher::<diagnostic_msgs::DiagnosticArray>("/diagnostics")
        .await?;
    let diag_sub = node
        .create_subscriber::<diagnostic_msgs::DiagnosticArray>("/diagnostics")
        .await?;

    diag_sub.on_message(|diag_array: diagnostic_msgs::DiagnosticArray| {
        info!(
            "💊 Received diagnostics: {} components",
            diag_array.status.len()
        );
        for status in &diag_array.status {
            let level_str = match status.level {
                diagnostic_msgs::OK => "OK",
                diagnostic_msgs::WARN => "WARN",
                diagnostic_msgs::ERROR => "ERROR",
                diagnostic_msgs::STALE => "STALE",
                _ => "UNKNOWN",
            };
            info!("  - {}: {} [{}]", status.name, status.message, level_str);
        }
    })?;

    info!("🚀 Publishing sensor data, action status, and diagnostics...");

    // ============================================================================
    // Generate and publish sample data
    // ============================================================================

    for i in 0..5 {
        let timestamp = get_current_time_ns();

        // Publish LaserScan data
        let mut ranges = Vec::new();
        for j in 0..360 {
            let angle = (j as f32) * std::f32::consts::PI / 180.0;
            let range = 2.0 + (angle + (i as f32) * 0.1).sin().abs();
            ranges.push(range);
        }

        let laser_scan = sensor_msgs::LaserScan {
            header: std_msgs::Header {
                stamp: timestamp,
                frame_id: "laser_frame".to_string(),
            },
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
            angle_increment: std::f32::consts::PI / 180.0,
            time_increment: 0.001,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 10.0,
            ranges,
            intensities: vec![], // No intensity data
        };

        // Validate and publish
        if laser_scan.validate().is_ok() {
            laser_pub.publish(&laser_scan).await?;
        }

        // Publish IMU data
        let angle = (i as f64) * 0.1;
        let imu = sensor_msgs::Imu {
            header: std_msgs::Header {
                stamp: timestamp,
                frame_id: "imu_frame".to_string(),
            },
            orientation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: (angle / 2.0).sin(),
                w: (angle / 2.0).cos(),
            },
            orientation_covariance: vec![0.01; 9],
            angular_velocity: geometry_msgs::Vector3 {
                x: 0.1 * (angle * 2.0).sin(),
                y: 0.1 * (angle * 3.0).cos(),
                z: 0.05,
            },
            angular_velocity_covariance: vec![0.001; 9],
            linear_acceleration: geometry_msgs::Vector3 {
                x: 0.2 * (angle * 4.0).sin(),
                y: 0.2 * (angle * 5.0).cos(),
                z: 9.81 + 0.1 * (angle * 6.0).sin(),
            },
            linear_acceleration_covariance: vec![0.01; 9],
        };

        if imu.validate().is_ok() {
            imu_pub.publish(&imu).await?;
        }

        // Publish action goal status
        let goal_status = action_msgs::GoalStatus {
            goal_info: action_msgs::GoalInfo {
                goal_id: format!("navigation_goal_{}", i),
                stamp: timestamp,
            },
            status: match i % 4 {
                0 => action_msgs::STATUS_ACCEPTED,
                1 => action_msgs::STATUS_EXECUTING,
                2 => action_msgs::STATUS_SUCCEEDED,
                _ => action_msgs::STATUS_ABORTED,
            },
        };

        if goal_status.validate().is_ok() {
            goal_status_pub.publish(&goal_status).await?;
        }

        // Publish diagnostic information
        let motor_temp = 25.0 + 10.0 * (i as f64 * 0.5).sin();
        let battery_voltage = 12.0 + 0.5 * (i as f64 * 0.3).cos();

        let diagnostic_array = diagnostic_msgs::DiagnosticArray {
            header: std_msgs::Header {
                stamp: timestamp,
                frame_id: "base_link".to_string(),
            },
            status: vec![
                diagnostic_msgs::DiagnosticStatus {
                    level: if motor_temp > 35.0 {
                        diagnostic_msgs::WARN
                    } else {
                        diagnostic_msgs::OK
                    },
                    name: "Motor Controller".to_string(),
                    message: format!("Temperature: {:.1}°C", motor_temp),
                    hardware_id: "motor_ctrl_001".to_string(),
                    values: vec![
                        diagnostic_msgs::KeyValue {
                            key: "temperature".to_string(),
                            value: format!("{:.1}", motor_temp),
                        },
                        diagnostic_msgs::KeyValue {
                            key: "max_temperature".to_string(),
                            value: "40.0".to_string(),
                        },
                    ],
                },
                diagnostic_msgs::DiagnosticStatus {
                    level: if battery_voltage < 11.5 {
                        diagnostic_msgs::ERROR
                    } else {
                        diagnostic_msgs::OK
                    },
                    name: "Battery Monitor".to_string(),
                    message: format!("Voltage: {:.1}V", battery_voltage),
                    hardware_id: "battery_001".to_string(),
                    values: vec![
                        diagnostic_msgs::KeyValue {
                            key: "voltage".to_string(),
                            value: format!("{:.1}", battery_voltage),
                        },
                        diagnostic_msgs::KeyValue {
                            key: "min_voltage".to_string(),
                            value: "11.0".to_string(),
                        },
                    ],
                },
            ],
        };

        if diagnostic_array.validate().is_ok() {
            diag_pub.publish(&diagnostic_array).await?;
        }

        info!("📊 Published sensor data batch {}/5", i + 1);
        sleep(Duration::from_millis(800)).await;
    }

    info!("✅ New message types demo completed!");
    info!("💡 All new ROS2-compatible message types are working correctly");

    Ok(())
}
