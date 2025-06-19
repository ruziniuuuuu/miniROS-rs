//! Example 06: Visualization with 3D Data
//!
//! This example demonstrates:
//! - 3D robot pose visualization
//! - Point cloud data visualization
//! - Laser scan visualization
//! - Real-time data logging with Rerun
//!
//! Note: Rerun viewer may not launch automatically - data is logged to memory
//! Run with: cargo run --example 06_visualization_advanced

use mini_ros::visualization::{
    LaserScan, PointCloud, RobotPose, Visualizable, VisualizationClient, VisualizationConfig,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 06: Visualization ===");

    // Create visualization client with memory recording (more reliable)
    info!("🖥️  Starting visualization logging...");
    let config = VisualizationConfig {
        application_id: "miniROS_Visualization".to_string(),
        spawn_viewer: false, // Use memory recording for reliability
    };

    let viz_client = VisualizationClient::new(config)?;
    info!("✅ Visualization client initialized!");

    info!("🚀 Starting 3D visualization demonstration...");

    // Simulate robot mission with 3D data
    for i in 0..20 {
        let t = i as f64 * 0.15;

        // === Robot Trajectory ===
        // Robot moves in a complex 3D path
        let x = 4.0 * (t * 0.3).cos() * (1.0 + 0.3 * (t * 0.7).sin());
        let y = 4.0 * (t * 0.3).sin() * (1.0 + 0.3 * (t * 0.7).cos());
        let z = 0.5 + 0.5 * (t * 0.4).sin();
        let yaw = t * 0.3 + 0.5 * (t * 0.8).sin();

        let robot_pose = RobotPose {
            position: [x as f32, y as f32, z as f32],
            orientation: [0.0, 0.0, (yaw / 2.0).sin() as f32, (yaw / 2.0).cos() as f32], // Quaternion from yaw
        };
        robot_pose.visualize(&viz_client, "robot/pose")?;

        // === Point Cloud (3D sensor data) ===
        // Generate a realistic point cloud around the robot
        if i % 3 == 0 {
            // Update every 3rd frame for performance
            let mut points = Vec::new();
            let mut colors = Vec::new();

            // Generate points in a 360-degree scan pattern
            for angle_deg in (0..360).step_by(5) {
                let angle = angle_deg as f64 * std::f64::consts::PI / 180.0;
                let distance = 3.0 + 2.0 * (angle * 2.0).sin() + fastrand::f64() * 0.5;

                // Point relative to robot
                let px = x + distance * (yaw + angle).cos();
                let py = y + distance * (yaw + angle).sin();
                let pz = z + 0.2 * (angle * 5.0).sin();

                points.extend_from_slice(&[px as f32, py as f32, pz as f32]);

                // Color based on distance (blue = close, red = far)
                let color_ratio = (distance - 1.0) / 4.0;
                colors.extend_from_slice(&[
                    (255.0 * color_ratio) as u8,         // Red
                    50,                                  // Green
                    (255.0 * (1.0 - color_ratio)) as u8, // Blue
                ]);
            }

            // Convert flattened points to array of 3D points
            let mut point_cloud_data = Vec::new();
            for chunk in points.chunks(3) {
                if chunk.len() == 3 {
                    point_cloud_data.push([chunk[0], chunk[1], chunk[2]]);
                }
            }

            let point_cloud = PointCloud {
                points: point_cloud_data,
            };
            point_cloud.visualize(&viz_client, "sensors/lidar/points")?;
        }

        // === Laser Scan (2D) ===
        if i % 2 == 0 {
            // Update every 2nd frame
            let mut ranges = Vec::new();
            let mut intensities = Vec::new();

            for angle_deg in (0..360).step_by(2) {
                let angle = angle_deg as f64 * std::f64::consts::PI / 180.0;
                let range = 5.0 + 2.0 * (angle * 3.0).cos() + fastrand::f64() * 0.3;
                let intensity = 100.0 + 50.0 * (angle * 7.0).sin();

                ranges.push(range as f32);
                intensities.push(intensity as u8);
            }

            let laser_scan = LaserScan {
                ranges,
                angle_min: 0.0,
                angle_max: 2.0 * std::f64::consts::PI as f32,
            };
            laser_scan.visualize(&viz_client, "sensors/laser/scan")?;
        }

        // === Additional 3D Data ===
        // Log waypoints
        if i % 8 == 0 {
            let waypoint_x = x + 2.0 * (t * 0.2).cos();
            let waypoint_y = y + 2.0 * (t * 0.2).sin();
            viz_client.log_points(
                "navigation/waypoint",
                vec![[waypoint_x as f32, waypoint_y as f32, (z + 1.0) as f32]],
            )?;
        }

        // === System Status ===
        // Log various scalar metrics
        let battery = 100.0 - (t * 2.0).max(0.0);
        viz_client.log_scalar("robot/battery_percentage", battery)?;

        let cpu_load = 40.0 + 20.0 * (t * 1.5).sin() + fastrand::f64() * 15.0;
        viz_client.log_scalar("system/cpu_load", cpu_load.clamp(0.0, 100.0))?;

        let wifi_signal = -45.0 + 10.0 * (t * 0.5).cos() + fastrand::f64() * 5.0;
        viz_client.log_scalar("network/wifi_signal_dbm", wifi_signal)?;

        // === Status Messages ===
        if i % 10 == 0 {
            let status = match i {
                0 => "🚀 Mission started: Autonomous exploration",
                10 => "🔍 Scanning environment with LiDAR",
                20 => "🗺️  Building 3D map of surroundings",
                30 => "🎯 Navigation to target coordinates",
                _ => "📡 All systems operational",
            };
            viz_client.log_text("mission/status", status)?;
        }

        info!(
            "🤖 Step {}: Robot at ({:.2}, {:.2}, {:.2}), Battery: {:.1}%, CPU: {:.0}%",
            i + 1,
            x,
            y,
            z,
            battery,
            cpu_load
        );

        sleep(Duration::from_millis(300)).await;
    }

    // Mission complete
    viz_client.log_text("mission/status", "🎉 Mission completed successfully!")?;
    viz_client.log_scalar("robot/mission_progress", 100.0)?;

    info!("✅ 3D visualization demonstration completed!");
    info!("📊 Successfully logged robot trajectory, point clouds, and sensor data");
    info!("💡 Next: Try example 07 for the complete integrated system");

    Ok(())
}
