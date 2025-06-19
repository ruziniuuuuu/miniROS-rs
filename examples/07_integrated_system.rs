//! Example 07: Integrated System
//!
//! This example demonstrates:
//! - Multiple nodes communicating
//! - Pub/sub + visualization integration
//! - Simplified robot system simulation
//!
//! Run with: cargo run --example 07_integrated_system

use mini_ros::{
    message::{Float64Msg, StringMsg},
    node::Node,
    visualization::{
        LaserScan, PointCloud, RobotPose, Visualizable, VisualizationClient, VisualizationConfig,
    },
};

use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 07: Integrated System ===");

    // Step 1: Start visualization
    info!("🖥️  Starting visualization...");
    let config = VisualizationConfig {
        application_id: "IntegratedSystem".to_string(),
        spawn_viewer: false, // Use memory recording for reliability
    };
    let viz_client = VisualizationClient::new(config)?;
    info!("✅ Visualization initialized!");

    // Step 2: Initialize integrated system node
    info!("🤖 Initializing integrated system node...");
    let mut robot_node = Node::new("integrated_system")?;
    robot_node.init().await?;
    info!("✅ Integrated system node ready!");

    // Step 3: Create communication channels
    info!("📡 Setting up communication...");

    // Publishers for integrated system
    let pose_pub = robot_node
        .create_publisher::<StringMsg>("/robot/pose")
        .await?;
    let sensor_pub = robot_node
        .create_publisher::<Float64Msg>("/robot/sensors/lidar")
        .await?;
    let status_pub = robot_node
        .create_publisher::<StringMsg>("/mission/status")
        .await?;

    info!("✅ Communication channels established!");

    // Step 4: Start integrated mission
    info!("🚀 Starting integrated mission...");

    // Mission parameters
    let mut robot_x = 0.0;
    let mut robot_y = 0.0;
    let mut robot_yaw = 0.0;
    let waypoints = vec![
        [2.0, 0.0],
        [2.0, 2.0],
        [0.0, 2.0],
        [-2.0, 2.0],
        [-2.0, 0.0],
        [0.0, 0.0],
    ];

    let mut current_waypoint = 0;
    let total_steps = 50; // Shorter demo

    for step in 0..total_steps {
        let t = step as f64 / 10.0;

        // Mission control logic
        if step % 20 == 0 && current_waypoint < waypoints.len() {
            let status = format!(
                "🎯 Heading to waypoint {} at ({:.1}, {:.1})",
                current_waypoint + 1,
                waypoints[current_waypoint][0],
                waypoints[current_waypoint][1]
            );

            status_pub
                .publish(&StringMsg {
                    data: status.clone(),
                })
                .await?;
            viz_client.log_text("mission/current_objective", &status)?;
            info!("{}", status);
        }

        // Robot movement simulation
        if current_waypoint < waypoints.len() {
            let target_x = waypoints[current_waypoint][0];
            let target_y = waypoints[current_waypoint][1];

            let dx = target_x - robot_x;
            let dy = target_y - robot_y;
            let distance = ((dx * dx + dy * dy) as f64).sqrt();

            if distance > 0.1 {
                // Move towards target
                let speed = 0.05;
                robot_x += dx / distance * speed;
                robot_y += dy / distance * speed;
                robot_yaw = (dy as f64).atan2(dx as f64);
            } else {
                // Reached waypoint
                current_waypoint += 1;
                info!("✅ Reached waypoint {}", current_waypoint);
            }
        } else {
            // All waypoints reached, do exploration pattern
            robot_x = 2.0 * (t * 0.3).cos();
            robot_y = 1.0 * (t * 0.6).sin();
            robot_yaw = t * 0.2;
        }

        // Publish robot pose
        let pose_msg = format!(
            "{{\"x\": {:.3}, \"y\": {:.3}, \"yaw\": {:.3}}}",
            robot_x, robot_y, robot_yaw
        );
        pose_pub.publish(&StringMsg { data: pose_msg }).await?;

        // Visualize robot pose
        let (sin_yaw, cos_yaw) = (robot_yaw / 2.0).sin_cos();
        let robot_pose = RobotPose {
            position: [robot_x as f32, robot_y as f32, 0.1],
            orientation: [0.0, 0.0, sin_yaw as f32, cos_yaw as f32],
        };
        robot_pose.visualize(&viz_client, "robot/pose")?;

        // Simulate LiDAR sensor data
        let mut lidar_points = Vec::new();
        let mut lidar_ranges = Vec::new();

        for i in 0..360 {
            let angle = (i as f64).to_radians() + robot_yaw;
            let base_range = 3.0 + fastrand::f64() * 2.0;

            // Add obstacles
            let range = if i % 72 == 0 {
                1.0 + fastrand::f64() * 0.5 // Close obstacle
            } else {
                base_range
            };

            lidar_ranges.push(range as f32);

            // Convert to 3D point
            let px = robot_x + range * angle.cos();
            let py = robot_y + range * angle.sin();
            let pz = 0.1 + fastrand::f64() * 0.1;
            lidar_points.push([px as f32, py as f32, pz as f32]);
        }

        // Publish sensor data
        let avg_range = lidar_ranges.iter().sum::<f32>() / lidar_ranges.len() as f32;
        sensor_pub
            .publish(&Float64Msg {
                data: avg_range as f64,
            })
            .await?;

        // Visualize LiDAR data
        let point_cloud = PointCloud {
            points: lidar_points.clone(),
        };
        point_cloud.visualize(&viz_client, "sensors/lidar_points")?;

        let laser_scan = LaserScan {
            ranges: lidar_ranges,
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
        };
        laser_scan.visualize(&viz_client, "sensors/laser_scan")?;

        // Log system metrics
        viz_client.log_scalar("robot/battery", 100.0 - t * 0.5)?;
        viz_client.log_scalar("robot/cpu_usage", 20.0 + 10.0 * (t * 0.1).sin())?;
        viz_client.log_scalar(
            "mission/progress",
            (step as f64 / total_steps as f64) * 100.0,
        )?;
        viz_client.log_scalar("sensors/avg_distance", avg_range as f64)?;

        // Mission phases
        let phase = match step {
            0..=30 => "🚀 Navigation Phase",
            31..=60 => "🔍 Exploration Phase",
            61..=80 => "📊 Data Collection Phase",
            _ => "🏠 Return Phase",
        };

        if step % 10 == 0 {
            viz_client.log_text("mission/phase", phase)?;
        }

        if step % 5 == 0 {
            info!(
                "🤖 Robot at ({:.2}, {:.2}) | Phase: {} | Progress: {:.1}%",
                robot_x,
                robot_y,
                phase,
                (step as f64 / total_steps as f64) * 100.0
            );
        }

        sleep(Duration::from_millis(100)).await;
    }

    // Mission complete
    let completion_msg = "✅ Integrated mission completed successfully!";
    status_pub
        .publish(&StringMsg {
            data: completion_msg.to_string(),
        })
        .await?;
    viz_client.log_text("mission/status", completion_msg)?;
    viz_client.log_scalar("mission/progress", 100.0)?;

    info!("🎉 Integration demonstration complete!");
    info!("📊 This example showed:");
    info!("   • Multiple node coordination");
    info!("   • Publisher/subscriber communication");
    info!("   • Real-time 3D visualization");
    info!("   • Sensor data simulation");
    info!("   • Mission control logic");

    info!("🎓 Next steps: Explore the visualization in Rerun!");
    info!("💡 You've now seen all major features of miniROS-rs working together!");

    // Keep visualization open
    info!("⏳ Keeping visualization open for 20 seconds...");
    sleep(Duration::from_secs(20)).await;

    Ok(())
}
