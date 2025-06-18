//! Rerun visualization demo for miniROS
//! 
//! Demonstrates data visualization capabilities using Rerun viewer.

use mini_ros::visualization::{
    VisualizationClient, VisualizationConfig, 
    RobotPose, PointCloud, LaserScan, Visualizable
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();
    
    info!("Starting Rerun visualization demo");

    // Create visualization client (using buffered mode to avoid needing external viewer)
    let config = VisualizationConfig {
        application_id: "miniROS_demo".to_string(),
        spawn_viewer: false,  // Use buffered mode instead of spawning viewer
    };
    
    let viz_client = VisualizationClient::new(config)?;
    info!("Visualization client created (buffered mode)");

    // Simulate robot moving in a circle
    info!("Visualizing robot trajectory...");
    for i in 0..100 {
        let t = i as f32 * 0.1;
        
        // Robot pose (circular trajectory)
        let robot_pose = RobotPose {
            position: [
                2.0 * (t * 0.5).cos(),
                2.0 * (t * 0.5).sin(), 
                0.0
            ],
            orientation: [0.0, 0.0, (t * 0.5).sin(), (t * 0.5).cos()],
        };
        
        robot_pose.visualize(&viz_client, "robot/pose")?;

        // Simulate laser scan data
        let mut ranges = Vec::new();
        for j in 0..360 {
            let angle = (j as f32).to_radians();
            let noise = 0.1 * ((t + angle) * 10.0).sin();
            let range = 3.0 + noise;
            ranges.push(range);
        }
        
        let laser_scan = LaserScan {
            ranges,
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
        };
        
        laser_scan.visualize(&viz_client, "robot/laser_scan")?;

        // Generate some random point cloud
        let mut points = Vec::new();
        for _ in 0..100 {
            points.push([
                4.0 * (t + fastrand::f32()).cos(),
                4.0 * (t + fastrand::f32()).sin(),
                fastrand::f32() * 2.0 - 1.0,
            ]);
        }
        
        let point_cloud = PointCloud { points };
        point_cloud.visualize(&viz_client, "environment/obstacles")?;

        // Log scalar data
        viz_client.log_scalar("sensors/velocity", (t * 0.5).sin() as f64 + 1.0)?;
        viz_client.log_scalar("sensors/temperature", 20.0 + 5.0 * (t * 0.1).sin() as f64)?;

        // Log text message
        if i % 10 == 0 {
            viz_client.log_text("robot/status", &format!("Step {}, time: {:.1}s", i, t))?;
        }

        sleep(Duration::from_millis(50)).await;
    }

    // Keep visualization open for a while
    info!("Demo completed. Visualization will remain open for 10 seconds...");
    sleep(Duration::from_secs(10)).await;

    info!("Visualization demo finished");
    Ok(())
} 