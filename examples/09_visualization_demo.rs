//! Minimal Visualization Demo
//!
//! This example demonstrates how to use MiniROS visualization features
//! with Rerun integration for real-time data visualization.

use mini_ros::message::StringMsg;
use mini_ros::{Context, Node};
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== MiniROS Visualization Demo ===");

    // Create context and initialize
    let context = Context::with_domain_id(200)?;
    context.init().await?;

    // Create a node
    let mut node = Node::new("visualization_demo")?;
    println!("Created node: {}", node.name());

    // Create publisher for status messages
    let status_pub = node.create_publisher::<StringMsg>("robot/status").await?;

    // Initialize visualization client (optional feature)
    #[cfg(feature = "visualization")]
    {
        use mini_ros::visualization::{VisualizationClient, VisualizationConfig};

        let config = VisualizationConfig {
            application_id: "visualization_demo".to_string(),
            spawn_viewer: false,
        };
        let viz_client = VisualizationClient::new(config)?;
        println!("Visualization client initialized");

        // Simulate robot data for 10 seconds
        for i in 0..20 {
            let time_sec = i as f64 * 0.5;

            // Log scalar data (battery level)
            let battery_level = 100.0 - (i as f64 * 2.0);
            viz_client.log_scalar("robot/battery", battery_level)?;

            // Log 3D position
            let x = (time_sec * 0.5).sin() * 2.0;
            let y = (time_sec * 0.5).cos() * 2.0;
            let z = 0.5;
            let points = vec![[x as f32, y as f32, z as f32]];
            viz_client.log_points("robot/position", points)?;

            // Log text status
            let status = format!(
                "Step {}: Battery {}%, Position ({:.1}, {:.1}, {:.1})",
                i, battery_level, x, y, z
            );
            viz_client.log_text("robot/status_log", &status)?;

            // Publish status message
            let msg = StringMsg {
                data: status.clone(),
            };
            status_pub.publish(&msg).await?;

            println!("Step {}: {}", i, status);
            sleep(Duration::from_millis(500)).await;
        }

        println!("Visualization demo completed. Check Rerun viewer for data.");
    }

    #[cfg(not(feature = "visualization"))]
    {
        println!(
            "Visualization feature not enabled. Build with --features visualization to see visual output."
        );

        // Still demonstrate basic pub/sub
        for i in 0..5 {
            let msg = StringMsg {
                data: format!("Demo message {} without visualization", i),
            };
            status_pub.publish(&msg).await?;
            println!("Published: {}", msg.data);
            sleep(Duration::from_secs(1)).await;
        }
    }

    // Cleanup
    context.shutdown().await?;
    println!("Demo completed successfully!");

    Ok(())
}
