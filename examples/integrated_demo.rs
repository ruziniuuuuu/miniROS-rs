//! Integrated demo combining Zenoh communication and Rerun visualization
//! 
//! Shows how to use both high-performance communication and real-time visualization.

use mini_ros::zenoh_transport::{ZenohConfig, ZenohTransport};
use mini_ros::visualization::{VisualizationClient, VisualizationConfig, RobotPose, Visualizable};
use mini_ros::message::{StringMsg, Float64Msg};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();
    
    info!("Starting integrated Zenoh + Rerun demo");

    // Setup Zenoh transport
    let zenoh_config = ZenohConfig::default();
    let transport = ZenohTransport::new(zenoh_config).await?;
    
    // Setup Rerun visualization
    let viz_config = VisualizationConfig::default();
    let viz_client = VisualizationClient::new(viz_config)?;

    // Create communication channels
    let pose_topic = "/robot/pose";
    let status_topic = "/robot/status";
    let sensor_topic = "/robot/sensor_data";
    
    let pose_pub = transport.create_publisher::<RobotPose>(pose_topic).await?;
    let status_pub = transport.create_publisher::<StringMsg>(status_topic).await?;
    let sensor_pub = transport.create_publisher::<Float64Msg>(sensor_topic).await?;

    // Create subscribers with visualization callbacks
    let pose_sub = transport.create_subscriber::<RobotPose>(pose_topic).await?;
    let status_sub = transport.create_subscriber::<StringMsg>(status_topic).await?;
    let sensor_sub = transport.create_subscriber::<Float64Msg>(sensor_topic).await?;

    // Setup visualization callbacks
    let viz_client_clone = viz_client.clone();
    pose_sub.on_message(move |pose: RobotPose| {
        info!("Received pose: {:?}", pose);
        if let Err(e) = pose.visualize(&viz_client_clone, "robot/pose") {
            tracing::warn!("Failed to visualize pose: {}", e);
        }
    })?;

    let viz_client_clone = viz_client.clone();
    status_sub.on_message(move |msg: StringMsg| {
        info!("Status update: {}", msg.data);
        if let Err(e) = viz_client_clone.log_text("robot/status", &msg.data) {
            tracing::warn!("Failed to log status: {}", e);
        }
    })?;

    let viz_client_clone = viz_client.clone();
    sensor_sub.on_message(move |msg: Float64Msg| {
        info!("Sensor data: {}", msg.data);
        if let Err(e) = viz_client_clone.log_scalar("sensors/temperature", msg.data) {
            tracing::warn!("Failed to log sensor data: {}", e);
        }
    })?;

    // Give time for setup
    sleep(Duration::from_millis(100)).await;

    info!("Running integrated demo...");
    
    // Simulate robot operation
    for i in 0..50 {
        let t = i as f32 * 0.2;
        
        // Publish robot pose
        let pose = RobotPose {
            position: [t.cos() * 3.0, t.sin() * 3.0, 0.1 * t],
            orientation: [0.0, 0.0, (t * 0.5).sin(), (t * 0.5).cos()],
        };
        pose_pub.publish(&pose).await?;

        // Publish status every 5 steps
        if i % 5 == 0 {
            let status = StringMsg {
                data: format!("Robot operational - Step {}", i),
            };
            status_pub.publish(&status).await?;
        }

        // Publish sensor data
        let sensor_data = Float64Msg {
            data: 25.0 + 3.0 * (t as f64 * 0.3).sin() + fastrand::f64() * 0.5,
        };
        sensor_pub.publish(&sensor_data).await?;

        sleep(Duration::from_millis(200)).await;
    }

    // Final status
    let final_status = StringMsg {
        data: "Demo completed successfully".to_string(),
    };
    status_pub.publish(&final_status).await?;

    // Wait for final messages to be processed
    sleep(Duration::from_secs(2)).await;

    // Cleanup
    transport.shutdown().await?;
    info!("Integrated demo completed");
    
    Ok(())
} 