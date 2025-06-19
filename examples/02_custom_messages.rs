//! Example 02: Custom Message Types
//!
//! This example demonstrates:
//! - Creating custom message types
//! - Serialization and deserialization
//! - Multiple data types in messages
//!
//! Run with: cargo run --example 02_custom_messages

use mini_ros::node::Node;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

// Define a custom robot pose message
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    x: f64,
    y: f64,
    theta: f64,
    timestamp: u64,
}

// Define a custom sensor reading message
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SensorReading {
    sensor_id: String,
    temperature: f32,
    humidity: f32,
    pressure: f32,
}

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 02: Custom Messages ===");

    let mut node = Node::new("robot_node")?;
    node.init().await?;

    // Create publishers for different message types
    let pose_pub = node.create_publisher::<RobotPose>("/robot/pose").await?;
    let sensor_pub = node
        .create_publisher::<SensorReading>("/robot/sensors")
        .await?;

    // Create subscribers
    let pose_sub = node.create_subscriber::<RobotPose>("/robot/pose").await?;
    let sensor_sub = node
        .create_subscriber::<SensorReading>("/robot/sensors")
        .await?;

    // Set up callbacks
    pose_sub.on_message(|pose: RobotPose| {
        info!(
            "🤖 Robot Position: x={:.2}, y={:.2}, θ={:.2}°",
            pose.x,
            pose.y,
            pose.theta.to_degrees()
        );
    })?;

    sensor_sub.on_message(|reading: SensorReading| {
        info!(
            "🌡️  Sensor {}: Temp={:.1}°C, Humidity={:.1}%, Pressure={:.1}hPa",
            reading.sensor_id, reading.temperature, reading.humidity, reading.pressure
        );
    })?;

    info!("🚀 Publishing custom messages...");

    // Simulate robot movement and sensor readings
    for i in 0..8 {
        let t = i as f64 * 0.5;

        // Publish robot pose (moving in a circle)
        let pose = RobotPose {
            x: 3.0 * (t * 0.5).cos(),
            y: 3.0 * (t * 0.5).sin(),
            theta: t * 0.5,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
        };
        pose_pub.publish(&pose).await?;

        // Publish sensor reading every other iteration
        if i % 2 == 0 {
            let sensor = SensorReading {
                sensor_id: format!("ENV_SENSOR_{}", i / 2 + 1),
                temperature: (20.0 + 5.0 * (t * 0.3).sin()) as f32,
                humidity: (50.0 + 10.0 * (t * 0.2).cos()) as f32,
                pressure: (1013.25 + 5.0 * (t * 0.1).sin()) as f32,
            };
            sensor_pub.publish(&sensor).await?;
        }

        sleep(Duration::from_millis(600)).await;
    }

    info!("✅ Custom messages example completed!");
    info!("💡 Next: Try example 03 for service communication");

    Ok(())
}
