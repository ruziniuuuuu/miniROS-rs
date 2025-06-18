//! Example 05: High-Performance Communication with Zenoh Transport
//! 
//! This example demonstrates:
//! - Using Zenoh-style transport layer
//! - High-performance pub/sub communication
//! - UDP multicast messaging
//! - Multiple topic communication
//! 
//! Run with: cargo run --example 05_zenoh_transport

use mini_ros::{
    node::Node,
    core::Context,
    message::{StringMsg, Float64Msg, Int32Msg},
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();
    
    info!("=== miniROS-rs Example 05: Zenoh Transport ===");

    // Create context (with built-in high-performance transport)
    let context = Context::new()?;
    
    info!("🚀 High-performance transport initialized!");

    // Create high-performance node  
    let mut node = Node::with_context("zenoh_demo_node", context)?;
    node.init().await?;

    // Create multiple publishers for different data types
    let string_pub = node.create_publisher::<StringMsg>("/telemetry/status").await?;
    let position_pub = node.create_publisher::<Float64Msg>("/robot/position/x").await?;
    let temperature_pub = node.create_publisher::<Float64Msg>("/sensors/temperature").await?;
    let counter_pub = node.create_publisher::<Int32Msg>("/system/counter").await?;

    // Create subscribers
    let status_sub = node.create_subscriber::<StringMsg>("/telemetry/status").await?;
    let pos_sub = node.create_subscriber::<Float64Msg>("/robot/position/x").await?;
    let temp_sub = node.create_subscriber::<Float64Msg>("/sensors/temperature").await?;
    let counter_sub = node.create_subscriber::<Int32Msg>("/system/counter").await?;

    // Set up callbacks for high-speed data processing
    status_sub.on_message(|msg: StringMsg| {
        info!("📡 [HIGH-SPEED] Status: {}", msg.data);
    })?;

    pos_sub.on_message(|msg: Float64Msg| {
        info!("🤖 [HIGH-SPEED] Robot X position: {:.3} m", msg.data);
    })?;

    temp_sub.on_message(|msg: Float64Msg| {
        info!("🌡️  [HIGH-SPEED] Temperature: {:.2}°C", msg.data);
    })?;

    counter_sub.on_message(|msg: Int32Msg| {
        info!("🔢 [HIGH-SPEED] Counter: {}", msg.data);
    })?;

    info!("🏃‍♂️ Starting high-frequency data transmission...");

    // Simulate high-frequency robot data
    for i in 0..20 {
        let t = i as f64 * 0.1;
        
        // Publish robot position (high frequency)
        let x_position = 5.0 * (t * 0.5).cos() + 0.1 * (t * 5.0).sin();
        position_pub.publish(&Float64Msg { data: x_position }).await?;

        // Publish temperature data
        let temperature = 22.0 + 3.0 * (t * 0.3).sin() + fastrand::f64() * 1.0;
        temperature_pub.publish(&Float64Msg { data: temperature }).await?;

        // Publish counter
        counter_pub.publish(&Int32Msg { data: i }).await?;

        // Publish status messages every 5 iterations
        if i % 5 == 0 {
            let status = match i {
                0 => "🚀 Zenoh transport initialized",
                5 => "⚡ High-speed transmission active",
                10 => "📊 Data streaming optimally",
                15 => "🏁 Preparing to conclude",
                _ => "📡 System operational",
            };
            string_pub.publish(&StringMsg { data: status.to_string() }).await?;
        }

        // High-frequency publishing (10Hz)
        sleep(Duration::from_millis(100)).await;
    }

    info!("🏆 High-performance communication test completed!");
    info!("📈 Achieved high-frequency pub/sub with Zenoh-style transport");
    info!("💡 Next: Try example 06 for advanced visualization");
    
    Ok(())
} 