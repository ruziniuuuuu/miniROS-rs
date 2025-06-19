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
    message::{StringMsg, Float64Msg},
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

    // Create publishers for high-performance messaging
    let position_pub = node.create_publisher::<Float64Msg>("/robot/position/x").await?;
    let temperature_pub = node.create_publisher::<Float64Msg>("/sensors/temperature").await?;
    let status_pub = node.create_publisher::<StringMsg>("/telemetry/status").await?;

    // Create a subscriber to demonstrate pub/sub capabilities
    let position_sub = node.create_subscriber::<Float64Msg>("/robot/position/x").await?;

    // Set up callback for demonstration
    position_sub.on_message(|msg: Float64Msg| {
        info!("🤖 [TRANSPORT] Received position: {:.3} m", msg.data);
    })?;

    info!("🏃‍♂️ Starting high-frequency data transmission...");

    // Simulate high-performance robot data transmission
    for i in 0..15 {
        let t = i as f64 * 0.1;
        
        // Publish robot position data
        let x_position = 5.0 * (t * 0.5).cos() + 0.1 * (t * 5.0).sin();
        position_pub.publish(&Float64Msg { data: x_position }).await?;
        info!("📊 [TRANSPORT] Published position: {:.3} m", x_position);

        // Publish temperature data (less frequently)
        if i % 3 == 0 {
            let temperature = 22.0 + 3.0 * (t * 0.3).sin() + fastrand::f64() * 1.0;
            temperature_pub.publish(&Float64Msg { data: temperature }).await?;
            info!("🌡️  [TRANSPORT] Published temperature: {:.2}°C", temperature);
        }

        // Publish status messages
        if i % 5 == 0 {
            let status = match i {
                0 => "🚀 High-performance transport active",
                5 => "📡 Data streaming at high frequency",
                10 => "🏁 Concluding transport demonstration",
                _ => "⚡ System operating efficiently",
            };
            status_pub.publish(&StringMsg { data: status.to_string() }).await?;
            info!("📢 [TRANSPORT] Status: {}", status);
        }

        // Allow some time for message processing
        sleep(Duration::from_millis(200)).await;
    }

    info!("🏆 High-performance transport demonstration completed!");
    info!("📈 Successfully demonstrated efficient pub/sub messaging");
    info!("💡 Next: Try example 06 for advanced visualization");
    
    Ok(())
} 