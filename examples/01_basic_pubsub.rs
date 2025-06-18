//! Example 01: Basic Publisher-Subscriber Communication
//! 
//! This is your first miniROS-rs example. It demonstrates:
//! - Creating nodes
//! - Basic pub/sub communication
//! - Using built-in message types
//! 
//! Run with: cargo run --example 01_basic_pubsub

use mini_ros::{
    node::Node,
    message::StringMsg,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    
    info!("=== miniROS-rs Example 01: Basic Pub/Sub ===");

    // Create and initialize node
    let mut node = Node::new("basic_node")?;
    node.init().await?;

    // Create publisher and subscriber for the same topic
    let publisher = node.create_publisher::<StringMsg>("/greetings").await?;
    let subscriber = node.create_subscriber::<StringMsg>("/greetings").await?;

    // Set up message callback
    subscriber.on_message(|msg: StringMsg| {
        info!("📨 Received: {}", msg.data);
    })?;

    info!("🚀 Starting pub/sub communication...");

    // Publish messages in a loop
    for i in 0..5 {
        let message = StringMsg {
            data: format!("Hello miniROS-rs! Message #{}", i + 1),
        };
        
        publisher.publish(&message).await?;
        info!("📤 Published: {}", message.data);
        
        sleep(Duration::from_millis(500)).await;
    }

    info!("✅ Basic pub/sub example completed!");
    info!("💡 Next: Try example 02 for custom messages");
    
    Ok(())
} 