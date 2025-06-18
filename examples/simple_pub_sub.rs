//! Simple publisher-subscriber example for MiniROS
//! 
//! This example demonstrates basic pub/sub functionality:
//! - Creates a publisher node that sends string messages
//! - Creates a subscriber node that receives and prints messages
//! - Shows how to use async callbacks for message processing

use mini_ros::prelude::*;
use mini_ros::message::StringMsg;
use std::time::Duration;
use tokio::time::sleep;
use tracing::{info, warn};

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing for logging
    tracing_subscriber::fmt::init();
    
    info!("Starting MiniROS simple pub/sub example");

    // Create publisher node
    let mut pub_node = Node::new("publisher_node")?;
    pub_node.init().await?;
    
    // Create subscriber node  
    let mut sub_node = Node::new("subscriber_node")?;
    sub_node.init().await?;

    // Create publisher and subscriber
    let publisher = pub_node.create_publisher::<StringMsg>("chatter").await?;
    let subscriber = sub_node.create_subscriber::<StringMsg>("chatter").await?;

    // Set up message callback
    subscriber.on_message(|msg: StringMsg| {
        info!("Received message: {}", msg.data);
    })?;

    // Give some time for discovery and connection setup
    sleep(Duration::from_secs(1)).await;

    // Publish some messages
    info!("Publishing messages...");
    for i in 0..10 {
        let message = StringMsg {
            data: format!("Hello, MiniROS! Message #{}", i),
        };
        
        match publisher.publish(&message).await {
            Ok(_) => info!("Published: {}", message.data),
            Err(e) => warn!("Failed to publish message: {}", e),
        }
        
        sleep(Duration::from_millis(500)).await;
    }

    // Wait a bit for final messages to be processed
    sleep(Duration::from_secs(2)).await;

    // Shutdown nodes
    info!("Shutting down...");
    pub_node.shutdown().await?;
    sub_node.shutdown().await?;

    info!("Example completed successfully");
    Ok(())
} 