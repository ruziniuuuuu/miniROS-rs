//! Zenoh communication example for miniROS
//! 
//! Demonstrates high-performance pub/sub communication using Eclipse Zenoh protocol.

use mini_ros::zenoh_transport::{ZenohConfig, ZenohTransport};
use mini_ros::message::StringMsg;
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();
    
    info!("Starting Zenoh communication example");

    // Create Zenoh transport
    let config = ZenohConfig::default();
    let transport = ZenohTransport::new(config).await?;
    
    // Create publisher and subscriber
    let topic = "/chatter";
    let publisher = transport.create_publisher::<StringMsg>(topic).await?;
    let subscriber = transport.create_subscriber::<StringMsg>(topic).await?;

    // Set up subscriber callback
    subscriber.on_message(|msg: StringMsg| {
        info!("Received via Zenoh: {}", msg.data);
    })?;

    // Give time for setup
    sleep(Duration::from_millis(100)).await;

    // Publish messages
    info!("Publishing messages via Zenoh...");
    for i in 0..5 {
        let message = StringMsg {
            data: format!("Hello from Zenoh! Message #{}", i),
        };
        
        publisher.publish(&message).await?;
        info!("Published: {}", message.data);
        
        sleep(Duration::from_millis(500)).await;
    }

    // Wait for final messages
    sleep(Duration::from_secs(1)).await;

    // Shutdown
    transport.shutdown().await?;
    info!("Zenoh communication example completed");
    
    Ok(())
} 