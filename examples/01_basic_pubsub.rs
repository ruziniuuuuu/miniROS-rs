//! Example 01: Basic Publisher-Subscriber Communication
//!
//! This example demonstrates miniROS core functionality:
//! - ROS2-compatible message types
//! - Message serialization and validation
//! - Simple, focused robotics messaging patterns
//! - Direct message handling without background services
//!
//! Following miniROS philosophy: Maximum performance with minimum complexity
//!
//! Run with: cargo run --example 01_basic_pubsub

use mini_ros::types::{std_msgs, MiniRosMessage};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging for better debugging
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 01: Basic Pub/Sub ===");
    info!("🎯 Demonstrating core ROS2-compatible message functionality");

    // Demonstrate message creation and validation
    info!("📦 Creating ROS2-compatible messages...");

    let messages = vec![
        std_msgs::String {
            data: "Hello miniROS! Message #1".to_string(),
        },
        std_msgs::String {
            data: "Hello miniROS! Message #2".to_string(),
        },
        std_msgs::String {
            data: "Hello miniROS! Message #3".to_string(),
        },
    ];

    // Demonstrate message type info
    info!("📋 Message type: {}", std_msgs::String::message_type());
    info!("📋 Message schema: {:?}", std_msgs::String::schema());

    // Simulate publishing and subscribing
    info!("🚀 Simulating pub/sub communication...");

    for (i, message) in messages.iter().enumerate() {
        // Validate message before "publishing"
        match message.validate() {
            Ok(_) => {
                info!("📤 Publishing: '{}'", message.data);
                
                // Simulate message serialization (what would happen in real pub/sub)
                let serialized = message.to_bytes()?;
                info!("📦 Serialized to {} bytes", serialized.len());

                // Simulate message deserialization (what subscriber would do)
                let deserialized = std_msgs::String::from_bytes(&serialized)?;
                info!("📨 Received: '{}'", deserialized.data);

                // Verify round-trip integrity
                assert_eq!(message.data, deserialized.data);
                info!("✅ Message #{} integrity verified", i + 1);
            }
            Err(e) => {
                info!("❌ Message validation failed: {}", e);
            }
        }

        // Short delay to demonstrate message flow
        sleep(Duration::from_millis(200)).await;
    }

    info!("✅ Basic pub/sub simulation completed successfully!");
    info!("💡 Key miniROS principles demonstrated:");
    info!("   • Minimal complexity - direct message handling");
    info!("   • Maximum performance - zero-copy serialization");
    info!("   • ROS2 compatibility - standard message types");
    info!("   • Type safety - compile-time message validation");
    info!("   • Cross-platform - works everywhere Rust works");
    
    info!("📚 Message features shown:");
    info!("   • Message type identification");
    info!("   • Schema introspection");
    info!("   • Validation and error handling");
    info!("   • Binary serialization/deserialization");
    info!("   • Round-trip integrity verification");
    
    info!("💡 Next: Try example 02 for custom messages");
    info!("💡 For full pub/sub networking, see example 16 (message packages)");

    Ok(())
}
