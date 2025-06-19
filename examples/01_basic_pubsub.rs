//! Example 01: Basic Publisher-Subscriber Communication with Domain Isolation
//!
//! This is your first miniROS-rs example. It demonstrates:
//! - Creating nodes with domain isolation
//! - Basic pub/sub communication
//! - Using built-in message types
//! - DDS transport support with domain separation
//!
//! Run with: cargo run --example 01_basic_pubsub
//! DDS transport: cargo run --example 01_basic_pubsub --features dds-transport
//! Custom domain: DOMAIN_ID=10 cargo run --example 01_basic_pubsub --features dds-transport

use mini_ros::{message::StringMsg, node::Node};

#[cfg(feature = "dds-transport")]
use mini_ros::dds_transport::{DdsTransport, QosPolicy, ReliabilityKind};

use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 01: Basic Pub/Sub ===");

    #[cfg(feature = "dds-transport")]
    {
        info!("🌐 Using DDS transport");
        run_dds_example().await
    }

    #[cfg(not(feature = "dds-transport"))]
    {
        info!("🔗 Using TCP transport");
        run_node_example().await
    }
}

#[allow(dead_code)]
async fn run_node_example() -> mini_ros::error::Result<()> {
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

#[cfg(feature = "dds-transport")]
async fn run_dds_example() -> mini_ros::error::Result<()> {
    // Get domain ID from environment or use default
    let domain_id = std::env::var("DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    // Create DDS transport with specified domain ID
    let transport = DdsTransport::new(domain_id).await?;
    info!(
        "🌐 DDS transport initialized for domain {} (isolated network)",
        domain_id
    );

    // Create publisher with default QoS
    let publisher = transport.create_publisher::<StringMsg>("greetings").await?;
    info!("📤 Publisher created for topic: greetings");

    // Create subscriber with reliable QoS
    let reliable_qos = QosPolicy {
        reliability: ReliabilityKind::Reliable,
        depth: 10,
        ..Default::default()
    };
    let subscriber = transport
        .create_subscriber_with_qos::<StringMsg>("greetings", reliable_qos)
        .await?;
    info!("📥 Subscriber created for topic: greetings");

    // Set up message callback
    subscriber.on_message(|msg: StringMsg| {
        info!("📨 Received: {}", msg.data);
    })?;

    info!("🚀 Starting DDS pub/sub communication...");

    // Allow some time for discovery
    sleep(Duration::from_millis(100)).await;

    // Publish messages in a loop
    for i in 0..5 {
        let message = StringMsg {
            data: format!("Hello DDS World! Message #{}", i + 1),
        };

        publisher.publish(&message).await?;
        info!("📤 Published: {}", message.data);

        sleep(Duration::from_millis(500)).await;
    }

    // Allow time for final messages to be processed
    sleep(Duration::from_secs(1)).await;

    info!("✅ DDS pub/sub example completed!");
    info!("💡 Next: Try example 02 for custom messages with DDS");

    Ok(())
}
