//! Example 03: Service Communication
//!
//! This example demonstrates:
//! - Creating services (request/response pattern)
//! - Service clients
//! - Synchronous communication
//!
//! Run with: cargo run --example 03_services

use mini_ros::{
    message::{Int32Msg, StringMsg},
    node::Node,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 03: Services ===");

    // Create server node
    let mut server_node = Node::new("service_server")?;
    server_node.init().await?;

    // Create service: calculate string length
    let _string_service = server_node
        .create_service(
            "/calculate/string_length",
            |req: StringMsg| -> mini_ros::error::Result<Int32Msg> {
                let length = req.data.len() as i32;
                info!(
                    "🔧 Service processing: '{}' -> length = {}",
                    req.data, length
                );
                Ok(Int32Msg { data: length })
            },
        )
        .await?;

    // Create another service: calculate square
    let _square_service = server_node
        .create_service(
            "/calculate/square",
            |req: Int32Msg| -> mini_ros::error::Result<Int32Msg> {
                let result = req.data * req.data;
                info!("🔧 Service processing: {} squared = {}", req.data, result);
                Ok(Int32Msg { data: result })
            },
        )
        .await?;

    info!("🔌 Services started, creating client...");

    // Create client node
    let mut client_node = Node::new("service_client")?;
    client_node.init().await?;

    // Create service clients
    let string_client = client_node
        .create_service_client::<StringMsg, Int32Msg>("/calculate/string_length")
        .await?;

    let square_client = client_node
        .create_service_client::<Int32Msg, Int32Msg>("/calculate/square")
        .await?;

    // Wait for services to be available
    info!("⏳ Waiting for services to be available...");
    sleep(Duration::from_millis(500)).await;

    // Test string length service
    info!("🚀 Testing string length service...");
    let test_strings = vec![
        "Hello",
        "miniROS-rs",
        "Service communication example",
        "🚀🤖",
    ];

    for text in test_strings {
        let request = StringMsg {
            data: text.to_string(),
        };
        match string_client.call(request).await {
            Ok(response) => {
                info!("📞 String '{}' has length: {}", text, response.data);
            }
            Err(e) => {
                info!("❌ Service call failed: {}", e);
            }
        }
        sleep(Duration::from_millis(300)).await;
    }

    // Test square service
    info!("🚀 Testing square service...");
    let test_numbers = vec![3, 7, 12, 25];

    for number in test_numbers {
        let request = Int32Msg { data: number };
        match square_client.call(request).await {
            Ok(response) => {
                info!("📞 {} squared = {}", number, response.data);
            }
            Err(e) => {
                info!("❌ Service call failed: {}", e);
            }
        }
        sleep(Duration::from_millis(300)).await;
    }

    info!("✅ Services example completed!");
    info!("💡 Next: Try example 04 for visualization");

    Ok(())
}
