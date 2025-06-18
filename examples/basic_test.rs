//! Basic MiniROS test example
//! 
//! This example demonstrates basic functionality without complex networking:
//! - Creates nodes
//! - Shows message serialization
//! - Tests basic functionality

use mini_ros::prelude::*;
use mini_ros::message::{StringMsg, Int32Msg, Float64Msg, BoolMsg};
use tracing::info;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing for logging
    tracing_subscriber::fmt::init();
    
    info!("Starting MiniROS basic test example");

    // Test 1: Node creation
    info!("=== Test 1: Node Creation ===");
    let node1 = Node::new("test_node_1")?;
    let node2 = Node::new("test_node_2")?;
    
    info!("Created node: {} (ID: {})", node1.name(), node1.id());
    info!("Created node: {} (ID: {})", node2.name(), node2.id());

    // Test 2: Different domain IDs
    info!("=== Test 2: Context with Domain ID ===");
    let context = Context::with_domain_id(42)?;
    let node3 = Node::with_context("domain_node", context)?;
    info!("Created node with domain ID: {} (Domain: {})", 
          node3.name(), node3.context().domain_id());

    // Test 3: Message creation and serialization
    info!("=== Test 3: Message Types ===");
    
    let string_msg = StringMsg { data: "Hello, MiniROS!".to_string() };
    info!("String message: {:?}", string_msg);
    
    let int_msg = Int32Msg { data: 42 };
    info!("Int32 message: {:?}", int_msg);
    
    let float_msg = Float64Msg { data: 3.14159 };
    info!("Float64 message: {:?}", float_msg);
    
    let bool_msg = BoolMsg { data: true };
    info!("Bool message: {:?}", bool_msg);

    // Test 4: Message serialization
    info!("=== Test 4: Message Serialization ===");
    
    let serialized = bincode::serialize(&string_msg).unwrap();
    info!("Serialized string message: {} bytes", serialized.len());
    
    let deserialized: StringMsg = bincode::deserialize(&serialized).unwrap();
    info!("Deserialized message: {:?}", deserialized);
    
    assert_eq!(string_msg.data, deserialized.data);
    info!("✅ Serialization test passed!");

    // Test 5: Custom message
    info!("=== Test 5: Custom Message ===");
    
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    struct RobotPose {
        x: f64,
        y: f64,
        theta: f64,
    }
    
    let pose = RobotPose { x: 1.0, y: 2.0, theta: 0.5 };
    info!("Robot pose: {:?}", pose);
    
    let pose_serialized = bincode::serialize(&pose).unwrap();
    let pose_deserialized: RobotPose = bincode::deserialize(&pose_serialized).unwrap();
    info!("Custom message serialization test passed: {:?}", pose_deserialized);

    info!("=== All tests completed successfully! ===");
    Ok(())
} 