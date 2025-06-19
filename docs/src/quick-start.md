# Quick Start

Get miniROS-rs running in 5 minutes with pub/sub, services, actions, and visualization.

## 📦 Installation

### Rust
Add to your `Cargo.toml`:
```toml
[dependencies]
mini-ros = "0.1"
tokio = { version = "1.0", features = ["full"] }

# With visualization
mini-ros = { version = "0.1", features = ["visualization"] }
```

### Python (ROS2 Compatible)
```bash
# Clone repository
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# Install build tools
pip install maturin

# Build Python bindings
maturin develop --features python
```

## 🚀 Basic Example

### Rust: Complete Communication

Create `src/main.rs`:

```rust
use mini_ros::prelude::*;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    
    // Create and initialize node
    let mut node = Node::new("demo_node")?;
    node.init().await?;
    
    println!("🚀 miniROS-rs Demo Started");
    
    // Create publisher and subscriber
    let publisher = node.create_publisher::<StringMsg>("/hello").await?;
    let subscriber = node.create_subscriber::<StringMsg>("/hello").await?;
    
    // Set up message callback
    subscriber.on_message(|msg| {
        println!("📨 Received: {}", msg.data);
    })?;
    
    // Create service
    let _service = node.create_service("/add", |req: (i32, i32)| {
        Ok(req.0 + req.1)
    }).await?;
    
    let service_client = node.create_service_client::<(i32, i32), i32>("/add").await?;
    
    // Demo the system
    println!("=== Testing Communication ===");
    
    // Test pub/sub
    let message = StringMsg { data: "Hello miniROS-rs!".to_string() };
    publisher.publish(&message).await?;
    
    // Test service
    let result = service_client.call((10, 32)).await?;
    println!("🔧 Service result: 10 + 32 = {}", result);
    
    // Allow time for message processing
    sleep(Duration::from_millis(100)).await;
    
    println!("✅ Demo completed successfully!");
    Ok(())
}
```

Run with:
```bash
cargo run
```

### Python: ROS2-Compatible API

Create `demo.py`:

```python
#!/usr/bin/env python3
import mini_ros
import time

def main():
    print("🚀 miniROS-rs Python Demo")
    
    # Initialize (same as rclpy.init())
    mini_ros.init()
    
    # Create node (same as rclpy.Node)
    node = mini_ros.Node('demo_node')
    
    # Publisher (same as node.create_publisher)
    pub = node.create_publisher(mini_ros.StringMessage, '/hello', 10)
    
    # Subscriber (same as node.create_subscription)
    def callback(msg):
        print(f"📨 Received: {msg.data}")
    
    sub = node.create_subscription(mini_ros.StringMessage, '/hello', callback, 10)
    
    # Publish message
    msg = mini_ros.StringMessage()
    msg.data = "Hello from Python!"
    pub.publish(msg)
    
    # Brief pause for message processing
    time.sleep(0.1)
    
    print("✅ Python demo completed!")
    
    # Cleanup (same as rclpy.shutdown())
    node.destroy_node()
    mini_ros.shutdown()

if __name__ == '__main__':
    main()
```

Run with:
```bash
python demo.py
```

## 🎯 Core Concepts

### Nodes
Computational units that manage communication:
```rust
// Basic node
let mut node = Node::new("robot_controller")?;
node.init().await?;

// Node with custom domain ID
let context = Context::with_domain_id(10)?;
let node = Node::with_context("isolated_robot", context)?;
```

### Publishers
Send messages to topics:
```rust
// String messages
let pub = node.create_publisher::<StringMsg>("/status").await?;
pub.publish(&StringMsg { data: "Ready".into() }).await?;

// Numeric data
let int_pub = node.create_publisher::<Int32Msg>("/counter").await?;
int_pub.publish(&Int32Msg { data: 42 }).await?;
```

### Subscribers
Receive messages from topics:
```rust
let sub = node.create_subscriber::<StringMsg>("/commands").await?;
sub.on_message(|msg| {
    println!("Command: {}", msg.data);
})?;
```

### Services
Request/response communication:
```rust
// Service server
let service = node.create_service("/calculate", |x: i32| {
    Ok(x * x)  // Return square of input
}).await?;

// Service client
let client = node.create_service_client::<i32, i32>("/calculate").await?;
let result = client.call(5).await?; // Returns 25
```

### Actions
Long-running tasks with feedback:
```rust
use mini_ros::action::{ActionServer, GoalStatus};

// Action server
let action_server = ActionServer::new(&node, "/navigate").await?;

action_server.on_goal(|goal| async move {
    println!("Navigating to: {:?}", goal);
    // Simulation of long-running task
    GoalStatus::Succeeded
}).await?;
```

### Parameters
Dynamic configuration:
```rust
use mini_ros::parameter::{ParameterServer, ParameterValue};

let param_server = ParameterServer::new();
param_server.set_parameter("robot_name", ParameterValue::String("Bot1".into()))?;
param_server.set_parameter("max_speed", ParameterValue::Float(2.0))?;

// Retrieve parameters
if let Some(name) = param_server.get_parameter("robot_name")? {
    println!("Robot name: {:?}", name);
}
```

## 🎨 Visualization

With Rerun integration:
```rust
// Enable visualization feature
use mini_ros::visualization::RerunLogger;

let logger = RerunLogger::new("robot_demo")?;
logger.log_point_cloud(&points).await?;
logger.log_pose(&robot_pose).await?;
```

## 🌐 Transport Options

### Memory Broker (Default)
Fastest for single-process communication:
```rust
let mut node = Node::new("local_node")?;
// Uses memory broker automatically
```

### TCP Transport
For network communication:
```rust
// Enable TCP transport feature
let mut node = Node::new("network_node")?;
node.init().await?;
```

### DDS Transport
ROS2 compatibility:
```rust
// Enable dds-transport feature
use mini_ros::dds_transport::DdsTransport;

let transport = DdsTransport::new(0).await?;
let publisher = transport.create_publisher::<StringMsg>("topic").await?;
```

## 📋 Examples Walkthrough

Run the provided examples to learn progressively:

```bash
# Basic pub/sub
cargo run --example 01_basic_pubsub

# Custom message types
cargo run --example 02_custom_messages

# Service communication
cargo run --example 03_services

# Actions and parameters
cargo run --example 04_actions_parameters

# Visualization (requires visualization feature)
cargo run --example 04_visualization_basic --features visualization

# Complete system demo
cargo run --example 07_integrated_system
```

## 🔧 Development Tips

### Error Handling
```rust
use mini_ros::error::{MiniRosError, Result};

async fn robust_communication() -> Result<()> {
    let mut node = Node::new("robust_node")?;
    node.init().await?;
    
    match node.create_publisher::<StringMsg>("/topic").await {
        Ok(pub) => println!("Publisher created"),
        Err(e) => eprintln!("Failed to create publisher: {}", e),
    }
    
    Ok(())
}
```

### Async Best Practices
```rust
// Spawn concurrent tasks
tokio::spawn(async move {
    while let Some(msg) = subscriber.recv().await {
        process_message(msg).await;
    }
});

// Use timeouts
use tokio::time::timeout;

let result = timeout(Duration::from_secs(5), service_client.call(request)).await;
```

## 🐍 Python Integration

miniROS-rs provides a Python API that closely mirrors ROS2's rclpy:

```python
# Nearly identical to ROS2 code
import mini_ros as rclpy  # Drop-in replacement
from mini_ros import StringMessage as String

# Standard ROS2 patterns work
rclpy.init()
node = rclpy.create_node('my_node')
pub = node.create_publisher(String, 'topic', 10)
node.destroy_node()
rclpy.shutdown()
```

## 📚 Next Steps

1. **[Examples](examples.md)** - Comprehensive learning examples
2. **[Python Bindings](python-bindings.md)** - Complete Python API reference
3. **[API Documentation](api.md)** - Full Rust API reference
4. **[Visualization](visualization.md)** - 3D visualization with Rerun
5. **[Performance](performance.md)** - Optimization techniques

Start with the examples and gradually explore more advanced features! 