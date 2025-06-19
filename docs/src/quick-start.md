# Quick Start

Get miniROS-rs running in 5 minutes with complete pub/sub, services, actions, and visualization capabilities.

## 📦 Installation

### Rust
Add to your `Cargo.toml`:
```toml
[dependencies]
mini-ros = "0.1.2"
tokio = { version = "1.0", features = ["full"] }
```

For visualization:
```toml
[dependencies]
mini-ros = { version = "0.1.2", features = ["visualization"] }
tokio = { version = "1.0", features = ["full"] }
```

### Python (ROS2 rclpy Compatible)
```bash
# Clone the repository
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# Install build tools
pip install maturin

# Build and install Python bindings
maturin develop --features python
```

## 🚀 Your First miniROS Program

### Rust: Complete Communication Demo

Create `src/main.rs`:

```rust
use mini_ros::prelude::*;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    
    // Create node
    let mut node = Node::new("quickstart_demo")?;
    node.init().await?;
    
    println!("🚀 miniROS-rs Quickstart Demo");
    
    // === PUB/SUB ===
    let publisher = node.create_publisher::<StringMsg>("/hello").await?;
    let subscriber = node.create_subscriber::<StringMsg>("/hello").await?;
    
    subscriber.on_message(|msg| {
        println!("📨 Received: {}", msg.data);
    })?;
    
    // === SERVICE ===
    let _service = node.create_service("/add", |req: (i32, i32)| {
        let result = req.0 + req.1;
        println!("🔧 Service called: {} + {} = {}", req.0, req.1, result);
        Ok(result)
    }).await?;
    
    let service_client = node.create_service_client::<(i32, i32), i32>("/add").await?;
    
    // === PARAMETERS ===
    let param_server = ParameterServer::new();
    param_server.set_parameter("robot_name", ParameterValue::String("MiniBot".into()))?;
    param_server.set_parameter("max_speed", ParameterValue::Float(2.5))?;
    
    // Demo the system
    println!("\n=== Running Demo ===");
    
    // 1. Pub/Sub
    let message = StringMsg { data: "Hello miniROS-rs!".to_string() };
    publisher.publish(&message).await?;
    
    // 2. Service call
    let result = service_client.call((10, 32)).await?;
    println!("🔧 Service result: {}", result);
    
    // 3. Parameter access
    if let Some(name) = param_server.get_parameter("robot_name")? {
        println!("🔧 Robot name: {:?}", name);
    }
    
    // Let messages process
    sleep(Duration::from_millis(100)).await;
    
    println!("✅ miniROS-rs works perfectly!");
    Ok(())
}
```

Run it:
```bash
cargo run
```

### Python: ROS2-Compatible Demo

Create `quickstart.py`:

```python
#!/usr/bin/env python3
import mini_ros
import time

def main():
    print("🚀 miniROS-rs Python Quickstart")
    
    # Initialize (like rclpy.init())
    mini_ros.init()
    
    # Create node (like rclpy.Node)
    node = mini_ros.Node('quickstart_demo')
    
    # === PUB/SUB ===
    # Publisher (like rclpy create_publisher)
    pub = node.create_publisher(mini_ros.String, '/hello', 10)
    
    # Subscriber (like rclpy create_subscription)
    def callback(msg):
        print(f"📨 Received: {msg.data}")
    
    sub = node.create_subscription(mini_ros.String, '/hello', callback, 10)
    
    # === PUBLISHING ===
    msg = mini_ros.String()
    msg.data = "Hello from Python!"
    pub.publish(msg)
    
    # === LOGGING ===
    logger = node.get_logger()
    logger.info("Node initialized successfully")
    logger.info(f"Publishing to /hello topic")
    
    print("✅ miniROS-rs Python works!")
    
    # Cleanup (like rclpy.shutdown())
    node.destroy_node()
    mini_ros.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python quickstart.py
```

## 🎯 Core Concepts

### 1. Nodes - Computational Units
Nodes are the basic building blocks:
```rust
let mut node = Node::new("my_robot")?;
node.init().await?;

// With custom domain
let context = Context::with_domain_id(42)?;
let node = Node::with_context("isolated_robot", context)?;
```

### 2. Publishers - Send Data
Broadcast messages to topics:
```rust
let pub = node.create_publisher::<StringMsg>("/sensor_data").await?;
pub.publish(&StringMsg { data: "sensor reading".into() }).await?;

// Different message types
let int_pub = node.create_publisher::<Int32Msg>("/count").await?;
int_pub.publish(&Int32Msg { data: 42 }).await?;
```

### 3. Subscribers - Receive Data
Listen for messages on topics:
```rust
let sub = node.create_subscriber::<StringMsg>("/commands").await?;
sub.on_message(|msg| {
    println!("Command received: {}", msg.data);
})?;
```

### 4. Services - Request/Response
Synchronous communication:
```rust
// Server
let service = node.create_service("/calculate", |x: i32| {
    Ok(x * x)  // Square the input
}).await?;

// Client
let client = node.create_service_client::<i32, i32>("/calculate").await?;
let result = client.call(7).await?; // Returns 49
```

### 5. Actions - Long-Running Tasks
For tasks that take time and provide feedback:
```rust
use mini_ros::{ActionServer, ActionClient};

// Server
let action_server = ActionServer::new(&mut node, "fibonacci").await?;
action_server.on_goal(|goal| {
    // Process goal, send feedback, return result
}).await?;

// Client
let action_client = ActionClient::new(&mut node, "fibonacci").await?;
let goal_id = action_client.send_goal(&goal_data).await?;
```

### 6. Parameters - Configuration
Dynamic configuration management:
```rust
// Server
let param_server = ParameterServer::new();
param_server.set_parameter("debug_mode", ParameterValue::Bool(true))?;

// Client
let param_client = ParameterClient::from_server(&param_server);
let debug = param_client.get_parameter("debug_mode")?;
```

## 📊 Message Types

### Built-in Messages
```rust
// Basic types
StringMsg { data: "Hello".into() }
Int32Msg { data: 42 }
Float64Msg { data: 3.14159 }
BoolMsg { data: true }
EmptyMsg  // For triggers/events

// Stamped messages (with header)
use std::time::{SystemTime, UNIX_EPOCH};
let stamped = Stamped::new(
    StringMsg { data: "timestamped".into() },
    1,  // sequence number
    "base_link".into()  // frame_id
);
```

### Custom Messages
Any Serde-compatible struct:
```rust
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    position: [f64; 3],
    orientation: [f64; 4],
    timestamp: u64,
}

// Use like any built-in message
let pose_pub = node.create_publisher::<RobotPose>("/robot_pose").await?;
let pose = RobotPose {
    position: [1.0, 2.0, 0.0],
    orientation: [0.0, 0.0, 0.0, 1.0],
    timestamp: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs(),
};
pose_pub.publish(&pose).await?;
```

## 🌐 Transport Options

### TCP Transport (Default)
Simple, reliable, cross-platform:
```bash
cargo run  # Uses TCP by default
```

### DDS Transport (ROS2 Compatible)
Full ROS2 interoperability with domain isolation:
```bash
# Standard domain 0
cargo run --features dds-transport

# Custom domain for isolation
DOMAIN_ID=42 cargo run --features dds-transport
```

### Zenoh Transport (Cloud-Native)
Modern, cloud-ready transport:
```bash
cargo run --example 05_zenoh_transport
```

## 🎨 3D Visualization

Built-in Rerun integration for 3D visualization:
```rust
#[cfg(feature = "visualization")]
{
    let viz = node.create_visualizer("robot_viz").await?;
    
    // Log 3D points
    viz.log_point("robot/position", [1.0, 2.0, 0.5]).await?;
    
    // Log paths
    viz.log_path("robot/trajectory", &trajectory_points).await?;
    
    // Log text labels
    viz.log_text("robot/status", "All systems operational").await?;
}
```

Run with visualization:
```bash
cargo run --example 04_visualization_basic --features visualization
```

## 📚 Progressive Examples

Learn step-by-step with our numbered examples:

```bash
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# 1. Basic pub/sub communication
cargo run --example 01_basic_pubsub

# 2. Custom message types
cargo run --example 02_custom_messages

# 3. Services for request/response
cargo run --example 03_services

# 4. Basic 3D visualization
cargo run --example 04_visualization_basic --features visualization

# 5. Modern Zenoh transport
cargo run --example 05_zenoh_transport

# 6. Advanced visualization
cargo run --example 06_visualization_advanced --features visualization

# 7. Complete integrated system
cargo run --example 07_integrated_system --features visualization
```

### Python Examples (ROS2 Compatible)
```bash
# Minimal examples
python python/examples/minimal_publisher.py
python python/examples/minimal_subscriber.py

# Advanced examples
python python/examples/talker.py      # Advanced publisher
python python/examples/listener.py   # OOP subscriber
python python/examples/simple_pubsub.py  # Combined demo
python python/examples/simple_param.py   # Parameter usage
```

## 🔧 Common Patterns

### Multi-node Communication
```rust
// Terminal 1: Publisher node
let mut pub_node = Node::new("publisher")?;
pub_node.init().await?;
let pub = pub_node.create_publisher::<StringMsg>("/data").await?;

// Terminal 2: Subscriber node  
let mut sub_node = Node::new("subscriber")?;
sub_node.init().await?;
let sub = sub_node.create_subscriber::<StringMsg>("/data").await?;
```

### Error Handling
```rust
use mini_ros::{Result, MiniRosError};

async fn robust_communication() -> Result<()> {
    let mut node = Node::new("robust_node")?;
    node.init().await?;
    
    let publisher = node.create_publisher::<StringMsg>("/topic").await
        .map_err(|e| {
            eprintln!("Failed to create publisher: {}", e);
            e
        })?;
    
    Ok(())
}
```

### Async Patterns
```rust
// Concurrent publishers
let (pub1, pub2) = tokio::join!(
    node.create_publisher::<StringMsg>("/topic1"),
    node.create_publisher::<StringMsg>("/topic2")
);

// Periodic publishing
let mut interval = tokio::time::interval(Duration::from_millis(100));
loop {
    interval.tick().await;
    publisher.publish(&message).await?;
}
```

## 🚀 What's Next?

Choose your learning path:

### 🎓 **Learn the Fundamentals**
- **[Core Concepts](./concepts.md)** - Deep dive into architecture
- **[Examples](./examples.md)** - Progressive tutorial sequence

### 🐍 **Python Development**  
- **[Python Bindings](./python-bindings.md)** - Complete Python API guide
- Start with `python/examples/minimal_*.py`

### 🌐 **Advanced Topics**
- **[Transport Architecture](./transport-architecture.md)** - Multi-transport design
- **[Visualization](./visualization.md)** - 3D visualization guide
- **[Performance](./performance.md)** - Optimization techniques

### 🔧 **Integration**
- **[DDS Transport](./dds-transport.md)** - ROS2 compatibility
- Check out the `07_integrated_system` example

## 💡 Tips

1. **Start Simple**: Begin with basic pub/sub before moving to services/actions
2. **Use Examples**: Each example builds on the previous one
3. **Check Logs**: Enable tracing with `tracing_subscriber::fmt::init()`
4. **Domain Isolation**: Use different domain IDs for separate robot systems
5. **Python Compatibility**: Most rclpy patterns work directly

Happy coding with miniROS-rs! 🦾 