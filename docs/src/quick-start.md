# Quick Start

Get miniROS running in 5 minutes. This guide covers the absolute essentials.

## Installation

### Rust
Add to your `Cargo.toml`:
```toml
[dependencies]
mini-ros = "0.1.2"
tokio = { version = "1.0", features = ["full"] }
```

### Python (Optional)
```bash
pip install maturin
maturin develop --features python
```

## Your First miniROS Program

### Rust: Publisher & Subscriber

Create `src/main.rs`:

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create node
    let mut node = Node::new("quickstart")?;
    node.init().await?;
    
    // Publisher
    let publisher = node.create_publisher::<StringMsg>("hello").await?;
    
    // Subscriber
    let subscriber = node.create_subscriber::<StringMsg>("hello").await?;
    subscriber.on_message(|msg| {
        println!("📨 Received: {}", msg.data);
    })?;
    
    // Publish a message
    let message = StringMsg { data: "Hello miniROS!".to_string() };
    publisher.publish(&message).await?;
    
    // Let it run briefly to see the message
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    
    println!("✅ miniROS works!");
    Ok(())
}
```

Run it:
```bash
cargo run
```

### Python: Basic Communication

Create `hello.py`:

```python
#!/usr/bin/env python3
import mini_ros
import time

# Initialize
mini_ros.init()
node = mini_ros.Node('quickstart')

# Publisher  
pub = node.create_publisher(mini_ros.String, 'hello', 10)

# Subscriber
def callback(msg):
    print(f"📨 Received: {msg.data}")

sub = node.create_subscription(mini_ros.String, 'hello', callback, 10)

# Publish a message
msg = mini_ros.String()
msg.data = "Hello miniROS from Python!"
pub.publish(msg)

print("✅ miniROS Python works!")
```

Run it:
```bash
python hello.py
```

## Core Concepts

### 1. Nodes
Computational units that communicate:
```rust
let mut node = Node::new("my_robot")?;
node.init().await?;
```

### 2. Publishers
Send messages to topics:
```rust
let pub = node.create_publisher::<StringMsg>("sensor_data").await?;
pub.publish(&StringMsg { data: "sensor reading".into() }).await?;
```

### 3. Subscribers  
Receive messages from topics:
```rust
let sub = node.create_subscriber::<StringMsg>("commands").await?;
sub.on_message(|msg| println!("Command: {}", msg.data))?;
```

### 4. Services
Request/response communication:
```rust
// Server
let service = node.create_service("calculate", |x: i32| Ok(x * 2)).await?;

// Client
let client = node.create_service_client::<i32, i32>("calculate").await?;
let result = client.call(21).await?; // Returns 42
```

## Message Types

### Built-in Messages
```rust
StringMsg { data: "text".into() }
Int32Msg { data: 42 }
Float64Msg { data: 3.14 }
BoolMsg { data: true }
```

### Custom Messages
Any Serde-compatible struct:
```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    x: f64,
    y: f64,
    heading: f64,
}

let pose = RobotPose { x: 1.0, y: 2.0, heading: 0.5 };
publisher.publish(&pose).await?;
```

## Transport Options

### TCP (Default)
Simple and reliable:
```bash
cargo run --example 01_basic_pubsub
```

### DDS (ROS2 Compatible)
Full ROS2 interoperability:
```bash
cargo run --example 01_basic_pubsub --features dds-transport
```

## Examples

Clone and explore:
```bash
git clone <repo-url>
cd miniROS-rs

# Basic communication
cargo run --example 01_basic_pubsub

# Custom message types  
cargo run --example 02_custom_messages

# Request/response services
cargo run --example 03_services

# Python examples
python python/examples/talker.py
python python/examples/image_publisher.py
```

## What's Next?

- **[Core Concepts](concepts.md)** - Deeper understanding
- **[Python Bindings](python-bindings.md)** - Python development
- **[DDS Transport](dds-transport.md)** - ROS2 compatibility
- **[Examples](examples.md)** - More advanced patterns

## Common Patterns

### Multi-node Communication
```rust
// Node 1: Sensor
let sensor_node = Node::new("sensor")?;
let sensor_pub = sensor_node.create_publisher::<Float64Msg>("temperature").await?;

// Node 2: Controller  
let control_node = Node::new("controller")?;
let temp_sub = control_node.create_subscriber::<Float64Msg>("temperature").await?;
let cmd_pub = control_node.create_publisher::<StringMsg>("commands").await?;
```

### Error Handling
```rust
match publisher.publish(&message).await {
    Ok(_) => println!("Message sent"),
    Err(e) => eprintln!("Failed to send: {}", e),
}
```

### Graceful Shutdown
```rust
// Handle Ctrl+C
tokio::signal::ctrl_c().await?;
node.shutdown().await?;
```

---

*You're now ready to build with miniROS! 🚀* 