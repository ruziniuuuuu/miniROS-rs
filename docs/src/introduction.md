# miniROS-rs

**A lightweight, high-performance robotics middleware in Rust**

## What is miniROS-rs?

miniROS-rs is a simple, fast robotics communication library inspired by ROS2. It provides:

- **Publisher/Subscriber** communication
- **Service** request/response patterns  
- **Real-time visualization** with Rerun
- **Cross-platform** support (Linux, macOS, Windows)
- **Memory safety** guaranteed by Rust

## Why miniROS-rs?

- 🚀 **Fast**: High-performance async communication
- 🛡️ **Safe**: Rust's memory safety without runtime overhead
- 📊 **Visual**: Built-in 3D visualization support
- 📦 **Simple**: Easy to learn and integrate

## Quick Example

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create node
    let mut node = Node::new("my_robot")?;
    node.init().await?;
    
    // Publish messages
    let publisher = node.create_publisher::<StringMsg>("robot_status").await?;
    publisher.publish(&StringMsg { data: "Hello, Robot!".to_string() }).await?;
    
    Ok(())
}
```

Get started with the [Quick Start](./quick-start.md) guide! 