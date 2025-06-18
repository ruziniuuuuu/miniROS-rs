# Introduction

**miniROS** is a minimal, high-performance robotics middleware written in Rust. It provides the essential communication primitives needed for robotics applications without the complexity and overhead of traditional ROS systems.

<!-- Documentation deployed via GitHub Pages -->

## Design Philosophy: Less is More

miniROS embraces minimalism:

- **Core Only**: Pub/sub, services, and discovery - the essentials
- **Zero Bloat**: No unnecessary features or dependencies
- **Performance First**: Rust's zero-cost abstractions and async I/O
- **Simple APIs**: Easy to learn, easy to use
- **ROS2 Compatible**: Works with existing ROS2 tools when needed

## What miniROS Provides

### Essential Communication
- **Publishers/Subscribers**: Asynchronous message passing
- **Services**: Request/response patterns
- **Node Discovery**: Automatic network discovery
- **Message Types**: Built-in and custom message support

### Transport Options
- **TCP**: Simple, reliable (default)
- **DDS**: ROS2-compatible transport layer
- **Cross-platform**: Linux, macOS, Windows

### Language Support
- **Rust**: Native, zero-cost API
- **Python**: ROS2-compatible bindings

## What miniROS Doesn't Provide

We intentionally exclude complex features that add bloat:
- Parameter servers (use config files)
- Action servers (use services)
- Complex QoS policies (sensible defaults)
- Lifecycle management (keep it simple)
- Transformation trees (use external libraries)

## When to Use miniROS

**Perfect for:**
- Performance-critical robotics applications
- Embedded systems with limited resources
- Simple communication patterns
- Learning robotics middleware concepts
- Prototyping and research

**Consider ROS2 for:**
- Large, complex robotics ecosystems
- Heavy integration with existing ROS packages
- Advanced navigation and perception stacks

---

*miniROS: Maximum capability, minimum complexity*

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