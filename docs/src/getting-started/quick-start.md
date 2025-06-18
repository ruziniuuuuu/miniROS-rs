# Quick Start

This guide will get you up and running with miniROS-rs in just a few minutes.

## Create a New Project

Start by creating a new Rust project:

```bash
cargo new my-robot-app
cd my-robot-app
```

Add miniROS-rs to your `Cargo.toml`:

```toml
[dependencies]
mini-ros = { path = "../miniROS-rs" }  # Adjust path as needed
tokio = { version = "1.0", features = ["full"] }
tracing = "0.1"
tracing-subscriber = "0.3"
```

## Your First miniROS-rs Application

Create a simple publisher-subscriber application:

```rust
// src/main.rs
use mini_ros::{
    node::Node,
    message::StringMsg,
    context::Context,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    
    info!("Starting miniROS-rs application");

    // Create context and node
    let context = Context::new();
    let mut node = Node::new("my_robot_node", context).await?;

    // Create publisher and subscriber
    let publisher = node.create_publisher::<StringMsg>("/hello_topic").await?;
    let mut subscriber = node.create_subscriber::<StringMsg>("/hello_topic").await?;

    // Subscribe to messages
    subscriber.set_callback(|msg: StringMsg| {
        info!("Received: {}", msg.data);
    })?;

    // Publish messages in a loop
    for i in 0..10 {
        let message = StringMsg {
            data: format!("Hello, miniROS-rs! Message #{}", i),
        };
        
        publisher.publish(&message).await?;
        info!("Published: {}", message.data);
        
        sleep(Duration::from_secs(1)).await;
    }

    info!("Application completed");
    Ok(())
}
```

## Run Your Application

Build and run your application:

```bash
cargo run
```

You should see output similar to:

```
INFO my_robot_app: Starting miniROS-rs application
INFO my_robot_app: Published: Hello, miniROS-rs! Message #0
INFO my_robot_app: Received: Hello, miniROS-rs! Message #0
INFO my_robot_app: Published: Hello, miniROS-rs! Message #1
INFO my_robot_app: Received: Hello, miniROS-rs! Message #1
...
INFO my_robot_app: Application completed
```

## Adding Visualization

Let's add some visualization to see your data in action:

```toml
# Add to Cargo.toml
[dependencies]
mini-ros = { path = "../miniROS-rs" }
tokio = { version = "1.0", features = ["full"] }
tracing = "0.1"
tracing-subscriber = "0.3"
```

Update your application:

```rust
// src/main.rs
use mini_ros::{
    node::Node,
    message::StringMsg,
    context::Context,
    visualization::{VisualizationClient, VisualizationConfig},
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();
    info!("Starting miniROS-rs with visualization");

    // Create visualization client
    let viz_config = VisualizationConfig::default();
    let viz_client = VisualizationClient::new(viz_config)?;

    // Create context and node
    let context = Context::new();
    let mut node = Node::new("my_robot_node", context).await?;

    // Create publisher and subscriber
    let publisher = node.create_publisher::<StringMsg>("/robot_status").await?;

    // Simulate robot operation
    for i in 0..20 {
        let t = i as f32 * 0.5;
        
        // Publish status message
        let status = StringMsg {
            data: format!("Robot operational - Step {}", i),
        };
        publisher.publish(&status).await?;
        
        // Log visualization data
        viz_client.log_scalar("robot/battery_level", 100.0 - (i as f64 * 2.5))?;
        viz_client.log_scalar("robot/speed", (t * 0.5).sin() as f64 + 1.0)?;
        viz_client.log_text("robot/status", &status.data)?;
        
        // Simulate robot position
        let x = (t * 0.3).cos() * 5.0;
        let y = (t * 0.3).sin() * 5.0;
        viz_client.log_transform_3d(
            "robot/pose",
            [x, y, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        )?;
        
        info!("Step {}: Battery {}%, Position [{:.1}, {:.1}]", 
              i, 100.0 - (i as f32 * 2.5), x, y);
        
        sleep(Duration::from_millis(500)).await;
    }

    info!("Robot simulation completed");
    Ok(())
}
```

## Next Steps

Congratulations! You've created your first miniROS-rs application. Here's what to explore next:

### Core Concepts
- **[Nodes](../core-concepts/nodes.md)**: Learn about the fundamental building blocks
- **[Messages](../core-concepts/messages.md)**: Understand message types and creation
- **[Publishers & Subscribers](../core-concepts/pubsub.md)**: Deep dive into communication patterns

### Advanced Features
- **[Zenoh Integration](../communication/zenoh.md)**: High-performance communication
- **[Rerun Visualization](../visualization/rerun.md)**: Rich data visualization
- **[Services](../core-concepts/services.md)**: Request-response communication

### Examples
- **[Examples](./examples.md)**: Explore more complex examples
- **[Visualization Examples](../visualization/examples.md)**: Advanced visualization patterns

## Common Patterns

### Multiple Nodes

Create separate applications for different robot components:

```rust
// Publisher node (sensors.rs)
use mini_ros::{node::Node, message::Float64Msg, context::Context};

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    let context = Context::new();
    let mut node = Node::new("sensor_node", context).await?;
    let temp_pub = node.create_publisher::<Float64Msg>("/temperature").await?;
    
    loop {
        let temp = 20.0 + fastrand::f64() * 10.0; // Simulate sensor
        temp_pub.publish(&Float64Msg { data: temp }).await?;
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
```

```rust
// Subscriber node (controller.rs)
use mini_ros::{node::Node, message::Float64Msg, context::Context};

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    let context = Context::new();
    let mut node = Node::new("controller_node", context).await?;
    let mut temp_sub = node.create_subscriber::<Float64Msg>("/temperature").await?;
    
    temp_sub.set_callback(|msg: Float64Msg| {
        if msg.data > 25.0 {
            println!("Temperature high: {:.1}°C", msg.data);
        }
    })?;
    
    // Keep node running
    tokio::signal::ctrl_c().await.unwrap();
    Ok(())
}
```

### Error Handling

Always handle errors appropriately:

```rust
use mini_ros::error::{Result, MiniRosError};

async fn robot_operation() -> Result<()> {
    let context = Context::new();
    let node = Node::new("robot", context).await
        .map_err(|e| {
            eprintln!("Failed to create node: {}", e);
            e
        })?;
    
    // Your robot logic here
    Ok(())
}

#[tokio::main]
async fn main() {
    if let Err(e) = robot_operation().await {
        eprintln!("Robot operation failed: {}", e);
        std::process::exit(1);
    }
}
```

## Tips for Success

1. **Start Simple**: Begin with basic pub/sub before adding complexity
2. **Use Logging**: The `tracing` crate helps debug issues
3. **Handle Errors**: Always use proper error handling
4. **Organize Code**: Separate nodes into different modules/files
5. **Test Incrementally**: Build and test small pieces first

## Getting Help

- Check the [examples directory](https://github.com/your-username/miniROS-rs/tree/main/examples) for working code
- Read the [API documentation](../api/core.md) for detailed function references
- Join the [community discussions](https://github.com/your-username/miniROS-rs/discussions) for questions and help

Ready to build amazing robotics applications with miniROS-rs! 🚀🤖
