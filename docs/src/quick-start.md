# Quick Start

Get up and running with miniROS-rs in 5 minutes!

## Installation

Add miniROS-rs to your `Cargo.toml`:

```toml
[dependencies]
mini-ros = "0.1"
tokio = { version = "1.0", features = ["full"] }
```

## Basic Usage

### 1. Create a Node

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let mut node = Node::new("my_robot")?;
    node.init().await?;
    
    // Your robot code here...
    
    Ok(())
}
```

### 2. Publish Messages

```rust
// Create publisher
let publisher = node.create_publisher::<StringMsg>("robot_status").await?;

// Send messages
publisher.publish(&StringMsg { 
    data: "Robot is running!".to_string() 
}).await?;
```

### 3. Subscribe to Messages

```rust
// Create subscriber
let subscriber = node.create_subscriber::<StringMsg>("robot_status").await?;

// Handle incoming messages
subscriber.on_message(|msg: StringMsg| {
    println!("Received: {}", msg.data);
}).await?;
```

### 4. Visualization

```rust
use mini_ros::visualization::*;

// Start visualization
let config = VisualizationConfig {
    application_id: "MyRobot".to_string(),
    spawn_viewer: true,
};
let viz = VisualizationClient::new(config)?;

// Log data
viz.log_scalar("battery", 85.0)?;
viz.log_text("status", "All systems operational")?;
```

## Run Examples

Try the built-in examples:

```bash
# Clone the repository
git clone https://github.com/your-repo/miniROS-rs
cd miniROS-rs

# Basic communication
cargo run --example 01_basic_pubsub

# Visualization (starts GUI automatically)
cargo run --example 04_visualization_basic

# Complete robot system
cargo run --example 07_integrated_system
```

## Next Steps

- See [Examples](./examples.md) for detailed tutorials
- Learn about [Core Concepts](./concepts.md)
- Check the [API Reference](./api.md) 