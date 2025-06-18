# Core Concepts

Understanding the building blocks of miniROS-rs.

## Nodes

**Nodes** are independent processes that communicate with each other.

```rust
// Create a node
let mut node = Node::new("robot_controller")?;
node.init().await?;

// Each node has a unique name and can create publishers/subscribers
```

**Key Points:**
- Each node runs independently
- Nodes communicate via topics and services
- Multiple nodes can run in the same process

## Messages

**Messages** are data structures sent between nodes.

### Built-in Messages
```rust
use mini_ros::message::*;

// Simple types
let string_msg = StringMsg { data: "Hello".to_string() };
let number_msg = Float64Msg { data: 42.0 };
let int_msg = Int32Msg { data: 100 };
```

### Custom Messages
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    x: f64,
    y: f64,
    theta: f64,
}

// Automatically implements Message trait
```

## Communication Patterns

### Publisher/Subscriber

**Asynchronous**, many-to-many communication:

```rust
// Publisher side
let publisher = node.create_publisher::<StringMsg>("robot_status").await?;
publisher.publish(&StringMsg { data: "Moving".to_string() }).await?;

// Subscriber side  
let subscriber = node.create_subscriber::<StringMsg>("robot_status").await?;
subscriber.on_message(|msg| {
    println!("Robot status: {}", msg.data);
}).await?;
```

**When to use:** Sensor data, status updates, continuous streams

### Services

**Synchronous**, one-to-one request/response:

```rust
// Server side
let _service = node.create_service("calculate_distance", |req: StringMsg| -> Result<Float64Msg> {
    let distance = req.data.len() as f64; // Simple calculation
    Ok(Float64Msg { data: distance })
}).await?;

// Client side
let client = node.create_service_client::<StringMsg, Float64Msg>("calculate_distance").await?;
let response = client.call(StringMsg { data: "test".to_string() }).await?;
```

**When to use:** Calculations, configuration, one-time requests

## Topics

**Topics** are named channels for message passing:

```rust
// Topic names are strings
"/robot/pose"           // Robot position
"/sensors/lidar"        // Laser scan data  
"/camera/image"         // Camera images
"/mission/status"       // Mission updates
```

**Naming Convention:**
- Use forward slashes for hierarchy
- Lowercase with underscores
- Descriptive and consistent

## Error Handling

miniROS-rs uses Rust's `Result` type:

```rust
use mini_ros::error::Result;

#[tokio::main]  
async fn main() -> Result<()> {
    let mut node = Node::new("my_node")?;  // Can fail
    node.init().await?;                    // Can fail
    
    // Handle specific errors
    match node.create_publisher::<StringMsg>("test").await {
        Ok(publisher) => { /* use publisher */ },
        Err(e) => println!("Failed to create publisher: {}", e),
    }
    
    Ok(())
}
```

## Best Practices

### Node Design
- One node per logical component (sensor, controller, planner)
- Keep nodes small and focused  
- Use descriptive names

### Message Design
- Keep messages simple and flat when possible
- Use appropriate data types (f32 vs f64)
- Include timestamps for time-sensitive data

### Communication
- Use pub/sub for streaming data
- Use services for calculations and queries
- Choose topic names carefully - they're hard to change later

## Next Steps

- See [Visualization](./visualization.md) for data display
- Check [API Reference](./api.md) for complete documentation
- Try [Examples](./examples.md) for hands-on practice 