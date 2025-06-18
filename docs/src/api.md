# API Reference

Essential API documentation for miniROS-rs.

## Core Types

### Node

```rust
impl Node {
    // Create new node
    pub fn new(name: &str) -> Result<Self>
    
    // Initialize node (must call before use)
    pub async fn init(&mut self) -> Result<()>
    
    // Create publisher
    pub async fn create_publisher<T: Message>(&self, topic: &str) -> Result<Publisher<T>>
    
    // Create subscriber  
    pub async fn create_subscriber<T: Message>(&self, topic: &str) -> Result<Subscriber<T>>
    
    // Create service provider
    pub async fn create_service<Req, Res, F>(&self, name: &str, callback: F) -> Result<Service<Req, Res>>
    where F: Fn(Req) -> Result<Res> + Send + Sync + 'static
    
    // Create service client
    pub async fn create_service_client<Req: Message, Res: Message>(&self, name: &str) -> Result<ServiceClient<Req, Res>>
}
```

### Publisher

```rust
impl<T: Message> Publisher<T> {
    // Publish message
    pub async fn publish(&self, message: &T) -> Result<()>
    
    // Get topic name
    pub fn topic(&self) -> &str
}
```

### Subscriber

```rust
impl<T: Message> Subscriber<T> {
    // Set message callback
    pub async fn on_message<F>(&self, callback: F) -> Result<()>
    where F: Fn(T) + Send + 'static
    
    // Get topic name
    pub fn topic(&self) -> &str
}
```

### ServiceClient

```rust
impl<Req: Message, Res: Message> ServiceClient<Req, Res> {
    // Call service
    pub async fn call(&self, request: Req) -> Result<Res>
    
    // Call with timeout
    pub async fn call_with_timeout(&self, request: Req, timeout: Duration) -> Result<Res>
    
    // Wait for service to become available
    pub async fn wait_for_service(&self, timeout: Duration) -> Result<()>
}
```

## Message Types

### Built-in Messages

```rust
// String message
pub struct StringMsg {
    pub data: String,
}

// Numeric messages
pub struct Int32Msg {
    pub data: i32,
}

pub struct Float64Msg {
    pub data: f64,
}

// Stamped message (with timestamp)
pub struct Stamped<T> {
    pub header: Header,
    pub data: T,
}

pub struct Header {
    pub stamp: u64,     // Timestamp in nanoseconds
    pub seq: u32,       // Sequence number
    pub frame_id: String, // Reference frame
}
```

### Custom Messages

```rust
use serde::{Deserialize, Serialize};

// Any struct with Serialize + Deserialize + Clone + Send + Sync automatically implements Message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomMessage {
    pub field1: String,
    pub field2: f64,
    pub field3: Vec<i32>,
}
```

## Visualization

### VisualizationClient

```rust
impl VisualizationClient {
    // Create new client
    pub fn new(config: VisualizationConfig) -> Result<Self>
    
    // Log scalar value (creates time series plot)
    pub fn log_scalar(&self, entity_path: &str, value: f64) -> Result<()>
    
    // Log text message
    pub fn log_text(&self, entity_path: &str, text: &str) -> Result<()>
    
    // Log 2D points
    pub fn log_points_2d(&self, entity_path: &str, points: Vec<[f32; 2]>) -> Result<()>
    
    // Log 3D points
    pub fn log_points_3d(&self, entity_path: &str, points: Vec<[f32; 3]>) -> Result<()>
    
    // Log 3D transform
    pub fn log_transform_3d(&self, entity_path: &str, translation: [f32; 3], rotation: [f32; 4]) -> Result<()>
}
```

### VisualizationConfig

```rust
pub struct VisualizationConfig {
    pub application_id: String,  // Application name
    pub spawn_viewer: bool,      // Auto-start GUI
}
```

### Visualizable Trait

```rust
pub trait Visualizable {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()>;
}

// Implemented for:
// - RobotPose
// - PointCloud  
// - LaserScan
```

### Robot Data Types

```rust
// Robot pose with position and orientation
pub struct RobotPose {
    pub position: [f32; 3],      // [x, y, z]
    pub orientation: [f32; 4],   // Quaternion [x, y, z, w]
}

// 3D point cloud
pub struct PointCloud {
    pub points: Vec<[f32; 3]>,   // Array of [x, y, z] points
}

// 2D laser scan
pub struct LaserScan {
    pub ranges: Vec<f32>,        // Distance measurements
    pub angle_min: f32,          // Start angle (radians)
    pub angle_max: f32,          // End angle (radians)
}
```

## Error Handling

### Result Type

```rust
pub type Result<T> = std::result::Result<T, MiniRosError>;
```

### Error Types

```rust
pub enum MiniRosError {
    NetworkError(String),        // Network/transport errors
    SerializationError(String),  // Message serialization errors
    NodeError(String),          // Node creation/management errors
    ServiceError(String),       // Service call errors
    Other(String),              // Other errors
}
```

## Async Runtime

miniROS-rs requires Tokio async runtime:

```rust
#[tokio::main]
async fn main() -> Result<()> {
    // Your miniROS code here
    Ok(())
}
```

Or in non-main functions:

```rust
use tokio::runtime::Runtime;

fn sync_function() -> Result<()> {
    let rt = Runtime::new().unwrap();
    rt.block_on(async {
        // Async miniROS code here
        Ok(())
    })
}
```

## Feature Flags

Available Cargo features:

```toml
[dependencies]
mini-ros = { version = "0.1", features = ["visualization"] }
```

- `visualization` - Enable Rerun visualization support (default)

## Example Usage Patterns

### Basic Node Setup

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let mut node = Node::new("my_node")?;
    node.init().await?;
    
    // Use node...
    
    Ok(())
}
```

### Publisher Pattern

```rust
let publisher = node.create_publisher::<StringMsg>("topic").await?;

loop {
    publisher.publish(&StringMsg { data: "Hello".to_string() }).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
}
```

### Subscriber Pattern

```rust
let subscriber = node.create_subscriber::<StringMsg>("topic").await?;

subscriber.on_message(|msg: StringMsg| {
    println!("Received: {}", msg.data);
}).await?;

// Keep node alive
tokio::signal::ctrl_c().await.unwrap();
``` 