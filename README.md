# MiniROS-rs

A high-performance, cross-platform ROS2-like middleware written in Rust, focusing on implementing the core communication features for robotics systems.

## Features

- 🚀 **High Performance**: Built on Tokio async runtime with support for concurrent message processing
- 🌐 **Cross-Platform**: Supports Linux, macOS, and Windows
- 🔗 **Multiple Transports**: UDP and TCP transport support
- 🔍 **Auto Discovery**: Multicast-based node and service discovery
- 📨 **Pub/Sub**: Decoupled message passing mechanism
- 🔧 **Service Calls**: Synchronous request/response communication pattern
- 🧩 **Type Safety**: Compile-time type checking and serialization
- 📝 **Clean API**: Easy-to-use modern Rust API

## Quick Start

### Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
mini-ros = "0.1.0"
tokio = { version = "1.0", features = ["full"] }
```

### Basic Usage

#### Publisher Example

```rust
use mini_ros::prelude::*;
use mini_ros::message::StringMsg;

#[tokio::main]
async fn main() -> Result<()> {
    // Create node
    let mut node = Node::new("publisher_node")?;
    node.init().await?;
    
    // Create publisher
    let publisher = node.create_publisher::<StringMsg>("chatter").await?;
    
    // Publish message
    let message = StringMsg {
        data: "Hello, MiniROS!".to_string(),
    };
    publisher.publish(&message).await?;
    
    node.shutdown().await?;
    Ok(())
}
```

#### Subscriber Example

```rust
use mini_ros::prelude::*;
use mini_ros::message::StringMsg;

#[tokio::main]
async fn main() -> Result<()> {
    // Create node
    let mut node = Node::new("subscriber_node")?;
    node.init().await?;
    
    // Create subscriber
    let subscriber = node.create_subscriber::<StringMsg>("chatter").await?;
    
    // Set message callback
    subscriber.on_message(|msg: StringMsg| {
        println!("Received: {}", msg.data);
    })?;
    
    // Keep node running
    node.spin().await?;
    Ok(())
}
```

## Architecture Overview

MiniROS uses a layered architecture design:

```
┌─────────────────────────────────────────┐
│           Application Layer             │
├─────────────────────────────────────────┤
│  Node Management & Service Discovery    │
├─────────────────────────────────────────┤
│    Publisher/Subscriber & Services      │
├─────────────────────────────────────────┤
│      Message Serialization Layer       │
├─────────────────────────────────────────┤
│      Transport Layer (UDP/TCP)         │
├─────────────────────────────────────────┤
│        Network & OS Interfaces         │
└─────────────────────────────────────────┘
```

### Core Components

- **Node**: Computational units that can publish, subscribe to messages and provide services
- **Publisher/Subscriber**: Implements publish-subscribe messaging patterns
- **Service/ServiceClient**: Implements request-response communication patterns
- **Discovery**: Multicast-based node and service discovery mechanism
- **Transport**: Pluggable transport layer supporting UDP and TCP
- **Message**: Type-safe message serialization system

## Built-in Message Types

MiniROS provides common built-in message types:

```rust
use mini_ros::message::*;

// String message
let string_msg = StringMsg { data: "Hello".to_string() };

// Numeric messages
let int_msg = Int32Msg { data: 42 };
let float_msg = Float64Msg { data: 3.14 };

// Boolean message
let bool_msg = BoolMsg { data: true };

// Empty message (for triggering events)
let empty_msg = EmptyMsg;

// Timestamped message
let stamped = Stamped::new("Hello".to_string(), 1, "frame_id".to_string());
```

## Custom Messages

You can easily define custom message types:

```rust
use serde::{Deserialize, Serialize};
use mini_ros::message::Message;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    x: f64,
    y: f64,
    theta: f64,
}

// Message trait is automatically implemented

// Use custom message
let mut node = Node::new("robot_node")?;
node.init().await?;

let publisher = node.create_publisher::<RobotPose>("robot_pose").await?;
let pose = RobotPose { x: 1.0, y: 2.0, theta: 0.5 };
publisher.publish(&pose).await?;
```

## Service Example

```rust
use mini_ros::prelude::*;
use mini_ros::message::{StringMsg, Int32Msg};

#[tokio::main]
async fn main() -> Result<()> {
    // Service provider
    let mut server_node = Node::new("server_node")?;
    server_node.init().await?;
    
    let _service = server_node.create_service(
        "string_length",
        |req: StringMsg| -> Result<Int32Msg> {
            Ok(Int32Msg { data: req.data.len() as i32 })
        }
    ).await?;
    
    // Service client
    let client_node = Node::new("client_node")?;
    client_node.init().await?;
    
    let client = client_node.create_service_client::<StringMsg, Int32Msg>("string_length").await?;
    
    // Wait for service availability
    client.wait_for_service(Duration::from_secs(5)).await?;
    
    // Call service
    let request = StringMsg { data: "Hello".to_string() };
    let response = client.call(request).await?;
    println!("String length: {}", response.data);
    
    Ok(())
}
```

## Running Examples

Clone the project and run built-in examples:

```bash
git clone <repository-url>
cd miniROS-rs

# Run simple pub/sub example
cargo run --example simple_pub_sub

# Run visualization demo
cargo run --example visualization_demo

# Run Zenoh communication example  
cargo run --example zenoh_communication

# Run integrated demo (Zenoh + Rerun)
cargo run --example integrated_demo

# Run tests
cargo test

# Build release version
cargo build --release
```

## 📚 Documentation

Complete documentation is available as an interactive book built with [mdBook](https://rust-lang.github.io/mdBook/):

```bash
cd docs
mdbook serve --open
```

The documentation includes:
- **Getting Started**: [Installation](docs/src/getting-started/installation.md) and [Quick Start](docs/src/getting-started/quick-start.md)
- **Core Concepts**: Nodes, messages, pub/sub, and services
- **Visualization**: Complete [Rerun integration](docs/src/visualization/rerun.md) guide with [examples](docs/src/visualization/examples.md)
- **Communication**: [Zenoh transport](docs/src/communication/zenoh.md) and discovery service
- **API Reference**: Detailed function documentation
- **Advanced Topics**: Performance tuning and custom development

### Key Documentation Features
- 🌐 **English Documentation**: Complete international accessibility
- 📊 **Visualization Guide**: Comprehensive Rerun integration with working examples
- 🔍 **Interactive Search**: Full-text search across all documentation
- 📱 **Responsive Design**: Works on mobile and desktop
- 🖨️ **Print-Friendly**: Optimized for PDF export

## Configuration

### Domain ID

Use different domain IDs to isolate different robot systems:

```rust
let context = Context::with_domain_id(42)?;
let node = Node::with_context("my_node", context)?;
```

### Transport Configuration

```rust
// Use UDP (default)
publisher.publish(&message).await?;

// Or explicitly specify transport protocol
let udp_endpoint = "udp://127.0.0.1:8000";
let tcp_endpoint = "tcp://127.0.0.1:8000";
```

## Performance

MiniROS is optimized for performance:

- **Zero-copy Serialization**: Efficient binary serialization using bincode
- **Async I/O**: Tokio-based asynchronous network communication
- **Concurrent Processing**: Support for multiple publishers/subscribers working concurrently
- **Lock-free Data Structures**: Lock-free algorithms in critical paths

## Cross-Platform Support

MiniROS supports the following platforms:

- **Linux**: Full support, including multicast discovery
- **macOS**: Full support
- **Windows**: Basic functionality support (multicast features may be limited)

## Comparison with ROS2

| Feature | MiniROS | ROS2 |
|---------|---------|------|
| Language | Rust | C++/Python |
| Performance | High | Medium |
| Memory Safety | Compile-time guaranteed | Runtime checks |
| Cross-platform | Native support | Partial support |
| Learning Curve | Medium | Steep |
| Ecosystem | Emerging | Mature |

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the project
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is dual-licensed under MIT or Apache-2.0. See [LICENSE-MIT](LICENSE-MIT) and [LICENSE-APACHE](LICENSE-APACHE) files for details.

## Roadmap

- [ ] Complete service response handling
- [ ] Parameter server
- [ ] Action support
- [ ] Message filtering and transformation
- [ ] Performance monitoring and diagnostics
- [ ] More built-in message types
- [ ] ROS2 interoperability
- [ ] Graphical debugging tools

## Contact

For questions or suggestions, please contact:

- Submit Issues: [GitHub Issues](https://github.com/ChenyuCao/miniROS-rs/issues)
- Email: ruziniuuuuu@gmail.com

---

*MiniROS - Making robot communication simpler, faster, and safer*
