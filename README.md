# miniROS-rs

[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://ruziniuuuuu.github.io/miniROS-rs/)
[![DeepWiki](https://img.shields.io/badge/wiki-DeepWiki-purple)](https://deepwiki.com/ruziniuuuuu/miniROS-rs)
[![macOS Tested](https://img.shields.io/badge/macOS-tested-brightgreen?logo=apple)](https://github.com/ruziniuuuuu/miniROS-rs)
[![Rust](https://img.shields.io/badge/rust-1.70+-orange?logo=rust)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue)](LICENSE-MIT)

A **lightweight, high-performance** ROS2-compatible middleware implementation in Rust with Python bindings.

> **"Mini" Philosophy**: Focus on **core functionality** - pub/sub, services, parameters, actions, and visualization. Maximum performance with minimum complexity.

## 🚀 Key Features

### Core Communication
- **Publisher/Subscriber** - Type-safe message passing with async support
- **Services** - Request/response communication patterns  
- **Actions** - Long-running task management with goal/feedback/result
- **Parameters** - Dynamic configuration system

### Advanced Features
- **Custom Messages** - Define structured data types with automatic serialization
- **Visualization** - Built-in Rerun integration for 3D data visualization
- **Discovery** - Automatic node and service discovery across network
- **Multi-Transport** - DDS, TCP, UDP, and in-memory transport options
- **Python Bindings** - ROS2 rclpy-compatible API for easy migration

## 🛠️ Quick Start

### Rust API

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create and initialize node
    let mut node = Node::new("my_node")?;
    node.init().await?;

    // Publisher
    let pub = node.create_publisher::<StringMsg>("/topic").await?;
    pub.publish(&StringMsg { data: "Hello!".into() }).await?;

    // Subscriber with callback
    let sub = node.create_subscriber::<StringMsg>("/topic").await?;
    sub.on_message(|msg| println!("Received: {}", msg.data))?;

    Ok(())
}
```

### Python API (ROS2 Compatible)

```python
import mini_ros

# Initialize (same as rclpy.init())
mini_ros.init()

# Create node (same as rclpy.create_node())
node = mini_ros.Node('my_node')

# Publisher (same as node.create_publisher())
pub = node.create_publisher(mini_ros.StringMessage, 'topic', 10)

# Subscriber (same as node.create_subscription())
def callback(msg):
    print(f'Received: {msg.data}')

sub = node.create_subscription(mini_ros.StringMessage, 'topic', callback, 10)

# Publish message
msg = mini_ros.StringMessage()
msg.data = 'Hello miniROS!'
pub.publish(msg)

# Cleanup (same as rclpy.shutdown())
mini_ros.shutdown()
```

## 📦 Installation

### Rust

```bash
# Add to Cargo.toml
[dependencies]
mini-ros = "0.1"

# With visualization
mini-ros = { version = "0.1", features = ["visualization"] }
```

### Python

```bash
# Install from source
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs
pip install -e .

# With visualization support
pip install -e ".[viz]"
```

## 🏃‍♂️ Examples

### Rust Examples (Progressive Learning)

```bash
# Basic pub/sub communication
cargo run --example 01_basic_pubsub

# Custom message types
cargo run --example 02_custom_messages

# Service communication
cargo run --example 03_services

# Visualization with Rerun
cargo run --example 04_visualization_basic --features visualization

# Complete system integration
cargo run --example 07_integrated_system

# Turtlebot controller (classic ROS robotics)
cargo run --example 12_turtlebot_controller

# Turtlebot keyboard control (teleop)
cargo run --example 13_turtlebot_teleop

# Turtlebot simulator with Rerun visualization
cargo run --example 14_turtlebot_simulator --features visualization
```

### Python Examples

```bash
cd python/examples

# Minimal publisher
python minimal_publisher.py

# Minimal subscriber  
python minimal_subscriber.py

# Complete pub/sub demo
python simple_pubsub.py

# Turtlebot controller (robotics control)
python turtlebot_controller.py
```

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐
│   Python API    │    │    Rust Core    │
│   (rclpy-like)  │◄──►│  (Performance)  │
└─────────────────┘    └─────────────────┘
         │                       │
         └───────────────────────┤
                                 ▼
         ┌─────────────────────────────────┐
         │      Transport Layer           │
         │  • Memory Broker (local)       │
         │  • TCP (network)               │
         │  • UDP (multicast)             │
         │  • DDS (ROS2 compatibility)    │
         └─────────────────────────────────┘
```

## 🎯 Roadmap

### Phase 1: Core ✅
- [x] Basic pub/sub with memory broker
- [x] Services with request/response patterns
- [x] Custom message definitions
- [x] Python bindings (rclpy-compatible)
- [x] Discovery service
- [x] Documentation and examples

### Phase 2: Advanced ✅
- [x] Actions with goal/feedback/result
- [x] Parameters with dynamic configuration
- [x] Visualization (Rerun integration)
- [x] Cross-language type system
- [x] DDS transport layer

### Phase 3: Ecosystem 🔮
- [ ] ROS2 bridge compatibility
- [ ] Additional language bindings
- [ ] Plugin system
- [ ] Production deployment tools

## 🔧 Development

```bash
# Build the project
cargo build

# Run tests
cargo test

# Run examples
cargo run --example 01_basic_pubsub

# Build Python bindings
cd python && pip install -e .

# Build documentation
cd docs && mdbook serve
```

## 📚 Documentation

- **[Complete Documentation](https://ruziniuuuuu.github.io/miniROS-rs/)** - API reference and tutorials
- **[DeepWiki](https://deepwiki.com/ruziniuuuuu/miniROS-rs)** - Community knowledge base

## 🤝 Contributing

Contributions are welcome! Please read our [contributing guidelines](CONTRIBUTING.md) and submit pull requests for any improvements.

## 📄 License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

- Inspired by ROS2 and rclpy
- Built with Rust and PyO3
- Visualization powered by Rerun

---

**miniROS-rs**: *Maximum robotics performance, minimum complexity* 🤖⚡
