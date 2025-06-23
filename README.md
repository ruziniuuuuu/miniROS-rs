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

### New: ROS2-Compatible Message Packages 📦
- **std_msgs** - Standard primitive types (String, Int32, Float64, Bool, Header)
- **geometry_msgs** - Geometric types (Point, Pose, Twist, Quaternion, etc.)
- **nav_msgs** - Navigation types (Odometry, Path, OccupancyGrid)
- **sensor_msgs** - Sensor types (LaserScan, PointCloud2, Image, IMU)
- **action_msgs** - Action system types (GoalInfo, GoalStatus, GoalStatusArray)
- **diagnostic_msgs** - System diagnostics (DiagnosticStatus, DiagnosticArray, KeyValue)
- **Full ROS2 Compatibility** - Drop-in replacement for ROS2 message types

### Package & Launch System
- **Package Management** - Organize code into reusable packages with manifests
- **Launch System** - ROS2-like launch files for multi-node orchestration
- **CLI Tools** - `mini_ros` command-line interface for easy package operations
- **Built-in Packages** - Production-ready turtlebot package included

## 🛠️ Quick Start

### 💻 API Overview

#### Rust API - ROS2 Compatible Messages
```rust
use mini_ros::prelude::*;
use mini_ros::types::{std_msgs, geometry_msgs, nav_msgs, sensor_msgs, diagnostic_msgs};

#[tokio::main]
async fn main() -> Result<()> {
    // Create and initialize node
    let mut node = Node::new("my_node")?;
    node.init().await?;

    // Publisher with ROS2-compatible message
    let pub = node.create_publisher::<std_msgs::String>("/topic").await?;
    let msg = std_msgs::String { data: "Hello ROS2!".to_string() };
    pub.publish(&msg).await?;

    // Geometry messages for robotics
    let twist = geometry_msgs::Twist {
        linear: geometry_msgs::Vector3 { x: 0.5, y: 0.0, z: 0.0 },
        angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 1.0 },
    };

    // Sensor messages for perception
    let laser_scan = sensor_msgs::LaserScan {
        header: std_msgs::Header { 
            stamp: get_current_time_ns(),
            frame_id: "laser".to_string() 
        },
        angle_min: -1.57,
        angle_max: 1.57,
        angle_increment: 0.01,
        ranges: vec![1.0, 1.1, 1.2, 1.3],
        intensities: vec![],
        // ... other fields
    };

    // Diagnostic messages for system monitoring
    let diagnostic = diagnostic_msgs::DiagnosticStatus {
        level: diagnostic_msgs::OK,
        name: "Motor Controller".to_string(),
        message: "Operating normally".to_string(),
        hardware_id: "motor_001".to_string(),
        values: vec![],
    };

    Ok(())
}
```

#### Python API (ROS2 Compatible)
```python
import mini_ros

# Initialize (same as rclpy.init())
mini_ros.init()

# Create node (same as rclpy.create_node())
node = mini_ros.Node('my_node')

# Publisher with ROS2-compatible messages (same API as rclpy)
pub = node.create_publisher(mini_ros.std_msgs.String, 'topic', 10)

# Standard ROS2 message types
msg = mini_ros.std_msgs.String()
msg.data = 'Hello miniROS!'
pub.publish(msg)

# Geometry messages for robot control
twist = mini_ros.geometry_msgs.Twist()
twist.linear.x = 0.5
twist.angular.z = 1.0

# Navigation messages
odom = mini_ros.nav_msgs.Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

# Cleanup (same as rclpy.shutdown())
mini_ros.shutdown()
```

### 🚀 One-Command Setup (Recommended)

```bash
# Clone and auto-setup everything
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# One command to setup both Rust and Python CLIs
bash scripts/setup.sh

# Restart terminal or source config
source ~/.zshrc  # for zsh
# or source ~/.bashrc  # for bash

# Ready to use!
mini_ros pkg list
mini_ros_py examples list
```

#### 🔧 Advanced Setup Options

```bash
# Show all available commands
bash scripts/setup.sh help

# Build CLIs only (no environment setup)
bash scripts/setup.sh build

# Setup environment only (if CLIs already built)
bash scripts/setup.sh env

# Test current installation
bash scripts/setup.sh test

# Clean build artifacts
bash scripts/setup.sh clean

# Smart wrapper (auto-selects best CLI)
bash scripts/mini_ros_wrapper.sh pkg list      # → Uses Rust CLI
bash scripts/mini_ros_wrapper.sh examples list # → Uses Python CLI
```

### 🎯 Alternative: Manual Setup

#### Using mini_ros CLI (Recommended)

```bash
# Build the project
cargo build

# List available packages
cargo run --bin mini_ros pkg list

# Run turtlebot demos
cargo run --bin mini_ros run turtlebot controller
cargo run --bin mini_ros run turtlebot teleop
cargo run --bin mini_ros launch turtlebot full_system
```

#### Learning Examples (Progressive)

```bash
# Basic pub/sub communication
cargo run --example 01_basic_pubsub

# Custom message types
cargo run --example 02_custom_messages

# Service communication
cargo run --example 03_services

# ROS2-compatible message packages (NEW!)
cargo run --example 16_message_packages_demo

# Visualization with Rerun
cargo run --example 04_visualization_basic --features visualization

# Complete system integration
cargo run --example 07_integrated_system
```

#### Turtlebot Package (Production Ready)

```bash
# Individual components
cargo run --bin mini_ros run turtlebot controller
cargo run --bin mini_ros run turtlebot teleop
cargo run --bin mini_ros run turtlebot simulator

# Complete systems via launch files
cargo run --bin mini_ros launch turtlebot simulation
cargo run --bin mini_ros launch turtlebot teleop  
cargo run --bin mini_ros launch turtlebot full_system
```

#### Python Examples

```bash
cd python/examples

# Minimal publisher/subscriber
python minimal_publisher.py
python minimal_subscriber.py

# Complete demos
python simple_pubsub.py
python comprehensive_demo.py

# Turtlebot Python controller
cargo run --bin mini_ros run turtlebot py_controller
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
