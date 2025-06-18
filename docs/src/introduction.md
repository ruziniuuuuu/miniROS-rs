# Introduction

Welcome to **miniROS-rs** - a lightweight, high-performance robotics middleware written in Rust. This documentation will guide you through everything you need to know to get started with miniROS-rs.

## What is miniROS-rs?

miniROS-rs is a modern robotics middleware that combines the core concepts of ROS2 with the performance and safety of Rust. It provides:

- **🚀 High Performance**: Built with Rust for maximum speed and memory safety
- **🌐 Flexible Communication**: Support for multiple transport protocols including UDP, TCP, and Zenoh
- **📊 Rich Visualization**: Integrated Rerun visualization for real-time data display
- **🔧 Simple API**: Easy-to-use APIs inspired by ROS2 but designed for Rust
- **🎯 Minimal Footprint**: Lightweight design with only essential features

## Key Features

### Core Robotics Concepts
- **Nodes**: Independent processing units that can communicate with each other
- **Topics**: Named channels for asynchronous communication via publish/subscribe
- **Services**: Synchronous request/response communication patterns
- **Messages**: Strongly-typed data structures for safe communication

### Advanced Communication
- **Zenoh Integration**: High-performance pub/sub with Eclipse Zenoh protocol
- **Multi-transport**: Support for UDP, TCP, and custom transport layers
- **Service Discovery**: Automatic discovery of nodes and services
- **Serialization**: Efficient binary serialization with bincode

### Visualization & Monitoring
- **Rerun Integration**: Real-time visualization of robotics data
- **Data Types**: Built-in support for poses, point clouds, laser scans, and more
- **Flexible Display**: Both buffered and real-time visualization modes

## Who Should Use miniROS-rs?

miniROS-rs is perfect for:

- **Robotics Developers** who want the performance of Rust with familiar ROS concepts
- **Researchers** who need a lightweight, customizable middleware
- **Students** learning robotics programming with modern tools
- **Teams** building high-performance robotics applications

## Architecture Philosophy

miniROS-rs follows these design principles:

1. **Minimalism**: Only include essential features, keep the API simple
2. **Performance**: Optimize for speed and memory efficiency
3. **Safety**: Leverage Rust's type system for compile-time safety
4. **Extensibility**: Allow custom transport layers and message types
5. **Compatibility**: Maintain familiar concepts from ROS ecosystems

## Getting Started

Ready to dive in? Head over to the [Installation](./getting-started/installation.md) section to get miniROS-rs up and running on your system.

## Community

miniROS-rs is open source and welcomes contributions. Visit our [GitHub repository](https://github.com/your-username/miniROS-rs) to:

- Report issues
- Contribute code
- Request features
- Join discussions

## License

miniROS-rs is licensed under the MIT License. See the [LICENSE](https://github.com/your-username/miniROS-rs/blob/main/LICENSE) file for details. 