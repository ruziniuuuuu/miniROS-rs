# Introduction

miniROS-rs is a **lightweight, high-performance** robotics middleware implementation written in Rust with Python bindings. It provides core communication primitives for building distributed robotics systems.

## Core Philosophy

**"Mini" means focused** - miniROS-rs implements essential robotics communication patterns without the complexity of a full ROS2 distribution:

- **Publisher/Subscriber** - Type-safe message passing
- **Services** - Request/response communication  
- **Actions** - Long-running task management
- **Parameters** - Dynamic configuration
- **Discovery** - Automatic node detection

## Key Features

### Multi-Language Support
- **Rust Core** - High-performance implementation with zero-cost abstractions
- **Python Bindings** - ROS2 rclpy-compatible API for easy adoption
- **Type Safety** - Compile-time message validation and serialization

### Transport Flexibility
- **Memory Broker** - In-process communication for single-node applications
- **TCP Transport** - Reliable network communication with automatic reconnection
- **DDS Transport** - Industry-standard distributed communication
- **UDP Transport** - Low-latency multicast for real-time data

### Built-in Visualization
- **Rerun Integration** - 3D visualization for robotics data
- **Real-time Streaming** - Live sensor data and robot state visualization
- **Multiple Data Types** - Point clouds, poses, images, and custom data

## Architecture

miniROS-rs follows a modular architecture:

```
┌─────────────────┐    ┌─────────────────┐
│   Application   │    │   Python API    │
│      Code       │    │  (rclpy-like)   │
└─────────────────┘    └─────────────────┘
         │                       │
         └───────────────────────┤
                                 ▼
         ┌─────────────────────────────────┐
         │         Rust Core              │
         │  • Node Management             │
         │  • Message Serialization       │
         │  • Type System                 │
         └─────────────────────────────────┘
                         │
                         ▼
         ┌─────────────────────────────────┐
         │      Transport Layer           │
         │  • Memory Broker               │
         │  • TCP/UDP                     │
         │  • DDS                         │
         └─────────────────────────────────┘
```

## When to Use miniROS-rs

### ✅ Ideal For:
- **Learning robotics concepts** - Simple, clear APIs
- **High-performance applications** - Rust's zero-cost abstractions
- **Prototyping** - Quick iteration with Python compatibility
- **Embedded systems** - Minimal resource requirements
- **Cross-platform development** - Works on Linux, macOS, Windows

### 🤔 Consider ROS2 For:
- **Large, complex systems** - Full ecosystem with navigation, perception
- **Established workflows** - Teams already using ROS2 tools
- **Third-party packages** - Extensive package ecosystem
- **Custom message types** - Complex message definitions (coming soon)

## Getting Started

The fastest way to understand miniROS-rs is through examples:

1. **[Quick Start](quick-start.md)** - 5-minute setup and basic usage
2. **[Examples](examples.md)** - Progressive learning from basic to advanced
3. **[Python Bindings](python-bindings.md)** - ROS2-compatible Python API
4. **[API Reference](api.md)** - Complete function and type documentation

## Design Principles

### Performance First
- **Async by default** - Non-blocking I/O for maximum throughput
- **Zero-copy where possible** - Minimize memory allocations
- **Type-safe serialization** - Compile-time optimization

### Developer Experience
- **Clear error messages** - Helpful debugging information
- **Familiar APIs** - ROS2-like interfaces for easy migration
- **Comprehensive documentation** - Examples for every feature

### Minimal Dependencies
- **Core functionality only** - No unnecessary features
- **Optional features** - Enable visualization, Python bindings as needed
- **Small binary size** - Suitable for resource-constrained environments

miniROS-rs aims to provide **90% of ROS2's functionality with 10% of the complexity**, making it ideal for learning, prototyping, and high-performance robotics applications.

---

*miniROS: Essential robotics, maximum efficiency* 