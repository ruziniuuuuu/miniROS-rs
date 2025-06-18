# miniROS Python Package

A high-performance ROS2-like middleware with Python bindings. This package provides a Python API that's compatible with ROS2 rclpy, allowing easy migration of existing ROS2 Python code.

## 🚀 Features

- **ROS2-Compatible API**: Drop-in replacement for basic rclpy functionality
- **High Performance**: Rust-powered backend for maximum efficiency  
- **Async Support**: Built-in async/await support with automatic event loop management
- **Cross-Platform**: Works on Linux, macOS, and Windows
- **Multiple Message Types**: String, Int32, Float64 with extensible architecture
- **Simple Installation**: Single pip install with no external dependencies

## 📦 Installation

### Prerequisites

- Python 3.8 or higher
- Rust toolchain (for building from source)

### Install from PyPI (Coming Soon)

```bash
pip install mini-ros
```

### Build from Source

#### Recommended: Using uv (10-100x faster than pip)

```bash
# Clone the repository
git clone https://github.com/ruziniuuuuu/miniROS-rs.git
cd miniROS-rs

# Install uv - extremely fast Python package manager written in Rust
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install maturin for building Python extensions
uv tool install maturin

# Build and install in development mode
maturin develop --features python

# Or build wheel for distribution
maturin build --features python --release
```

#### Alternative: Using pip

```bash
# Clone the repository
git clone https://github.com/ruziniuuuuu/miniROS-rs.git
cd miniROS-rs

# Install maturin for building Python extensions
pip install maturin

# Build and install in development mode
maturin develop --features python

# Or build wheel for distribution
maturin build --features python --release
```

## 🎯 Quick Start

The API is designed to be identical to ROS2 rclpy for easy migration:

### Basic Publisher (Talker)

```python
import mini_ros
import time

def main():
    # Initialize miniROS (equivalent to rclpy.init())
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('talker')
    
    # Create publisher 
    publisher = node.create_publisher(mini_ros.String, 'chatter', 10)
    
    # Create and publish messages
    msg = mini_ros.String()
    i = 0
    while mini_ros.ok():
        msg.data = f'Hello World: {i}'
        publisher.publish(msg)
        node.get_logger().info(f'Publishing: "{msg.data}"')
        i += 1
        time.sleep(1)
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Subscriber (Listener)

```python
import mini_ros

class MinimalSubscriber(mini_ros.Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            mini_ros.String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    mini_ros.init()
    minimal_subscriber = MinimalSubscriber()
    mini_ros.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    mini_ros.shutdown()

if __name__ == '__main__':
    main()
```

## 🧪 Example Nodes

The package includes several example nodes demonstrating different features:

### Run the Examples

```bash
# Terminal 1: Start the talker
python python/examples/talker.py

# Terminal 2: Start the listener  
python python/examples/listener.py

# Terminal 3: Start number publisher (publishes Int32 and Float64)
python python/examples/number_publisher.py

# Terminal 4: Start multi-subscriber (listens to all topics)
python python/examples/multi_subscriber.py
```

### Available Examples

- **`talker.py`**: Basic string publisher
- **`listener.py`**: Basic string subscriber with OOP design
- **`number_publisher.py`**: Multi-type publisher (Int32, Float64)
- **`multi_subscriber.py`**: Multi-topic subscriber with statistics

## 📚 API Reference

### Core Functions

- `mini_ros.init()`: Initialize the miniROS system
- `mini_ros.shutdown()`: Shutdown and cleanup
- `mini_ros.ok()`: Check if system is running
- `mini_ros.spin(node)`: Spin node indefinitely
- `mini_ros.spin_once(node, timeout_sec=None)`: Process callbacks once

### Node Class

```python
class Node:
    def __init__(self, node_name: str, **kwargs)
    def get_name(self) -> str
    def get_namespace(self) -> str  
    def get_logger(self)
    def create_publisher(self, msg_type, topic: str, qos_profile=10)
    def create_subscription(self, msg_type, topic: str, callback, qos_profile=10)
    def destroy_node(self)
```

### Message Types

- `mini_ros.String(data="")`: String message
- `mini_ros.Int32(data=0)`: 32-bit integer message
- `mini_ros.Float64(data=0.0)`: 64-bit float message

### Publisher Class

```python
class Publisher:
    def publish(self, msg)
    def get_subscription_count(self) -> int
```

## 🔄 ROS2 Migration Guide

miniROS is designed to be a drop-in replacement for basic ROS2 functionality:

### Direct Replacements

```python
# ROS2 rclpy              # miniROS
import rclpy              import mini_ros
rclpy.init()             mini_ros.init() 
rclpy.shutdown()         mini_ros.shutdown()
rclpy.ok()               mini_ros.ok()
rclpy.spin(node)         mini_ros.spin(node)
rclpy.Node               mini_ros.Node
```

### Message Types

```python
# ROS2                           # miniROS
from std_msgs.msg import String  mini_ros.String
from std_msgs.msg import Int32   mini_ros.Int32  
from std_msgs.msg import Float64 mini_ros.Float64
```

### What's Different

- **No QoS Configuration**: QoS parameters are accepted but ignored for compatibility
- **Simplified Logging**: Basic console logging instead of full ROS2 logging
- **No Parameters**: Parameter services not yet implemented
- **No Services**: Service/client pattern not yet implemented
- **No Actions**: Action pattern not yet implemented

## 🚧 Limitations & Roadmap

### Current Limitations

- Limited message types (String, Int32, Float64)
- No ROS2 message compatibility (std_msgs, geometry_msgs, etc.)
- No service/client support
- No parameter server
- No action server/client
- No ROS2 bag compatibility
- No DDS interoperability

### Planned Features

- [ ] Full ROS2 message type support
- [ ] Service/client implementation  
- [ ] Parameter server
- [ ] Action server/client
- [ ] Custom message types
- [ ] ROS2 bag recording/playback
- [ ] DDS bridge for ROS2 interoperability
- [ ] Visualization tools integration

## 🐛 Troubleshooting

### Common Issues

1. **Import Error**: Make sure you've built with `--features python` flag
2. **Network Error on macOS**: Known issue with UDP binding, use localhost only
3. **Async Loop Warning**: Can be safely ignored, handled automatically

### Debug Mode

```python
import logging
logging.basicConfig(level=logging.DEBUG)
mini_ros.init()
```

## 🤝 Contributing

Contributions are welcome! Please see the main repository for contribution guidelines.

## 📝 License

This project is licensed under either of

- Apache License, Version 2.0
- MIT License

at your option. 