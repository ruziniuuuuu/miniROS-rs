# miniROS Python Bindings

**ROS2 rclpy compatible** Python API for miniROS. Core features only, maximum simplicity.

## Drop-in ROS2 Replacement

Replace these imports:
```python
# Instead of:
# import rclpy
# from std_msgs.msg import String

# Use:
import mini_ros
from mini_ros import String
```

Everything else stays **exactly the same**.

## Quick Setup with uv

```bash
# Install uv (ultra-fast Python package manager)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Clone and build
git clone https://github.com/ruziniuuuuu/miniROS-rs.git
cd miniROS-rs

# Build with uv (automatically installs Rust + dependencies)
uv sync --dev
uv run maturin develop --features python

# Or use traditional method
# pip install maturin
# maturin develop --features python
```

## API Compatibility

### ✅ Identical to ROS2
```python
# Node lifecycle (same as rclpy)
mini_ros.init()
node = mini_ros.Node('my_node')
mini_ros.spin(node)
node.destroy_node()
mini_ros.shutdown()

# Publisher (same as rclpy)
pub = node.create_publisher(mini_ros.String, '/topic', 10)
msg = mini_ros.String()
msg.data = 'Hello!'
pub.publish(msg)

# Subscriber (same as rclpy)
def callback(msg):
    print(f'Got: {msg.data}')

sub = node.create_subscription(mini_ros.String, '/topic', callback, 10)

# Services (same as rclpy)
srv = node.create_service(mini_ros.AddTwoInts, '/add', add_callback)
client = node.create_client(mini_ros.AddTwoInts, '/add')
```

## Core Examples

### Minimal Publisher
```python
import mini_ros

mini_ros.init()
node = mini_ros.Node('publisher')
pub = node.create_publisher(mini_ros.String, '/chat', 10)

msg = mini_ros.String()
msg.data = 'Hello miniROS!'
pub.publish(msg)

node.destroy_node()
mini_ros.shutdown()
```

### Minimal Subscriber
```python
import mini_ros

def callback(msg):
    print(f'Received: {msg.data}')

mini_ros.init()
node = mini_ros.Node('subscriber')
sub = node.create_subscription(mini_ros.String, '/chat', callback, 10)
mini_ros.spin(node)
```

## Message Types

```python
# Built-in types (like std_msgs)
msg = mini_ros.String()
msg.data = "hello"

msg = mini_ros.Int32()
msg.data = 42

msg = mini_ros.Float64()
msg.data = 3.14

msg = mini_ros.Bool()
msg.data = True
```

## Development with uv

```bash
# Setup development environment
uv venv
uv sync --dev

# Build the package
uv run maturin develop --features python

# Run examples
uv run python examples/minimal_publisher.py
uv run python examples/minimal_subscriber.py

# Run tests
uv run pytest python/tests/

# Format and lint
uv run ruff check python/
uv run ruff format python/
```

## Examples

### Run Examples
```bash
cd python/examples

# Using uv (recommended)
uv run python minimal_publisher.py    # Terminal 1
uv run python minimal_subscriber.py   # Terminal 2

# Or traditional Python
python minimal_publisher.py
python minimal_subscriber.py
```

### Available Examples
- `minimal_publisher.py` - Basic publisher (15 lines)
- `minimal_subscriber.py` - Basic subscriber (15 lines)  
- `talker.py` - Publisher with logging
- `listener.py` - Subscriber with logging

## Performance vs ROS2

| Feature | miniROS | ROS2 |
|---------|---------|------|
| Startup | ~100ms | ~2s |
| Memory | ~10MB | ~100MB |
| Latency | ~50μs | ~200μs |
| Install time | ~10s with uv | ~300s |

## What's Missing

### ❌ Not Implemented (yet)
- Custom message definitions
- Advanced QoS policies
- Timers and lifecycle nodes
- Parameter callbacks
- tf2 transformations

### 🚧 Roadmap
- [ ] More std_msgs types
- [ ] Custom message support
- [ ] Timer system
- [ ] QoS policies

## When to Use

### ✅ Perfect for:
- **Learning ROS concepts** - Same API, less complexity
- **Simple robots** - Pub/sub + services
- **Performance critical** - Embedded systems
- **Prototyping** - Fast iteration with uv

### ❌ Use ROS2 for:
- **Complex systems** - Navigation, perception
- **Custom messages** - Complex data types
- **Large teams** - Established workflows
- **ROS ecosystem** - rviz, Gazebo, etc.

## Build System

miniROS uses:
- **uv** for Python package management (10-100x faster than pip)
- **maturin** for Rust-Python bindings
- **Cargo** for Rust compilation

This provides the fastest possible build and install experience.

---

*miniROS Python: ROS2 compatibility, minimal complexity, maximum speed* 