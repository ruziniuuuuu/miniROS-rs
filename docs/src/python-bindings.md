# Python Bindings

miniROS provides Python bindings with ROS2-compatible API for easy migration and rapid prototyping.

## Quick Setup

### Recommended: Using uv (10-100x faster than pip)

```bash
# Install uv (extremely fast Python package manager)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install build tool
uv tool install maturin

# Build and install
maturin develop --features python

# Verify
python -c "import mini_ros; print('✅ miniROS Python ready!')"
```

### Alternative: Using pip

```bash
# Install build tool
pip install maturin

# Build and install  
maturin develop --features python

# Verify
python -c "import mini_ros; print('✅ miniROS Python ready!')"
```

## Basic Usage

### Publisher

```python
import mini_ros
import time

mini_ros.init()
node = mini_ros.Node('talker')
pub = node.create_publisher(mini_ros.String, 'chatter', 10)

msg = mini_ros.String()
for i in range(5):
    msg.data = f'Hello {i}'
    pub.publish(msg)
    node.get_logger().info(f'Published: {msg.data}')
    time.sleep(1)

node.destroy_node()
mini_ros.shutdown()
```

### Subscriber

```python
import mini_ros

class Listener(mini_ros.Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            mini_ros.String, 'chatter', self.callback, 10)
    
    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')

mini_ros.init()
node = Listener()
# Note: spin() currently blocks indefinitely
mini_ros.spin(node)
```

## Message Types

```python
# Available message types
mini_ros.String(data="hello")      # String message
mini_ros.Int32(data=42)            # Integer message  
mini_ros.Float64(data=3.14)        # Float message
```

## API Reference

### Core Functions
- `mini_ros.init()` - Initialize system
- `mini_ros.shutdown()` - Cleanup and exit
- `mini_ros.ok()` - Check if running
- `mini_ros.spin(node)` - Process callbacks
- `mini_ros.spin_once(node, timeout_sec=None)` - Single iteration

### Node Class
- `Node(name)` - Create node
- `node.get_name()` - Get node name
- `node.get_logger()` - Get logger
- `node.create_publisher(msg_type, topic, qos)` - Create publisher
- `node.create_subscription(msg_type, topic, callback, qos)` - Create subscriber
- `node.destroy_node()` - Cleanup

## Examples

Run the included examples:

```bash
# Basic examples
python python/examples/talker.py           # Basic publisher
python python/examples/listener.py         # Basic subscriber  
python python/examples/demo_all.py         # Comprehensive demo

# Advanced examples with visualization
python python/examples/image_publisher.py  # Image processing with OpenCV
python python/examples/image_subscriber.py # Image visualization
python python/examples/robot_visualization.py # 3D robot simulation
```

### Prerequisites for Advanced Examples

```bash
# Install additional dependencies
pip install opencv-python numpy rerun-sdk

# Or using uv (recommended)
uv add opencv-python numpy rerun-sdk
```

## ROS2 Migration

miniROS Python API is designed for easy ROS2 migration:

| ROS2 | miniROS |
|------|---------|
| `import rclpy` | `import mini_ros` |
| `rclpy.init()` | `mini_ros.init()` |
| `rclpy.Node` | `mini_ros.Node` |
| `std_msgs.msg.String` | `mini_ros.String` |

## Current Limitations

- Publishers/subscribers don't communicate yet (stub implementation)
- No service/client support
- No parameter server
- Limited to String/Int32/Float64 messages
- Synchronous only (no async/await)

The Python bindings provide the API surface for future full implementation. 