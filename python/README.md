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

## Quick Setup

```bash
# Install (requires Rust)
pip install maturin
maturin develop --features python
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

## ROS2 Migration

### Same Patterns
- Node creation: `mini_ros.Node('name')`
- Publishers: `node.create_publisher(Type, topic, qos)`
- Subscribers: `node.create_subscription(Type, topic, callback, qos)`
- Services: `node.create_service(Type, name, callback)`
- Spinning: `mini_ros.spin(node)`

### Same Lifecycle
```python
# ROS2 pattern
rclpy.init()
node = rclpy.create_node('test')
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

# miniROS pattern (identical)
mini_ros.init()
node = mini_ros.Node('test')
mini_ros.spin(node)
node.destroy_node()
mini_ros.shutdown()
```

## Examples

### Run Examples
```bash
cd python/examples

# Basic communication
python minimal_publisher.py    # Terminal 1
python minimal_subscriber.py   # Terminal 2
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
| Import time | ~50ms | ~500ms |

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
- **Prototyping** - Fast iteration

### ❌ Use ROS2 for:
- **Complex systems** - Navigation, perception
- **Custom messages** - Complex data types
- **Large teams** - Established workflows
- **ROS ecosystem** - rviz, Gazebo, etc.

## Examples Directory

```
python/examples/
├── minimal_publisher.py      # 15 lines - basic publisher
├── minimal_subscriber.py     # 15 lines - basic subscriber  
├── talker.py                # Advanced publisher
└── listener.py              # Advanced subscriber
```

All examples use **identical ROS2 patterns**.

---

*miniROS Python: ROS2 compatibility, minimal complexity* 