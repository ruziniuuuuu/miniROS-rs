# Examples

Learn miniROS with focused examples. Each demonstrates **one core concept**.

## Quick Start

```bash
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# Run any example
cargo run --example 01_basic_pubsub
```

## Core Examples (Rust)

### 01 - Basic Pub/Sub
```bash
cargo run --example 01_basic_pubsub
```

**What you'll learn**: Create publishers and subscribers
- Node creation
- Topic-based communication  
- Message types

### 02 - Custom Messages
```bash
cargo run --example 02_custom_messages
```

**What you'll learn**: Define your own message types
- Serde serialization
- Custom structs as messages
- Type safety

### 03 - Services
```bash
cargo run --example 03_services
```

**What you'll learn**: Request/response communication
- Service servers
- Service clients
- Synchronous communication

## Python Examples

All Python examples use **identical ROS2 API**:

### Minimal Publisher
```bash
python python/examples/minimal_publisher.py
```

```python
import mini_ros

mini_ros.init()
node = mini_ros.Node('publisher')
pub = node.create_publisher(mini_ros.String, '/topic', 10)

msg = mini_ros.String()
msg.data = 'Hello!'
pub.publish(msg)

node.destroy_node()
mini_ros.shutdown()
```

### Minimal Subscriber
```bash
python python/examples/minimal_subscriber.py
```

```python
import mini_ros

def callback(msg):
    print(f'Received: {msg.data}')

mini_ros.init()
node = mini_ros.Node('subscriber')
sub = node.create_subscription(mini_ros.String, '/topic', callback, 10)
mini_ros.spin(node)
```

## Advanced Examples (Optional Features)

### Actions (Long-running tasks)
```bash
cargo run --example 04_actions --features actions
```

### Parameters (Dynamic config)
```bash
cargo run --example 05_parameters --features parameters
```

### Visualization (3D display)
```bash
cargo run --example 06_visualization --features visualization
```

## Multi-Terminal Demo

### Terminal 1 - Publisher
```bash
cargo run --example 01_basic_pubsub
```

### Terminal 2 - Subscriber  
```bash
cargo run --example 01_basic_pubsub
```

### Terminal 3 - Python Node
```bash
python python/examples/minimal_subscriber.py
```

All nodes automatically discover each other. No configuration needed.

## Transport Options

### Default TCP
```bash
cargo run --example 01_basic_pubsub
```

### ROS2 Compatible DDS
```bash
cargo run --example 01_basic_pubsub --features dds-transport
```

Interoperates with ROS2 nodes using DDS.

## Example Output

```
[INFO] Node 'publisher' initialized
[INFO] Publishing to '/chat': Hello miniROS!
[INFO] Node 'subscriber' received: Hello miniROS!
```

Clean, minimal logging. No debug noise.

## Troubleshooting

### Port Conflicts
```bash
# Use different port
MINI_ROS_PORT=8080 cargo run --example 01_basic_pubsub
```

### Discovery Issues
```bash
# Check network
ping localhost

# Restart with debug
RUST_LOG=debug cargo run --example 01_basic_pubsub
```

## Next Steps

1. **Start with 01-03** - Core patterns
2. **Try Python examples** - ROS2 compatibility  
3. **Add features** - Actions, parameters as needed
4. **Build your robot** - Use patterns in your project

---

*Examples: Learn by doing, one concept at a time* 