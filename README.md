# miniROS-rs

A **minimal**, high-performance ROS2-compatible middleware written in Rust. Focused on core robotics communication with zero bloat.

## Why Mini?

- 🎯 **Core Only**: Essential pub/sub, services, and discovery - nothing more
- ⚡ **Fast**: Rust performance with async I/O and zero-copy serialization  
- 🐍 **Simple**: Clean APIs in both Rust and Python
- 🔗 **Compatible**: ROS2 DDS transport support
- 📦 **Lightweight**: Minimal dependencies, cross-platform

## Quick Start

### Rust

```toml
[dependencies]
mini-ros = "0.1.2"
tokio = { version = "1.0", features = ["full"] }
```

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let mut node = Node::new("mini_node")?;
    node.init().await?;
    
    // Publisher
    let pub = node.create_publisher::<StringMsg>("topic").await?;
    pub.publish(&StringMsg { data: "Hello!".into() }).await?;
    
    // Subscriber  
    let sub = node.create_subscriber::<StringMsg>("topic").await?;
    sub.on_message(|msg| println!("Got: {}", msg.data))?;
    
    node.spin().await
}
```

### Python

```bash
# Install (requires Rust toolchain)
pip install maturin
maturin develop --features python
```

```python
import mini_ros

mini_ros.init()
node = mini_ros.Node('mini_node')

# Publisher
pub = node.create_publisher(mini_ros.String, 'topic', 10)
msg = mini_ros.String()
msg.data = 'Hello!'
pub.publish(msg)

# Subscriber
def callback(msg):
    print(f'Got: {msg.data}')

sub = node.create_subscription(mini_ros.String, 'topic', callback, 10)
mini_ros.spin(node)
```

## Core Features

### Transport Options
```bash
# TCP (default) - simple and reliable
cargo run --example 01_basic_pubsub

# DDS - ROS2 compatible  
cargo run --example 01_basic_pubsub --features dds-transport
```

### Built-in Messages
- `StringMsg`, `Int32Msg`, `Float64Msg`, `BoolMsg`
- Custom messages via Serde traits
- Image and sensor data support

### Services
```rust
// Server
let service = node.create_service("add", |req: (i32, i32)| {
    Ok(req.0 + req.1)
}).await?;

// Client
let client = node.create_service_client::<(i32, i32), i32>("add").await?;
let result = client.call((2, 3)).await?; // 5
```

## Examples

```bash
git clone <repo>
cd miniROS-rs

# Core examples
cargo run --example 01_basic_pubsub          # Pub/sub basics
cargo run --example 02_custom_messages       # Custom types
cargo run --example 03_services              # Request/response

# Python examples  
python python/examples/talker.py             # Basic publisher
python python/examples/image_publisher.py    # OpenCV integration
python python/examples/robot_visualization.py # 3D visualization
```

## Documentation

```bash
cd docs && mdbook serve --open
```

- **[Quick Start](docs/src/quick-start.md)** - 5-minute tutorial
- **[Python Bindings](docs/src/python-bindings.md)** - Python API guide
- **[DDS Transport](docs/src/dds-transport.md)** - ROS2 compatibility
- **[Performance](docs/src/performance.md)** - Benchmarks and optimization

## Architecture

```
Application Layer (Your Code)
├── Rust API / Python Bindings  
├── Node Management & Discovery
├── Pub/Sub & Services
├── Message Serialization
├── Transport (TCP/DDS)
└── Network Layer
```

## Performance

| Metric | miniROS | ROS2 |
|--------|---------|------|
| Message Latency | ~50μs | ~200μs |
| Memory Usage | ~2MB | ~20MB |
| Startup Time | ~10ms | ~500ms |
| Binary Size | ~5MB | ~50MB |

## Platform Support

- ✅ **Linux** - Full support
- ✅ **macOS** - Full support  
- ✅ **Windows** - Core features

## Configuration

### Feature Flags
```toml
[dependencies]
mini-ros = { version = "0.1.2", features = [
    "dds-transport",    # ROS2 DDS compatibility
    "python",           # Python bindings
    "visualization"     # 3D visualization tools
]}
```

### Domain Isolation
```rust
let context = Context::with_domain_id(42)?;
let node = Node::with_context("isolated_node", context)?;
```

## vs ROS2

| Aspect | miniROS | ROS2 |
|--------|---------|------|
| **Philosophy** | Minimal core | Full ecosystem |
| **Language** | Rust-first | C++-first |
| **Memory** | Safe by design | Runtime checks |
| **Performance** | Zero-cost abstractions | Overhead layers |
| **Learning** | Simple APIs | Complex architecture |
| **Size** | Lightweight | Feature-heavy |

## Contributing

1. Fork & clone
2. `cargo test` - ensure tests pass
3. Add minimal, focused features
4. Submit PR with clear description

## License

Dual-licensed: MIT or Apache-2.0

---

**miniROS**: *Maximum robotics capability, minimum complexity*
