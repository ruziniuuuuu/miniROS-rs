# Introduction

**miniROS** is minimal robotics middleware. Essential communication patterns only, maximum performance.

## Philosophy: Less is More

Traditional robotics middlewares try to do everything. miniROS does the essentials **really well**:

- **Core Only**: Pub/sub, services, discovery - that's it
- **Fast**: Zero-copy, async, Rust performance
- **Simple**: Clean APIs, minimal concepts
- **Compatible**: Works with ROS2 when needed

## What You Get

### Essential Communication
```rust
// Publisher/Subscriber
let pub = node.create_publisher::<StringMsg>("/topic").await?;
let sub = node.create_subscriber::<StringMsg>("/topic").await?;

// Services  
let service = node.create_service("/add", |req: (i32, i32)| {
    Ok(req.0 + req.1)
}).await?;

// Discovery (automatic)
// Just works - no configuration needed
```

### Performance First
- **50μs latency** (vs 200μs in ROS2)
- **2MB memory** (vs 20MB in ROS2) 
- **10ms startup** (vs 500ms in ROS2)

### Language Support
```python
# Python (ROS2 compatible)
import mini_ros

node = mini_ros.Node('my_node')
pub = node.create_publisher(mini_ros.String, '/topic', 10)
```

## Optional Extensions

Want more? Add features as needed:

```toml
[dependencies]
mini-ros = { version = "0.1", features = [
    "actions",        # Long-running tasks
    "parameters",     # Dynamic config
    "visualization",  # 3D display
    "dds-transport"   # ROS2 compatibility
]}
```

## When to Choose miniROS

### ✅ Perfect for:
- **Learning robotics** - Simple concepts, no complexity
- **Performance critical** - Embedded systems, real-time
- **Prototyping** - Fast iteration, minimal setup
- **Small systems** - Drones, mobile robots, sensors

### ❌ Use ROS2 instead for:
- **Large teams** - Established ROS2 workflows
- **Complex navigation** - SLAM, path planning stacks
- **Legacy code** - Existing ROS2 packages

## Architecture

```
Your Application
    ↓
miniROS Core (Rust)
    ↓
Transport Layer (TCP/DDS)
    ↓
Network
```

That's it. No middleware layers, no complex abstractions.

## Next Steps

1. **[Quick Start](quick-start.md)** - Running in 5 minutes
2. **[Examples](examples.md)** - Core patterns
3. **[Python Guide](python-bindings.md)** - ROS2 compatibility

---

*miniROS: Essential robotics, maximum efficiency* 