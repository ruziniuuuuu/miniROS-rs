# Transport Architecture

miniROS provides multiple transport layers to match different use cases while maintaining ROS2 compatibility.

## Transport Layer Overview

```
┌─────────────────────────────────────────────┐
│           Application Layer                 │
│        (Your Robot Code)                    │
├─────────────────────────────────────────────┤
│          miniROS API Layer                  │
│     (Publishers, Subscribers, Services)     │
├─────────────────────────────────────────────┤
│           Transport Selection               │
│                                             │
│  ┌─────────────┐    ┌─────────────────────┐ │
│  │ TCP/UDP     │    │ DDS (ROS2 Compat)  │ │
│  │ Transport   │    │ Transport           │ │
│  │ (Simple)    │    │ (Standard)          │ │
│  └─────────────┘    └─────────────────────┘ │
├─────────────────────────────────────────────┤
│           Network Layer                     │
│         (UDP/TCP Sockets)                   │
└─────────────────────────────────────────────┘
```

## Why Multiple Transports?

### DDS Transport (Default, ROS2 Compatible)
- **Purpose**: Full ROS2 compatibility and interoperability
- **Protocol**: Data Distribution Service (DDS) over UDP
- **Features**:
  - Domain isolation (0-232)
  - Quality of Service (QoS) policies
  - Automatic discovery
  - ROS2 ecosystem compatibility
- **Use when**: You need ROS2 interoperability or multi-robot systems

```rust
// Enable with feature flag
cargo run --features dds-transport

// Programmatic usage
let context = Context::with_domain_id(42)?;  // Isolated domain
let transport = DdsTransport::new(42).await?;
```

### TCP Transport (Simple, Development)
- **Purpose**: Simple development and testing
- **Protocol**: Direct TCP connections
- **Features**:
  - Reliable delivery
  - Simple configuration
  - Lower overhead
  - No external dependencies
- **Use when**: Local development, testing, or simple scenarios

```rust
// Enable with feature flag (default)
cargo run --features tcp-transport

// Programmatic usage
let context = Context::new()?;  // Default TCP
```

## DDS vs TCP/UDP Clarification

**Important**: DDS is NOT a replacement for TCP/UDP - it's built ON TOP of UDP.

### The Layering

```
ROS2 Application
       ↓
   DDS Layer      ← miniROS DDS transport implements this
       ↓
   UDP Layer      ← Network protocol (standard UDP sockets)
       ↓
   IP Layer
       ↓
   Ethernet
```

### miniROS TCP Transport

For simplicity, miniROS also provides direct TCP transport that **bypasses** DDS:

```
miniROS Application
       ↓
  TCP Transport    ← Direct TCP, no DDS overhead
       ↓
   TCP Layer      ← Standard TCP sockets
       ↓
   IP Layer
```

## Transport Comparison

| Feature | DDS Transport | TCP Transport |
|---------|---------------|---------------|
| **ROS2 Compatibility** | ✅ Full | ❌ None |
| **Domain Isolation** | ✅ 0-232 domains | ❌ No isolation |
| **Discovery** | ✅ Automatic | ❌ Manual |
| **QoS Policies** | ✅ Full support | ❌ Basic |
| **Multi-robot** | ✅ Excellent | ❌ Limited |
| **Simplicity** | ⚠️ Complex | ✅ Very simple |
| **Performance** | ⚠️ Good | ✅ Excellent |
| **Network Protocol** | UDP multicast | TCP point-to-point |

## Domain Isolation (DDS Only)

Domains provide network-level isolation between robot systems:

```rust
// Robot A uses domain 0
let robot_a = Context::with_domain_id(0)?;

// Robot B uses domain 1 (completely isolated)
let robot_b = Context::with_domain_id(1)?;

// They cannot communicate with each other
```

### Domain ID Ranges
- **0-232**: Valid ROS2 domain IDs
- **Default**: Domain 0
- **Recommendation**: Use different domains for different robot fleets

## Choosing Your Transport

### Use DDS Transport When:
- ✅ You need ROS2 ecosystem compatibility
- ✅ Running multiple robots that need isolation
- ✅ Production robotics systems
- ✅ Need advanced QoS policies
- ✅ Working with existing ROS2 tools

### Use TCP Transport When:
- ✅ Simple development and testing
- ✅ Single robot systems
- ✅ Learning miniROS concepts
- ✅ Maximum performance (local only)
- ✅ Minimal dependencies

## Configuration Examples

### Feature Flags in Cargo.toml
```toml
[dependencies]
mini-ros = { version = "0.1.2", features = ["dds-transport"] }
# OR
mini-ros = { version = "0.1.2", features = ["tcp-transport"] }
```

### Runtime Selection
```rust
#[cfg(feature = "dds-transport")]
async fn setup_transport() -> Result<Context> {
    let domain_id = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    
    Context::with_domain_id(domain_id)
}

#[cfg(feature = "tcp-transport")]
async fn setup_transport() -> Result<Context> {
    Context::new()  // Simple TCP transport
}
```

### Environment Variables
```bash
# DDS transport with custom domain
export ROS_DOMAIN_ID=42
cargo run --features dds-transport

# TCP transport (default)
cargo run --features tcp-transport
```

## Best Practices

### For Development
1. Start with TCP transport for simplicity
2. Switch to DDS when you need ROS2 features
3. Use domain 0 for initial testing

### For Production
1. Always use DDS transport for ROS2 compatibility
2. Assign unique domain IDs to different robot fleets
3. Document your domain allocation strategy

### For Multi-Robot Systems
1. Use DDS transport exclusively
2. Plan domain allocation (0-232 range)
3. Test domain isolation thoroughly

## Performance Considerations

### DDS Transport
- **Latency**: ~100-200μs (includes DDS overhead)
- **Throughput**: High (UDP multicast)
- **CPU**: Moderate (DDS processing)
- **Memory**: Higher (DDS buffers)

### TCP Transport  
- **Latency**: ~50μs (direct TCP)
- **Throughput**: Very high (no DDS overhead)
- **CPU**: Low (minimal processing)
- **Memory**: Lower (simple buffers)

## Migration Path

If you start with TCP and later need DDS:

```rust
// Before: TCP transport
let mut node = Node::new("robot")?;

// After: DDS transport (minimal changes)
let context = Context::with_domain_id(42)?;
let mut node = Node::with_context("robot", context)?;
```

The API remains the same - only the transport layer changes.

---

*Choose the right transport for your needs: TCP for simplicity, DDS for ROS2 compatibility.* 