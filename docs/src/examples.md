# Examples

Learn miniROS-rs through hands-on examples. Each example builds upon the previous ones.

## Learning Path

### 📚 Basic Communication (Examples 1-3)

#### 01 - Basic Pub/Sub
```bash
cargo run --example 01_basic_pubsub
```
**Learn:** Node creation, publishing, subscribing

#### 02 - Custom Messages  
```bash
cargo run --example 02_custom_messages
```
**Learn:** Define custom data types, serialization

#### 03 - Services
```bash
cargo run --example 03_services  
```
**Learn:** Request/response communication

### 📊 Visualization (Examples 4, 6)

#### 04 - Basic Visualization
```bash
cargo run --example 04_visualization_basic
```
**Learn:** Real-time data plotting, automatic GUI startup

#### 06 - Advanced 3D Visualization
```bash
cargo run --example 06_visualization_advanced
```
**Learn:** 3D robot poses, point clouds, laser scans

### 🚀 High Performance (Example 5)

#### 05 - Zenoh Transport
```bash
cargo run --example 05_zenoh_transport
```
**Learn:** High-performance communication layer

### 🤖 Complete System (Example 7)

#### 07 - Integrated System
```bash
cargo run --example 07_integrated_system
```
**Learn:** Multi-node coordination, sensor simulation, mission control

## Code Structure

Each example follows this pattern:

```rust
//! Example N: Description
//! 
//! Learning Objectives:
//! - Objective 1
//! - Objective 2
//! 
//! Run with: cargo run --example example_name

use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Example code...
    Ok(())
}
```

## Troubleshooting

### Network Issues (macOS)
If examples 1, 2, 3, 5 fail with network errors:
```bash
sudo dscacheutil -flushcache
sudo killall -HUP mDNSResponder
```

### Visualization Not Working
Make sure Rerun CLI version matches SDK:
```bash
cargo install rerun-cli@0.20.3 --force
```

## Next Steps

After running examples:
1. Modify example code to experiment
2. Read [Core Concepts](./concepts.md) for deeper understanding  
3. Check [API Reference](./api.md) for complete documentation 