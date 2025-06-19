# Performance Optimization

miniROS-rs is designed for high-performance robotics applications. This guide covers optimization strategies and implementation best practices.

## Architecture Design

### Async Runtime Benefits

The async architecture provides several performance advantages:

```rust
// Concurrent message processing
tokio::spawn(async move {
    while let Some(msg) = subscriber.recv().await {
        process_message(msg).await;
    }
});

// Non-blocking publishers
publisher.publish(&message).await?; // Doesn't block other tasks
```

### Transport Layer Options

- **Memory Broker**: Fastest for local communication within same process
- **TCP Transport**: Reliable network communication with connection management
- **DDS Transport**: Industry-standard for distributed robotics systems
- **UDP Transport**: Low-latency multicast communication

## Message Optimization

### Efficient Message Design

```rust
// Efficient: Use fixed-size arrays when possible
#[derive(Serialize, Deserialize)]
struct RobotState {
    position: [f32; 3],    // Fixed-size arrays
    velocity: [f32; 3],
    joint_angles: Vec<f32>, // Vec only when size varies
}

// Efficient: Reuse message instances
let mut message = StringMsg::default();
loop {
    message.data = format!("Frame {}", frame_count);
    publisher.publish(&message).await?;
    frame_count += 1;
}
```

### Built-in Message Types

Use appropriate built-in types for optimal performance:

```rust
use mini_ros::message::{
    StringMsg,    // For text data
    Int32Msg,     // For integer values
    Float64Msg,   // For numeric data
    BoolMsg,      // For boolean flags
};
```

## Transport Configuration

### DDS Transport with QoS

```rust
use mini_ros::dds_transport::{QosPolicy, ReliabilityKind};

// High-frequency sensor data
let sensor_qos = QosPolicy {
    reliability: ReliabilityKind::BestEffort, // Faster, occasional loss OK
    depth: 1, // Only keep latest
    ..Default::default()
};

// Critical commands
let command_qos = QosPolicy {
    reliability: ReliabilityKind::Reliable, // Ensure delivery
    depth: 10, // Keep multiple messages
    ..Default::default()
};
```

### Domain Isolation

```rust
// Separate high-frequency data by domain
let sensor_transport = DdsTransport::new(0).await?; // Domain 0: sensors
let control_transport = DdsTransport::new(1).await?; // Domain 1: control
let ui_transport = DdsTransport::new(2).await?;     // Domain 2: UI/visualization
```

## Memory Management

### Publisher Pooling

```rust
use std::sync::Arc;
use tokio::sync::Mutex;
use std::collections::HashMap;

// Reuse publishers across tasks
struct PublisherPool {
    publishers: Arc<Mutex<HashMap<String, Publisher<StringMsg>>>>,
}

impl PublisherPool {
    async fn get_publisher(&self, topic: &str) -> Publisher<StringMsg> {
        let mut pubs = self.publishers.lock().await;
        pubs.entry(topic.to_string())
            .or_insert_with(|| create_publisher(topic))
            .clone()
    }
}
```

### Message Pooling Pattern

```rust
use crossbeam_channel::{bounded, Receiver, Sender};

// Pre-allocate message pool to reduce allocations
struct MessagePool<T> {
    pool: Receiver<T>,
    return_sender: Sender<T>,
}

impl<T: Default> MessagePool<T> {
    fn new(size: usize) -> Self {
        let (send, recv) = bounded(size);
        let (return_send, _return_recv) = bounded(size);
        
        // Pre-fill pool
        for _ in 0..size {
            let _ = send.send(T::default());
        }
        
        Self {
            pool: recv,
            return_sender: return_send,
        }
    }
}
```

## Async Task Optimization

### Dedicated Runtime Configuration

```rust
// Configure runtime for specific workloads
let rt = tokio::runtime::Builder::new_multi_thread()
    .worker_threads(4)
    .thread_name("miniROS-worker")
    .enable_all()
    .build()?;

// Use runtime handle for spawning tasks
let handle = rt.handle();
handle.spawn(async move {
    // High-priority task
});
```

### Batched Processing

```rust
// Process messages in batches for efficiency
let mut batch = Vec::with_capacity(100);

while let Ok(msg) = receiver.try_recv() {
    batch.push(msg);
    
    if batch.len() >= 100 {
        process_batch(&batch).await;
        batch.clear();
    }
}
```

## Network Optimization

### Multicast Configuration

For UDP/DDS transports on Linux:

```bash
# Optimize multicast buffer sizes
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=65536

# Monitor network usage
sudo iftop -i eth0 -f "dst net 239.255.0.0/24"
```

### Topic Design

```rust
// Good: Hierarchical topics
"/robot1/sensors/imu"
"/robot1/actuators/wheel_speeds"
"/global/map_updates"

// Avoid: Flat namespace
"/imu_data"
"/wheel_data" 
"/map_data"
```

## Monitoring and Profiling

### Built-in Benchmarking

```rust
// Use built-in benchmarking tools
use mini_ros::benchmarks::{LatencyBenchmark, ThroughputBenchmark};

// Measure publisher-subscriber latency
let benchmark = LatencyBenchmark::new();
let results = benchmark.run_pubsub_latency(1000).await?;

println!("Average latency: {}μs", results.average_latency_micros());
```

### Performance Monitoring

```rust
// Monitor system performance
use mini_ros::discovery::DiscoveryService;

let discovery = DiscoveryService::new(0)?;
let stats = discovery.get_performance_stats().await?;

println!("Active nodes: {}", stats.node_count);
println!("Message rate: {}/s", stats.message_rate);
```

## Best Practices

### 1. Choose Appropriate Transport

- **Memory broker**: Single-process applications
- **TCP**: Reliable network communication
- **DDS**: Multi-robot distributed systems
- **UDP**: Low-latency sensor streaming

### 2. Optimize Message Frequency

```rust
// High-frequency: IMU, camera frames
const HIGH_FREQ: Duration = Duration::from_millis(1);  // 1000Hz

// Medium-frequency: robot state
const MED_FREQ: Duration = Duration::from_millis(10);  // 100Hz

// Low-frequency: configuration
const LOW_FREQ: Duration = Duration::from_millis(100); // 10Hz
```

### 3. Use Appropriate QoS

- **Best effort**: High-frequency sensor data
- **Reliable**: Commands and configuration
- **Volatile**: Real-time data
- **Transient**: Configuration that new nodes need

### 4. Monitor Resource Usage

```rust
// Use tokio-console for async task monitoring
#[tokio::main]
async fn main() {
    console_subscriber::init();
    // Your application code
}
```

## Performance Testing

### Latency Measurement

```rust
use std::time::Instant;

let start = Instant::now();
publisher.publish(&message).await?;
// Measure round-trip time with subscriber callback
let latency = start.elapsed();
```

### Throughput Testing

```rust
let start = Instant::now();
let message_count = 10000;

for i in 0..message_count {
    publisher.publish(&create_message(i)).await?;
}

let duration = start.elapsed();
let throughput = message_count as f64 / duration.as_secs_f64();
println!("Throughput: {:.2} messages/second", throughput);
```

This performance guide focuses on practical optimization techniques that can be implemented with the current miniROS-rs architecture and feature set. 