# Performance Optimization

MiniROS is designed for high-performance robotics applications. This guide covers optimization strategies and best practices.

## Architecture Performance

### DDS Transport Advantages

- **Zero-copy serialization**: Direct memory access for message data
- **Multicast efficiency**: Single message reaches multiple subscribers
- **QoS-based optimization**: Choose appropriate reliability/durability
- **Domain isolation**: Separate network traffic by robot systems

### Async Runtime Benefits

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

## Message Optimization

### Choose Appropriate Message Types

```rust
// Efficient for small data
use mini_ros::message::StringMsg;

// Efficient for numeric data
use mini_ros::message::Float64Msg;

// Custom messages for complex data
#[derive(Serialize, Deserialize)]
struct OptimizedRobotState {
    position: [f32; 3],    // Fixed-size arrays are faster
    velocity: [f32; 3],
    joint_angles: Vec<f32>, // Use Vec only when size varies
}
```

### Minimize Serialization Overhead

```rust
// Good: Reuse message instances
let mut message = StringMsg::default();
loop {
    message.data = format!("Frame {}", frame_count);
    publisher.publish(&message).await?;
    frame_count += 1;
}

// Avoid: Creating new messages each time
loop {
    let message = StringMsg { 
        data: format!("Frame {}", frame_count) 
    };
    publisher.publish(&message).await?; // More allocation overhead
}
```

## QoS Optimization

### Choose Appropriate Reliability

```rust
use mini_ros::dds_transport::{QosPolicy, ReliabilityKind};

// High-frequency sensor data (e.g., IMU at 1000Hz)
let sensor_qos = QosPolicy {
    reliability: ReliabilityKind::BestEffort, // Faster, occasional loss OK
    depth: 1, // Only keep latest
    ..Default::default()
};

// Critical commands (e.g., emergency stop)
let command_qos = QosPolicy {
    reliability: ReliabilityKind::Reliable, // Ensure delivery
    depth: 10, // Keep multiple messages
    ..Default::default()
};
```

### Optimize History Settings

```rust
// Real-time data (latest value only)
let realtime_qos = QosPolicy {
    history: HistoryKind::KeepLast,
    depth: 1, // Minimal memory usage
    ..Default::default()
};

// Batch processing (keep all messages)
let batch_qos = QosPolicy {
    history: HistoryKind::KeepAll,
    depth: 1000, // Higher memory usage
    ..Default::default()
};
```

## Network Optimization

### Domain Strategy

```rust
// Separate high-frequency data
let sensor_transport = DdsTransport::new(0).await?; // Domain 0: sensors
let control_transport = DdsTransport::new(1).await?; // Domain 1: control
let ui_transport = DdsTransport::new(2).await?;     // Domain 2: UI/visualization
```

### Multicast Configuration

```bash
# Linux: Optimize multicast buffer sizes
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=65536

# Monitor network usage
sudo iftop -i eth0 -f "dst net 239.255.0.0/24"
```

## Memory Optimization

### Publisher Pool Pattern

```rust
use std::sync::Arc;
use tokio::sync::Mutex;

// Reuse publishers across threads
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

### Message Pooling

```rust
use crossbeam_channel::{bounded, Receiver, Sender};

// Pre-allocate message pool
struct MessagePool<T> {
    pool: Receiver<T>,
    return_sender: Sender<T>,
}

impl<T: Default> MessagePool<T> {
    fn new(size: usize) -> Self {
        let (send, recv) = bounded(size);
        let (return_send, return_recv) = bounded(size);
        
        // Pre-fill pool
        for _ in 0..size {
            send.send(T::default()).unwrap();
        }
        
        Self {
            pool: recv,
            return_sender: return_send,
        }
    }
    
    fn get(&self) -> Option<T> {
        self.pool.try_recv().ok()
    }
    
    fn return_message(&self, msg: T) {
        let _ = self.return_sender.try_send(msg);
    }
}
```

## CPU Optimization

### Async Task Management

```rust
// Dedicated executor for high-priority tasks
let rt = tokio::runtime::Builder::new_multi_thread()
    .worker_threads(4)
    .thread_name("miniROS-worker")
    .build()?;

// Spawn critical tasks on dedicated threads
rt.spawn(async move {
    // High-priority control loop
    while let Some(sensor_data) = sensor_subscriber.recv().await {
        let control_output = control_algorithm(sensor_data);
        control_publisher.publish(&control_output).await?;
    }
});
```

### CPU Affinity (Linux)

```rust
use core_affinity;

// Pin critical threads to specific CPU cores
let core_ids = core_affinity::get_core_ids().unwrap();
if let Some(core_id) = core_ids.get(0) {
    core_affinity::set_for_current(*core_id);
}
```

## Benchmarking

### Built-in Metrics

```rust
use std::time::Instant;

let start = Instant::now();
publisher.publish(&message).await?;
let publish_latency = start.elapsed();

tracing::info!("Publish latency: {:?}", publish_latency);
```

### Performance Testing

```rust
#[tokio::test]
async fn benchmark_publisher_throughput() {
    let transport = DdsTransport::new(99).await.unwrap();
    let publisher = transport.create_publisher::<StringMsg>("benchmark").await.unwrap();
    
    let message = StringMsg { data: "benchmark".to_string() };
    let start = Instant::now();
    let count = 10_000;
    
    for _ in 0..count {
        publisher.publish(&message).await.unwrap();
    }
    
    let elapsed = start.elapsed();
    let throughput = count as f64 / elapsed.as_secs_f64();
    
    println!("Throughput: {:.2} messages/sec", throughput);
    assert!(throughput > 1000.0); // Expect > 1000 msg/s
}
```

## Real-world Performance Tips

### Robot Control Loop

```rust
// Optimized control loop structure
async fn control_loop() -> Result<()> {
    let mut interval = tokio::time::interval(Duration::from_millis(10)); // 100Hz
    
    loop {
        interval.tick().await;
        
        // Non-blocking sensor reading
        if let Some(sensor_data) = sensor_subscriber.try_recv() {
            let control_cmd = compute_control(sensor_data);
            
            // Non-blocking command publishing
            if let Err(e) = control_publisher.try_publish(&control_cmd) {
                tracing::warn!("Control publish failed: {}", e);
            }
        }
    }
}
```

### Image Processing Pipeline

```rust
// Efficient image processing
async fn image_pipeline() -> Result<()> {
    let (tx, mut rx) = mpsc::channel(10); // Bounded channel
    
    // Producer task
    tokio::spawn(async move {
        while let Some(raw_image) = camera_subscriber.recv().await {
            if tx.send(raw_image).await.is_err() {
                break; // Consumer dropped
            }
        }
    });
    
    // Consumer task (processing)
    while let Some(image) = rx.recv().await {
        let processed = tokio::task::spawn_blocking(move || {
            // CPU-intensive processing on thread pool
            process_image(image)
        }).await?;
        
        processed_publisher.publish(&processed).await?;
    }
    
    Ok(())
}
```

## Monitoring Performance

### Enable Tracing

```rust
use tracing_subscriber;

// Initialize structured logging
tracing_subscriber::fmt()
    .with_max_level(tracing::Level::INFO)
    .with_target(false)
    .with_thread_ids(true)
    .init();
```

### Custom Metrics

```rust
use std::sync::atomic::{AtomicU64, Ordering};

static MESSAGE_COUNT: AtomicU64 = AtomicU64::new(0);
static TOTAL_LATENCY: AtomicU64 = AtomicU64::new(0);

// In message handler
MESSAGE_COUNT.fetch_add(1, Ordering::Relaxed);
TOTAL_LATENCY.fetch_add(latency_ns, Ordering::Relaxed);

// Periodic reporting
tokio::spawn(async move {
    let mut interval = tokio::time::interval(Duration::from_secs(1));
    loop {
        interval.tick().await;
        let count = MESSAGE_COUNT.swap(0, Ordering::Relaxed);
        let total_lat = TOTAL_LATENCY.swap(0, Ordering::Relaxed);
        
        if count > 0 {
            let avg_latency = total_lat / count;
            tracing::info!("Messages/sec: {}, Avg latency: {}ns", count, avg_latency);
        }
    }
});
```

## Performance Targets

| Metric | Target | Typical |
|--------|--------|---------|
| Message Latency | < 1ms | 0.1-0.5ms |
| Throughput | > 10k msg/s | 50k+ msg/s |
| Memory Usage | < 50MB | 10-20MB |
| CPU Usage | < 50% | 10-30% |
| Network Bandwidth | < 100Mbps | 1-10Mbps |

These targets are achievable with proper optimization and appropriate hardware. 