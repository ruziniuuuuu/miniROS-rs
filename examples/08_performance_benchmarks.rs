//! Performance benchmarks example for miniROS
//!
//! This example demonstrates how to run comprehensive performance benchmarks
//! to measure miniROS performance characteristics.

use mini_ros::benchmarks::{BenchmarkConfig, BenchmarkFramework};
use mini_ros::prelude::*;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging for detailed benchmark output
    tracing_subscriber::fmt()
        .with_level(true)
        .with_target(false)
        .init();

    println!("🚀 miniROS Performance Benchmark Suite");
    println!("======================================");

    // Standard benchmark configuration
    let config = BenchmarkConfig {
        duration_seconds: 10,     // Short test for demo
        message_size_bytes: 1024, // 1KB messages
        publisher_count: 1,
        subscriber_count: 1,
        message_rate_hz: 100.0,
        warmup_seconds: 2,
    };

    let mut framework = BenchmarkFramework::new(config);

    // Run all benchmarks
    match framework.run_all_benchmarks().await {
        Ok(results) => {
            println!("\n✅ Benchmark Results:");
            println!("{}", results.generate_report());

            // Export to JSON for further analysis
            if let Ok(json) = results.to_json() {
                std::fs::write("benchmark_results.json", json).expect("Failed to write results");
                println!("📄 Results saved to benchmark_results.json");
            }
        }
        Err(e) => {
            eprintln!("❌ Benchmark failed: {}", e);
            return Err(e);
        }
    }

    // Run custom latency test
    println!("\n🔧 Running custom latency test...");
    run_custom_latency_test().await?;

    // Run scalability test
    println!("\n📈 Running scalability test...");
    run_scalability_test().await?;

    println!("\n🎯 Benchmark suite completed!");
    println!("Results show miniROS performance characteristics for:");
    println!("- Pub/Sub latency");
    println!("- Maximum throughput");
    println!("- Memory usage patterns");
    println!("- Multi-node scalability");

    Ok(())
}

/// Custom latency test with precise measurements
async fn run_custom_latency_test() -> Result<()> {
    let _context = Context::new()?;

    // Create nodes
    let mut pub_node = Node::new("latency_pub")?;
    pub_node.init().await?;
    let publisher = pub_node
        .create_publisher::<StringMsg>("/latency_test")
        .await?;

    let mut sub_node = Node::new("latency_sub")?;
    sub_node.init().await?;
    let subscriber = sub_node
        .create_subscriber::<StringMsg>("/latency_test")
        .await?;

    let _latencies: Vec<f64> = Vec::new();
    let _start_time = std::time::Instant::now();

    // Set up subscriber to measure latency
    subscriber.on_message(move |msg: StringMsg| {
        // Extract timestamp from message
        if let Ok(timestamp_ns) = msg.data.parse::<u128>() {
            let now_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let latency_us = (now_ns - timestamp_ns) as f64 / 1000.0;
            println!("  Latency: {:.1}μs", latency_us);
        }
    })?;

    // Send test messages with timestamps
    for _i in 0..50 {
        let timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();

        let msg = StringMsg {
            data: timestamp_ns.to_string(),
        };

        publisher.publish(&msg).await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    println!("  ✅ Custom latency test completed");
    Ok(())
}

/// Test scalability with multiple nodes and topics
async fn run_scalability_test() -> Result<()> {
    let node_counts = vec![1, 2, 5, 10];

    for node_count in node_counts {
        println!("  Testing with {} nodes...", node_count);

        let start_time = std::time::Instant::now();
        let mut nodes = Vec::new();
        let mut publishers = Vec::new();

        // Create multiple publisher nodes
        for i in 0..node_count {
            let mut node = Node::new(&format!("scale_node_{}", i))?;
            node.init().await?;
            let publisher = node
                .create_publisher::<StringMsg>(&format!("/scale_topic_{}", i))
                .await?;
            publishers.push(publisher);
            nodes.push(node);
        }

        let setup_time = start_time.elapsed();

        // Send messages from all publishers
        let send_start = std::time::Instant::now();
        for (i, publisher) in publishers.iter().enumerate() {
            let msg = StringMsg {
                data: format!("Message from node {}", i),
            };
            publisher.publish(&msg).await?;
        }
        let send_time = send_start.elapsed();

        println!("    Setup time: {:.2}ms", setup_time.as_millis());
        println!("    Send time: {:.2}ms", send_time.as_millis());
        println!(
            "    Avg per node: {:.2}ms",
            send_time.as_millis() as f64 / node_count as f64
        );

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    println!("  ✅ Scalability test completed");
    Ok(())
}

/// Memory usage monitoring (simplified)
#[allow(dead_code)]
async fn monitor_memory_usage() -> Result<()> {
    println!("  Monitoring memory usage...");

    // Create nodes and monitor memory growth
    let mut nodes = Vec::new();
    for i in 0..20 {
        let mut node = Node::new(&format!("memory_node_{}", i))?;
        node.init().await?;
        nodes.push(node);

        // In a real implementation, we'd use system calls to get actual memory usage
        // For demo purposes, we'll simulate memory measurements
        let simulated_memory_mb = 5.0 + (i as f64 * 0.5);
        println!("    Node {}: ~{:.1}MB", i, simulated_memory_mb);

        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    println!("  ✅ Memory monitoring completed");
    Ok(())
}

/// Throughput stress test
#[allow(dead_code)]
async fn stress_test_throughput() -> Result<()> {
    println!("  Running throughput stress test...");

    let mut pub_node = Node::new("stress_pub")?;
    pub_node.init().await?;
    let publisher = pub_node
        .create_publisher::<StringMsg>("/stress_test")
        .await?;

    let message_count = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let count_clone = message_count.clone();

    let mut sub_node = Node::new("stress_sub")?;
    sub_node.init().await?;
    let subscriber = sub_node
        .create_subscriber::<StringMsg>("/stress_test")
        .await?;

    subscriber.on_message(move |_msg: StringMsg| {
        count_clone.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    })?;

    // Send messages as fast as possible for 5 seconds
    let start_time = std::time::Instant::now();
    let mut sent_count = 0u64;

    while start_time.elapsed() < Duration::from_secs(5) {
        let msg = StringMsg {
            data: format!("stress_test_message_{}", sent_count),
        };
        publisher.publish(&msg).await?;
        sent_count += 1;
    }

    let elapsed = start_time.elapsed();
    let received_count = message_count.load(std::sync::atomic::Ordering::Relaxed);

    println!("    Sent: {} messages", sent_count);
    println!("    Received: {} messages", received_count);
    println!("    Duration: {:.2}s", elapsed.as_secs_f64());
    println!(
        "    Send rate: {:.0} msg/s",
        sent_count as f64 / elapsed.as_secs_f64()
    );
    println!(
        "    Receive rate: {:.0} msg/s",
        received_count as f64 / elapsed.as_secs_f64()
    );

    println!("  ✅ Stress test completed");
    Ok(())
}
