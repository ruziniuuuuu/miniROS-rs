//! Minimal Performance Benchmarks Demo
//!
//! This example demonstrates basic performance measurement in MiniROS.

use mini_ros::message::StringMsg;
use mini_ros::{Context, Node};
use std::time::{Duration, Instant};
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== MiniROS Minimal Performance Demo ===");

    // Create context and initialize
    let context = Context::with_domain_id(301)?;
    context.init().await?;

    // 1. Basic Latency Test
    println!("\n1. Latency Test:");
    let latency_result = run_latency_test().await?;
    println!("Average latency: {:.2} μs", latency_result);

    // 2. Throughput Test
    println!("\n2. Throughput Test:");
    let throughput_result = run_throughput_test().await?;
    println!("Throughput: {:.0} messages/sec", throughput_result);

    // 3. Memory Usage Test
    println!("\n3. Memory Usage:");
    let memory_usage = get_memory_usage();
    println!("Memory usage: {:.1} MB", memory_usage);

    // Cleanup
    context.shutdown().await?;
    println!("\nBenchmark demo completed successfully!");

    Ok(())
}

/// Run a simple latency test
async fn run_latency_test() -> Result<f64, Box<dyn std::error::Error>> {
    let context = Context::with_domain_id(302)?;
    context.init().await?;

    let mut node = Node::new("latency_test")?;
    let publisher = node.create_publisher::<StringMsg>("test/latency").await?;

    let mut latencies = Vec::new();
    let test_messages = 10;

    // Measure publish latency
    for i in 0..test_messages {
        let start = Instant::now();
        let msg = StringMsg {
            data: format!("Test message {}", i),
        };
        publisher.publish(&msg).await?;
        let latency = start.elapsed().as_micros() as f64;
        latencies.push(latency);

        sleep(Duration::from_millis(100)).await;
    }

    context.shutdown().await?;

    // Calculate average latency
    let avg_latency = latencies.iter().sum::<f64>() / latencies.len() as f64;
    Ok(avg_latency)
}

/// Run a simple throughput test
async fn run_throughput_test() -> Result<f64, Box<dyn std::error::Error>> {
    let context = Context::with_domain_id(303)?;
    context.init().await?;

    let mut node = Node::new("throughput_test")?;
    let publisher = node
        .create_publisher::<StringMsg>("test/throughput")
        .await?;

    let start_time = Instant::now();
    let test_duration = Duration::from_secs(2);
    let mut messages_sent = 0;

    // Send messages for test duration
    let end_time = start_time + test_duration;
    while Instant::now() < end_time {
        let msg = StringMsg {
            data: format!("Message {}", messages_sent),
        };
        publisher.publish(&msg).await?;
        messages_sent += 1;

        // Small delay to prevent overwhelming
        tokio::task::yield_now().await;
    }

    let actual_duration = start_time.elapsed();
    context.shutdown().await?;

    // Calculate throughput
    let throughput = messages_sent as f64 / actual_duration.as_secs_f64();
    Ok(throughput)
}

/// Get approximate memory usage (simplified)
fn get_memory_usage() -> f64 {
    // Simplified memory usage estimation
    match std::fs::read_to_string("/proc/self/status") {
        Ok(content) => {
            for line in content.lines() {
                if line.starts_with("VmRSS:") {
                    if let Some(kb_str) = line.split_whitespace().nth(1) {
                        if let Ok(kb) = kb_str.parse::<f64>() {
                            return kb / 1024.0; // Convert KB to MB
                        }
                    }
                }
            }
        }
        Err(_) => {
            // Fallback for non-Linux systems
            return 10.0; // Approximate estimate
        }
    }
    10.0 // Default estimate
}
