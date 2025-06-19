//! Example 04: Basic Visualization with Rerun
//!
//! This example demonstrates:
//! - Starting Rerun GUI viewer automatically
//! - Basic data visualization
//! - Scalar data logging
//! - Text logging
//!
//! Prerequisites: Install Rerun viewer with `cargo install rerun-cli`
//! Run with: cargo run --example 04_visualization_basic

use mini_ros::visualization::{VisualizationClient, VisualizationConfig};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 04: Basic Visualization ===");

    // Create visualization client with GUI enabled
    info!("🖥️  Starting Rerun viewer...");
    let config = VisualizationConfig {
        application_id: "miniROS_Basic_Visualization".to_string(),
        spawn_viewer: true, // This will launch the Rerun GUI!
    };

    let viz_client = VisualizationClient::new(config)?;
    info!("✅ Rerun viewer started! Check the GUI window.");

    // Give the viewer time to start up
    sleep(Duration::from_secs(2)).await;

    info!("📊 Starting data visualization...");

    // Simulate robot sensor data
    for i in 0..30 {
        let t = i as f64 * 0.2;

        // Log temperature sensor data
        let temperature = 20.0 + 5.0 * (t * 0.5).sin() + fastrand::f64() * 2.0;
        viz_client.log_scalar("sensors/temperature", temperature)?;

        // Log battery voltage
        let battery = 12.6 - (t * 0.02) + 0.1 * (t * 2.0).sin();
        viz_client.log_scalar("robot/battery_voltage", battery)?;

        // Log robot speed
        let speed = 1.5 + 0.5 * (t * 0.3).cos();
        viz_client.log_scalar("robot/speed", speed)?;

        // Log CPU usage
        let cpu_usage = 30.0 + 20.0 * (t * 0.8).sin() + fastrand::f64() * 10.0;
        viz_client.log_scalar("system/cpu_usage", cpu_usage.max(0.0).min(100.0))?;

        // Log status messages periodically
        if i % 5 == 0 {
            let status = match i {
                0 => "🚀 System initializing",
                5 => "🤖 Robot starting up",
                10 => "🔄 Sensors calibrating",
                15 => "✅ All systems operational",
                20 => "🎯 Mission in progress",
                25 => "🏁 Preparing to finish",
                _ => "📊 Data logging active",
            };
            viz_client.log_text("robot/status", status)?;
        }

        info!(
            "📈 Step {}: Temp={:.1}°C, Battery={:.2}V, Speed={:.1}m/s, CPU={:.0}%",
            i + 1,
            temperature,
            battery,
            speed,
            cpu_usage
        );

        sleep(Duration::from_millis(400)).await;
    }

    // Final status
    viz_client.log_text("robot/status", "🎉 Visualization demo completed!")?;

    info!("✅ Basic visualization completed!");
    info!("💡 Check the Rerun viewer window to see your data!");
    info!("💡 Next: Try example 05 for Zenoh transport");

    // Keep the visualization open for a bit longer
    info!("⏳ Keeping visualization open for 10 seconds...");
    sleep(Duration::from_secs(10)).await;

    Ok(())
}
