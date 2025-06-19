//! Example 04: Parameters System
//!
//! This example demonstrates the Parameter system in miniROS.
//! Parameters manage runtime configuration for nodes and services.
//!
//! Run with: cargo run --example 04_actions_parameters

use mini_ros::{
    parameter::{ParameterClient, ParameterServer, ParameterValue},
    prelude::*,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 04: Parameter System ===");

    // Initialize context
    let context = Context::new()?;
    context.init().await?;

    // === Parameter System Demo ===
    info!("📋 Setting up Parameter System...");

    let param_server = ParameterServer::new();

    // Set robot configuration parameters
    param_server.set_parameter(
        "robot.name",
        ParameterValue::String("miniROS_Robot".to_string()),
    )?;
    param_server.set_parameter("robot.max_speed", ParameterValue::Float(2.5))?;
    param_server.set_parameter("robot.active", ParameterValue::Bool(true))?;
    param_server.set_parameter(
        "robot.sensors.enabled",
        ParameterValue::BoolArray(vec![true, true, false]),
    )?;
    param_server.set_parameter(
        "robot.waypoints",
        ParameterValue::StringArray(vec![
            "kitchen".to_string(),
            "living_room".to_string(),
            "bedroom".to_string(),
        ]),
    )?;

    let param_client = ParameterClient::from_server(&param_server);

    info!("✅ Parameter system initialized");

    // === Read Parameters ===
    info!("\n🔍 Reading Configuration Parameters:");

    if let Some(name) = param_client.get_parameter("robot.name")? {
        if let ParameterValue::String(robot_name) = name {
            info!("🤖 Robot Name: {}", robot_name);
        }
    }

    if let Some(speed) = param_client.get_parameter("robot.max_speed")? {
        if let ParameterValue::Float(max_speed) = speed {
            info!("⚡ Max Speed: {:.1} m/s", max_speed);
        }
    }

    if let Some(active) = param_client.get_parameter("robot.active")? {
        if let ParameterValue::Bool(is_active) = active {
            info!("🔋 Robot Active: {}", is_active);
        }
    }

    if let Some(sensors) = param_client.get_parameter("robot.sensors.enabled")? {
        if let ParameterValue::BoolArray(sensor_states) = sensors {
            info!("📡 Sensors: {:?}", sensor_states);
        }
    }

    if let Some(waypoints) = param_client.get_parameter("robot.waypoints")? {
        if let ParameterValue::StringArray(points) = waypoints {
            info!("🗺️  Waypoints: {:?}", points);
        }
    }

    // === Parameter Updates Demo ===
    info!("\n🔧 Testing Dynamic Parameter Updates...");

    // Simulate runtime parameter changes
    sleep(Duration::from_millis(500)).await;

    info!("Updating robot configuration...");
    param_server.set_parameter("robot.max_speed", ParameterValue::Float(1.5))?;
    param_server.set_parameter("robot.active", ParameterValue::Bool(false))?;
    param_server.set_parameter(
        "robot.current_task",
        ParameterValue::String("charging".to_string()),
    )?;

    // Read updated values
    info!("\n📊 Updated Configuration:");
    if let Some(speed) = param_client.get_parameter("robot.max_speed")? {
        if let ParameterValue::Float(new_speed) = speed {
            info!("🔄 New Max Speed: {:.1} m/s", new_speed);
        }
    }

    if let Some(active) = param_client.get_parameter("robot.active")? {
        if let ParameterValue::Bool(is_active) = active {
            info!(
                "🔄 Robot State: {}",
                if is_active { "Active" } else { "Standby" }
            );
        }
    }

    if let Some(task) = param_client.get_parameter("robot.current_task")? {
        if let ParameterValue::String(current_task) = task {
            info!("🔄 Current Task: {}", current_task);
        }
    }

    // === Parameter Management ===
    info!("\n📋 Parameter Management:");

    // List all parameters
    let all_params = param_client.list_parameters()?;
    info!("Total parameters: {}", all_params.len());

    for (key, value) in &all_params {
        match value {
            ParameterValue::String(s) => info!("  {} = '{}' (string)", key, s),
            ParameterValue::Float(f) => info!("  {} = {:.2} (float)", key, f),
            ParameterValue::Bool(b) => info!("  {} = {} (bool)", key, b),
            ParameterValue::Int(i) => info!("  {} = {} (int)", key, i),
            ParameterValue::StringArray(arr) => info!("  {} = {:?} (string[])", key, arr),
            ParameterValue::BoolArray(arr) => info!("  {} = {:?} (bool[])", key, arr),
            ParameterValue::IntArray(arr) => info!("  {} = {:?} (int[])", key, arr),
            ParameterValue::FloatArray(arr) => info!("  {} = {:?} (float[])", key, arr),
        }
    }

    // === Parameter Operations ===
    info!("\n🛠️  Testing Parameter Operations:");

    // Check parameter existence
    assert!(param_client.has_parameter("robot.name")?);
    info!("✅ Parameter 'robot.name' exists");

    assert!(!param_client.has_parameter("robot.non_existent")?);
    info!("✅ Parameter 'robot.non_existent' does not exist");

    // Test parameter deletion
    param_server.set_parameter("robot.temp_param", ParameterValue::Int(42))?;
    assert!(param_client.has_parameter("robot.temp_param")?);

    param_server.delete_parameter("robot.temp_param")?;
    assert!(!param_client.has_parameter("robot.temp_param")?);
    info!("✅ Parameter deletion works correctly");

    // === Nested Parameter Structure ===
    info!("\n🌲 Testing Nested Parameter Structure:");

    // Set hierarchical parameters
    param_server.set_parameter("sensors.lidar.range", ParameterValue::Float(30.0))?;
    param_server.set_parameter("sensors.lidar.enabled", ParameterValue::Bool(true))?;
    param_server.set_parameter(
        "sensors.camera.resolution",
        ParameterValue::StringArray(vec!["1920x1080".to_string(), "640x480".to_string()]),
    )?;
    param_server.set_parameter(
        "navigation.planner.algorithm",
        ParameterValue::String("A*".to_string()),
    )?;
    param_server.set_parameter(
        "navigation.controller.gains",
        ParameterValue::FloatArray(vec![1.0, 0.5, 0.1]),
    )?;

    // Display hierarchical structure
    let all_params = param_client.list_parameters()?;
    let mut sensor_params = 0;
    let mut nav_params = 0;
    let mut robot_params = 0;

    for key in all_params.keys() {
        if key.starts_with("sensors.") {
            sensor_params += 1;
        } else if key.starts_with("navigation.") {
            nav_params += 1;
        } else if key.starts_with("robot.") {
            robot_params += 1;
        }
    }

    info!("📊 Parameter Categories:");
    info!("  🤖 Robot: {} parameters", robot_params);
    info!("  📡 Sensors: {} parameters", sensor_params);
    info!("  🗺️  Navigation: {} parameters", nav_params);

    // === Performance Test ===
    info!("\n⚡ Performance Test:");

    let start = std::time::Instant::now();

    // Set many parameters quickly
    for i in 0..1000 {
        param_server.set_parameter(&format!("test.param_{}", i), ParameterValue::Int(i as i64))?;
    }

    let set_duration = start.elapsed();
    info!("Set 1000 parameters in: {:?}", set_duration);

    let start = std::time::Instant::now();

    // Read many parameters quickly
    for i in 0..1000 {
        let _ = param_client.get_parameter(&format!("test.param_{}", i))?;
    }

    let get_duration = start.elapsed();
    info!("Read 1000 parameters in: {:?}", get_duration);

    // Cleanup test parameters
    for i in 0..1000 {
        param_server.delete_parameter(&format!("test.param_{}", i))?;
    }

    info!("\n✅ Parameter System example completed!");
    info!("💡 This demonstrated comprehensive parameter management");
    info!("🔧 Features shown:");
    info!("   • Basic parameter operations (set/get/delete)");
    info!("   • Multiple data types (string, int, float, bool, arrays)");
    info!("   • Hierarchical parameter organization");
    info!("   • Dynamic parameter updates");
    info!("   • Parameter existence checking");
    info!("   • Performance characteristics");

    Ok(())
}
