# Visualization Examples

This section provides complete examples demonstrating different aspects of miniROS-rs visualization capabilities.

## Example 1: Basic Visualization Demo

This example demonstrates various data types and visualization patterns.

```rust
//! Basic visualization demo showing different data types
//! Run with: cargo run --example visualization_demo

use mini_ros::visualization::{
    VisualizationClient, VisualizationConfig, 
    RobotPose, PointCloud, LaserScan, Visualizable
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    info!("Starting visualization demo");

    // Create visualization client (buffered mode by default)
    let config = VisualizationConfig {
        application_id: "miniROS_demo".to_string(),
        spawn_viewer: false,  // Use buffered mode
    };
    
    let viz_client = VisualizationClient::new(config)?;
    info!("Visualization client created");

    // Simulate robot moving in a circular trajectory
    info!("Visualizing robot trajectory...");
    for i in 0..100 {
        let t = i as f32 * 0.1;
        
        // 1. Robot pose (circular trajectory)
        let robot_pose = RobotPose {
            position: [
                2.0 * (t * 0.5).cos(),
                2.0 * (t * 0.5).sin(), 
                0.0
            ],
            orientation: [0.0, 0.0, (t * 0.5).sin(), (t * 0.5).cos()],
        };
        robot_pose.visualize(&viz_client, "robot/pose")?;

        // 2. Simulated laser scan data
        let mut ranges = Vec::new();
        for j in 0..360 {
            let angle = (j as f32).to_radians();
            let noise = 0.1 * ((t + angle) * 10.0).sin();
            let range = 3.0 + noise;
            ranges.push(range);
        }
        
        let laser_scan = LaserScan {
            ranges,
            angle_min: -std::f32::consts::PI,
            angle_max: std::f32::consts::PI,
        };
        laser_scan.visualize(&viz_client, "robot/laser_scan")?;

        // 3. Random point cloud representing obstacles
        let mut points = Vec::new();
        for _ in 0..100 {
            points.push([
                4.0 * (t + fastrand::f32()).cos(),
                4.0 * (t + fastrand::f32()).sin(),
                fastrand::f32() * 2.0 - 1.0,
            ]);
        }
        let point_cloud = PointCloud { points };
        point_cloud.visualize(&viz_client, "environment/obstacles")?;

        // 4. Scalar sensor data
        viz_client.log_scalar("sensors/velocity", (t * 0.5).sin() as f64 + 1.0)?;
        viz_client.log_scalar("sensors/temperature", 20.0 + 5.0 * (t * 0.1).sin() as f64)?;

        // 5. Status messages
        if i % 10 == 0 {
            viz_client.log_text("robot/status", &format!("Step {}, time: {:.1}s", i, t))?;
        }

        sleep(Duration::from_millis(50)).await;
    }

    info!("Visualization demo completed");
    Ok(())
}
```

**Key Features Demonstrated:**
- Robot trajectory visualization with poses
- Laser scan data as 2D points
- Dynamic point clouds
- Scalar sensor readings
- Status text logging

## Example 2: Integrated Communication and Visualization

This example combines Zenoh communication with Rerun visualization.

```rust
//! Integrated demo combining Zenoh communication and Rerun visualization
//! Run with: cargo run --example integrated_demo

use mini_ros::zenoh_transport::{ZenohConfig, ZenohTransport};
use mini_ros::visualization::{VisualizationClient, VisualizationConfig, RobotPose, Visualizable};
use mini_ros::message::{StringMsg, Float64Msg};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    tracing_subscriber::fmt::init();
    info!("Starting integrated Zenoh + Rerun demo");

    // Setup communication transport
    let zenoh_config = ZenohConfig::default();
    let transport = ZenohTransport::new(zenoh_config).await?;
    
    // Setup visualization
    let viz_config = VisualizationConfig::default();
    let viz_client = VisualizationClient::new(viz_config)?;

    // Create communication channels
    let pose_topic = "/robot/pose";
    let status_topic = "/robot/status";
    let sensor_topic = "/robot/sensor_data";
    
    let pose_pub = transport.create_publisher::<RobotPose>(pose_topic).await?;
    let status_pub = transport.create_publisher::<StringMsg>(status_topic).await?;
    let sensor_pub = transport.create_publisher::<Float64Msg>(sensor_topic).await?;

    // Create subscribers with visualization callbacks
    let pose_sub = transport.create_subscriber::<RobotPose>(pose_topic).await?;
    let status_sub = transport.create_subscriber::<StringMsg>(status_topic).await?;
    let sensor_sub = transport.create_subscriber::<Float64Msg>(sensor_topic).await?;

    // Setup automatic visualization for received data
    let viz_client_clone = viz_client.clone();
    pose_sub.on_message(move |pose: RobotPose| {
        info!("Received pose: {:?}", pose);
        if let Err(e) = pose.visualize(&viz_client_clone, "robot/pose") {
            tracing::warn!("Failed to visualize pose: {}", e);
        }
    })?;

    let viz_client_clone = viz_client.clone();
    status_sub.on_message(move |msg: StringMsg| {
        info!("Status update: {}", msg.data);
        if let Err(e) = viz_client_clone.log_text("robot/status", &msg.data) {
            tracing::warn!("Failed to log status: {}", e);
        }
    })?;

    let viz_client_clone = viz_client.clone();
    sensor_sub.on_message(move |msg: Float64Msg| {
        info!("Sensor data: {}", msg.data);
        if let Err(e) = viz_client_clone.log_scalar("sensors/temperature", msg.data) {
            tracing::warn!("Failed to log sensor data: {}", e);
        }
    })?;

    // Simulate robot operation
    info!("Running integrated demo...");
    for i in 0..50 {
        let t = i as f32 * 0.2;
        
        // Publish robot pose
        let pose = RobotPose {
            position: [t.cos() * 3.0, t.sin() * 3.0, 0.1 * t],
            orientation: [0.0, 0.0, (t * 0.5).sin(), (t * 0.5).cos()],
        };
        pose_pub.publish(&pose).await?;

        // Publish status updates
        if i % 5 == 0 {
            let status = StringMsg {
                data: format!("Robot operational - Step {}", i),
            };
            status_pub.publish(&status).await?;
        }

        // Publish sensor data
        let sensor_data = Float64Msg {
            data: 25.0 + 3.0 * (t as f64 * 0.3).sin() + fastrand::f64() * 0.5,
        };
        sensor_pub.publish(&sensor_data).await?;

        sleep(Duration::from_millis(200)).await;
    }

    // Cleanup
    transport.shutdown().await?;
    info!("Integrated demo completed");
    Ok(())
}
```

**Key Features Demonstrated:**
- Integration of communication and visualization
- Automatic visualization of received messages
- Real-time data flow from publishers to visualization
- Multiple data types in one application

## Example 3: Custom Data Type Visualization

This example shows how to create and visualize custom data types.

```rust
use mini_ros::visualization::{VisualizationClient, VisualizationConfig, Visualizable};
use mini_ros::error::Result;
use serde::{Serialize, Deserialize};

// Custom robot state data type
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotState {
    position: [f32; 3],
    velocity: [f32; 3],
    battery_voltage: f64,
    system_status: String,
    sensor_readings: Vec<f32>,
}

// Implement visualization for custom type
impl Visualizable for RobotState {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Visualize position as 3D transform
        client.log_transform_3d(
            &format!("{}/pose", entity_path),
            self.position,
            [0.0, 0.0, 0.0, 1.0] // Identity quaternion
        )?;
        
        // Visualize velocity as 3D vector (as points from origin)
        let velocity_points = vec![
            [0.0, 0.0, 0.0],  // Origin
            self.velocity,     // Velocity vector endpoint
        ];
        client.log_points_3d(&format!("{}/velocity", entity_path), velocity_points)?;
        
        // Log scalar values
        client.log_scalar(&format!("{}/battery_voltage", entity_path), self.battery_voltage)?;
        
        // Log system status
        client.log_text(&format!("{}/status", entity_path), &self.system_status)?;
        
        // Log sensor readings as individual scalars
        for (i, &reading) in self.sensor_readings.iter().enumerate() {
            client.log_scalar(
                &format!("{}/sensors/sensor_{}", entity_path, i),
                reading as f64
            )?;
        }
        
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let viz_client = VisualizationClient::new(VisualizationConfig::default())?;
    
    // Create and visualize custom robot state
    let robot_state = RobotState {
        position: [1.0, 2.0, 0.5],
        velocity: [0.5, 0.0, 0.0],
        battery_voltage: 12.6,
        system_status: "All systems nominal".to_string(),
        sensor_readings: vec![23.5, 45.2, 67.8, 12.1],
    };
    
    robot_state.visualize(&viz_client, "my_robot")?;
    
    println!("Custom data visualization complete");
    Ok(())
}
```

## Example 4: Real-time Sensor Visualization

This example demonstrates real-time sensor data visualization patterns.

```rust
use mini_ros::visualization::{VisualizationClient, VisualizationConfig};
use std::time::Duration;
use tokio::time::{sleep, interval};

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Enable real-time viewer (requires Rerun viewer installation)
    let config = VisualizationConfig {
        application_id: "SensorMonitor".to_string(),
        spawn_viewer: true,  // Launch viewer for real-time display
    };
    
    let viz_client = VisualizationClient::new(config)?;
    
    // Simulate multiple sensors updating at different rates
    let mut timer_fast = interval(Duration::from_millis(50));   // 20 Hz
    let mut timer_medium = interval(Duration::from_millis(200)); // 5 Hz
    let mut timer_slow = interval(Duration::from_secs(1));      // 1 Hz
    
    let mut counter = 0;
    
    loop {
        tokio::select! {
            // High-frequency sensor (IMU)
            _ = timer_fast.tick() => {
                let t = counter as f32 * 0.05;
                viz_client.log_scalar("sensors/imu/accel_x", (t * 2.0).sin() as f64)?;
                viz_client.log_scalar("sensors/imu/accel_y", (t * 1.5).cos() as f64)?;
                viz_client.log_scalar("sensors/imu/accel_z", 9.81 + 0.1 * (t * 3.0).sin() as f64)?;
            }
            
            // Medium-frequency sensor (GPS)
            _ = timer_medium.tick() => {
                let t = counter as f32 * 0.05;
                viz_client.log_transform_3d(
                    "sensors/gps/position",
                    [t.cos() * 10.0, t.sin() * 10.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                )?;
            }
            
            // Low-frequency sensor (Battery)
            _ = timer_slow.tick() => {
                let battery_level = 100.0 - (counter as f64 * 0.1);
                viz_client.log_scalar("sensors/battery/level", battery_level)?;
                
                let status = if battery_level > 20.0 { "OK" } else { "LOW BATTERY" };
                viz_client.log_text("sensors/battery/status", status)?;
            }
        }
        
        counter += 1;
        
        // Stop after 30 seconds
        if counter > 600 {
            break;
        }
    }
    
    println!("Sensor monitoring complete");
    Ok(())
}
```

## Running the Examples

To run these examples in your miniROS-rs project:

1. **Basic visualization demo:**
   ```bash
   cargo run --example visualization_demo
   ```

2. **Integrated communication and visualization:**
   ```bash
   cargo run --example integrated_demo
   ```

3. **Zenoh communication only:**
   ```bash
   cargo run --example zenoh_communication
   ```

## Tips for Effective Visualization

1. **Choose appropriate entity hierarchies** - Group related data logically
2. **Use meaningful names** - Make entity paths descriptive
3. **Control update frequency** - Balance between detail and performance
4. **Leverage data types** - Use appropriate Rerun primitives for your data
5. **Handle errors gracefully** - Visualization failures shouldn't crash your robot

## Next Steps

- Explore the [Data Types](./data-types.md) section for more details on supported data types
- Check the [API Reference](../api/visualization.md) for complete function documentation
- Visit the [Rerun documentation](https://rerun.io/docs) for advanced visualization techniques 