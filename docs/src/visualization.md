# Visualization

Real-time data visualization with Rerun integration.

## Quick Start

### Basic Setup

```rust
use mini_ros::visualization::*;

// Create visualization client
let config = VisualizationConfig {
    application_id: "MyRobot".to_string(),
    spawn_viewer: true,  // Automatically opens GUI
};
let viz = VisualizationClient::new(config)?;
```

### Simple Data Logging

```rust
// Scalar values (plots as time series)
viz.log_scalar("battery_voltage", 12.5)?;
viz.log_scalar("cpu_usage", 45.0)?;

// Text messages
viz.log_text("status", "Robot is operational")?;
viz.log_text("errors", "No errors detected")?;

// 2D points
let points_2d = vec![[1.0, 2.0], [3.0, 4.0]];
viz.log_points_2d("sensor/detections", points_2d)?;

// 3D points  
let points_3d = vec![[1.0, 2.0, 0.1], [3.0, 4.0, 0.2]];
viz.log_points_3d("lidar/points", points_3d)?;
```

## Robot Data Types

### Robot Pose

```rust
use mini_ros::visualization::{RobotPose, Visualizable};

// Create robot pose
let pose = RobotPose {
    position: [1.0, 2.0, 0.0],
    orientation: [0.0, 0.0, 0.0, 1.0], // [x, y, z, w] quaternion
};

// Visualize
pose.visualize(&viz, "robot/pose")?;
```

### Point Cloud

```rust
use mini_ros::visualization::{PointCloud, Visualizable};

let cloud = PointCloud {
    points: vec![
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0], 
        [0.0, 0.0, 1.0],
    ],
};

cloud.visualize(&viz, "sensors/pointcloud")?;
```

### Laser Scan

```rust
use mini_ros::visualization::{LaserScan, Visualizable};

let scan = LaserScan {
    ranges: vec![1.0, 1.5, 2.0, 1.8, 1.2], // Distance measurements
    angle_min: -1.57,  // -90 degrees
    angle_max: 1.57,   // +90 degrees
};

scan.visualize(&viz, "sensors/laser")?;
```

## Organization Tips

### Entity Hierarchy

Use forward slashes to organize data:

```rust
// Robot data
viz.log_scalar("robot/battery", 85.0)?;
viz.log_scalar("robot/speed", 1.5)?;

// Sensor data
viz.log_points_3d("sensors/lidar/points", points)?;
viz.log_scalar("sensors/camera/fps", 30.0)?;

// Mission data
viz.log_text("mission/current_goal", "Navigate to waypoint 3")?;
viz.log_scalar("mission/progress", 65.0)?;
```

### Time Series Data

Rerun automatically handles time series for scalar data:

```rust
// Log data in a loop - Rerun will plot over time
for i in 0..100 {
    let temperature = 20.0 + 5.0 * (i as f64 * 0.1).sin();
    viz.log_scalar("sensors/temperature", temperature)?;
    
    tokio::time::sleep(Duration::from_millis(100)).await;
}
```

## Complete Example

```rust
use mini_ros::visualization::*;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<()> {
    // Start visualization
    let config = VisualizationConfig {
        application_id: "RobotDemo".to_string(), 
        spawn_viewer: true,
    };
    let viz = VisualizationClient::new(config)?;
    
    // Simulate robot mission
    for step in 0..50 {
        let t = step as f64 * 0.1;
        
        // Robot moving in circle
        let x = 2.0 * t.cos();
        let y = 2.0 * t.sin();
        let pose = RobotPose {
            position: [x as f32, y as f32, 0.1],
            orientation: [0.0, 0.0, (t / 2.0).sin() as f32, (t / 2.0).cos() as f32],
        };
        pose.visualize(&viz, "robot/pose")?;
        
        // System metrics
        viz.log_scalar("battery", 100.0 - t * 2.0)?;
        viz.log_scalar("speed", 1.0 + 0.5 * (t * 0.5).sin())?;
        
        // Status updates
        if step % 10 == 0 {
            viz.log_text("status", &format!("Step {}/50", step + 1))?;
        }
        
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    
    println!("Check the Rerun window!");
    tokio::time::sleep(Duration::from_secs(10)).await;
    
    Ok(())
}
```

## Troubleshooting

### GUI Not Opening
Make sure Rerun CLI version matches SDK:
```bash
cargo install rerun-cli@0.20.3 --force
```

### No Data Visible
- Check entity paths are correct
- Ensure data types are compatible
- Look for error messages in terminal

### Performance Issues  
- Use appropriate data types (f32 vs f64)
- Limit point cloud sizes
- Consider data decimation for high-frequency data

## Next Steps

- Try [Example 04](./examples.md#04---basic-visualization) for basic visualization
- Try [Example 06](./examples.md#06---advanced-3d-visualization) for 3D visualization  
- See [API Reference](./api.md) for complete documentation 