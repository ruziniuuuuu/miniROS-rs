# Rerun Integration

miniROS-rs integrates with [Rerun](https://rerun.io), a powerful visualization tool designed for robotics and computer vision applications. This integration allows you to visualize robot data in real-time or in buffered mode.

## Overview

Rerun provides a modern approach to robotics data visualization with:

- **Real-time visualization** of robot poses, sensor data, and point clouds
- **Time-series data** plotting for sensor readings
- **3D scene visualization** with coordinate frames and transformations
- **Multi-modal data support** including images, point clouds, and scalar data
- **Interactive exploration** with timeline scrubbing and data inspection

## Operating Modes

miniROS-rs supports two visualization modes:

### 1. Buffered Mode (Default)

In buffered mode, data is stored in memory without requiring an external viewer. This is the default mode and works out of the box.

```rust
use mini_ros::visualization::{VisualizationClient, VisualizationConfig};

let config = VisualizationConfig {
    application_id: "my_robot_app".to_string(),
    spawn_viewer: false,  // Buffered mode
};

let viz_client = VisualizationClient::new(config)?;
```

**Advantages:**
- No external dependencies required
- Works immediately after installation
- Good for logging and post-processing

**Limitations:**
- No real-time visual feedback
- Data is lost when program exits

### 2. Real-time Viewer Mode

In viewer mode, miniROS-rs launches the Rerun viewer for real-time visualization.

```rust
let config = VisualizationConfig {
    application_id: "my_robot_app".to_string(),
    spawn_viewer: true,  // Launch viewer
};

let viz_client = VisualizationClient::new(config)?;
```

**Requirements:**
The Rerun viewer must be installed separately. See [Installation](#installation) below.

**Advantages:**
- Real-time visual feedback
- Interactive data exploration
- Rich 3D visualization

## Installation

To use real-time visualization, install the Rerun viewer:

### Option 1: Cargo (Recommended)
```bash
cargo install rerun-cli
```

### Option 2: Python pip
```bash
pip install rerun-sdk==0.20.3
```

### Option 3: Direct Download
Visit the [Rerun releases page](https://github.com/rerun-io/rerun/releases/0.20.3/) and download the appropriate binary for your system.

## Basic Usage

### Creating a Visualization Client

```rust
use mini_ros::visualization::{VisualizationClient, VisualizationConfig};

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    let config = VisualizationConfig::default();
    let viz_client = VisualizationClient::new(config)?;
    
    // Use visualization client...
    
    Ok(())
}
```

### Logging Different Data Types

#### Scalar Values
```rust
// Log sensor readings
viz_client.log_scalar("sensors/temperature", 25.5)?;
viz_client.log_scalar("sensors/humidity", 60.2)?;
```

#### 3D Points
```rust
// Log point cloud data
let points = vec![
    [1.0, 2.0, 3.0],
    [4.0, 5.0, 6.0],
    [7.0, 8.0, 9.0],
];
viz_client.log_points_3d("lidar/points", points)?;
```

#### Transforms
```rust
// Log robot pose
let translation = [1.0, 2.0, 0.0];
let rotation = [0.0, 0.0, 0.0, 1.0]; // quaternion [x, y, z, w]
viz_client.log_transform_3d("robot/pose", translation, rotation)?;
```

#### Text Logs
```rust
// Log status messages
viz_client.log_text("robot/status", "System initialized")?;
viz_client.log_text("robot/errors", "Motor temperature high")?;
```

## Integration with miniROS-rs

### Visualizing Built-in Data Types

miniROS-rs provides built-in visualization support for common robotics data types:

```rust
use mini_ros::visualization::{RobotPose, PointCloud, LaserScan, Visualizable};

// Robot pose
let pose = RobotPose {
    position: [1.0, 2.0, 0.0],
    orientation: [0.0, 0.0, 0.0, 1.0],
};
pose.visualize(&viz_client, "robot/pose")?;

// Point cloud
let cloud = PointCloud {
    points: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
};
cloud.visualize(&viz_client, "sensors/lidar")?;

// Laser scan
let scan = LaserScan {
    ranges: vec![1.0, 2.0, 3.0, 4.0],
    angle_min: -std::f32::consts::PI,
    angle_max: std::f32::consts::PI,
};
scan.visualize(&viz_client, "sensors/laser_scan")?;
```

### Custom Visualization Types

You can implement the `Visualizable` trait for your own data types:

```rust
use mini_ros::visualization::{VisualizationClient, Visualizable};
use mini_ros::error::Result;

struct MyRobotState {
    position: [f32; 3],
    velocity: f32,
    battery_level: f64,
}

impl Visualizable for MyRobotState {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Log position as transform
        client.log_transform_3d(
            &format!("{}/pose", entity_path),
            self.position,
            [0.0, 0.0, 0.0, 1.0]
        )?;
        
        // Log velocity as scalar
        client.log_scalar(
            &format!("{}/velocity", entity_path),
            self.velocity as f64
        )?;
        
        // Log battery level
        client.log_scalar(
            &format!("{}/battery", entity_path),
            self.battery_level
        )?;
        
        Ok(())
    }
}
```

## Entity Hierarchy

Rerun organizes data in a hierarchical entity tree. Choose meaningful entity paths:

```rust
// Good entity organization
viz_client.log_transform_3d("robot/base_link", translation, rotation)?;
viz_client.log_points_3d("robot/sensors/lidar", points)?;
viz_client.log_scalar("robot/sensors/imu/acceleration", accel)?;
viz_client.log_text("robot/status/navigation", "Path planning active")?;

// Avoid flat organization
// viz_client.log_transform_3d("pose", translation, rotation)?;  // Too generic
// viz_client.log_points_3d("points", points)?;  // Not descriptive
```

## Performance Considerations

### Buffered Mode
- Data is stored in memory until program exit
- Low overhead for logging
- Suitable for high-frequency data

### Real-time Mode
- Data is streamed to viewer immediately
- Higher overhead due to network communication
- May impact performance with very high-frequency data

### Best Practices

1. **Use appropriate logging frequency**: Don't log every single data point if not necessary
2. **Batch related data**: Group related visualizations in time
3. **Choose descriptive entity paths**: Makes data exploration easier
4. **Limit data size**: Very large point clouds can impact performance

## Troubleshooting

### Viewer Not Found
If you see "Failed to find Rerun Viewer executable in PATH", install the Rerun viewer as described in the [Installation](#installation) section.

### Connection Issues
- Ensure no other applications are using the same ports
- Check firewall settings if viewer doesn't connect
- Try restarting both the application and viewer

### Performance Issues
- Reduce logging frequency for high-volume data
- Use buffered mode for better performance
- Consider downsampling large datasets

## Next Steps

- Check out the [Data Types](./data-types.md) section for detailed information about supported data types
- See [Examples](./examples.md) for complete working examples
- Visit the [Rerun documentation](https://rerun.io/docs) for advanced visualization techniques 