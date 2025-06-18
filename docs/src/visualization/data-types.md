# Visualization Data Types

miniROS-rs provides built-in support for common robotics data types that can be easily visualized with Rerun. This section describes the available data types and how to use them.

## Built-in Data Types

### RobotPose

Represents a robot's position and orientation in 3D space.

```rust
use mini_ros::visualization::{RobotPose, Visualizable};

let pose = RobotPose {
    position: [x, y, z],              // Position in meters
    orientation: [qx, qy, qz, qw],    // Quaternion rotation [x, y, z, w]
};

// Visualize the pose
pose.visualize(&viz_client, "robot/pose")?;
```

**Use Cases:**
- Robot base position and orientation
- End-effector poses
- Goal positions for navigation
- Camera poses

### PointCloud

Represents a collection of 3D points, commonly used for LIDAR data or 3D reconstruction.

```rust
use mini_ros::visualization::{PointCloud, Visualizable};

let cloud = PointCloud {
    points: vec![
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0],
    ],
};

// Visualize the point cloud
cloud.visualize(&viz_client, "sensors/lidar")?;
```

**Use Cases:**
- LIDAR/LiDAR point clouds
- Stereo camera depth data
- 3D mapping results
- Obstacle detection points

### LaserScan

Represents 2D laser scan data, commonly from LIDAR sensors.

```rust
use mini_ros::visualization::{LaserScan, Visualizable};

let scan = LaserScan {
    ranges: vec![1.0, 1.5, 2.0, 2.5, 3.0],  // Distances in meters
    angle_min: -std::f32::consts::PI / 2.0,  // Start angle (radians)
    angle_max: std::f32::consts::PI / 2.0,   // End angle (radians)
};

// Visualize as 2D points
scan.visualize(&viz_client, "sensors/laser_scan")?;
```

**Use Cases:**
- 2D LIDAR scans
- Ultrasonic sensor arrays
- IR sensor data
- Proximity detection

## Primitive Data Types

### Scalar Values

Log numerical sensor readings, metrics, or any single values.

```rust
// Temperature sensor
viz_client.log_scalar("sensors/temperature", 25.5)?;

// Battery voltage
viz_client.log_scalar("power/battery_voltage", 12.6)?;

// Speed
viz_client.log_scalar("motion/velocity", 2.3)?;

// Error metrics
viz_client.log_scalar("navigation/path_error", 0.15)?;
```

**Supported Types:**
- `f64` (double precision)
- `f32` (automatically converted to f64)
- `i32`, `i64` (automatically converted to f64)

### 3D Points

Visualize collections of 3D points.

```rust
let points = vec![
    [1.0, 0.0, 0.0],  // Red point at (1,0,0)
    [0.0, 1.0, 0.0],  // Green point at (0,1,0)
    [0.0, 0.0, 1.0],  // Blue point at (0,0,1)
];

viz_client.log_points_3d("debug/waypoints", points)?;
```

### 2D Points

Visualize collections of 2D points.

```rust
let points_2d = vec![
    [1.0, 2.0],
    [3.0, 4.0],
    [5.0, 6.0],
];

viz_client.log_points_2d("map/landmarks", points_2d)?;
```

### 3D Transforms

Log position and orientation together.

```rust
let translation = [1.0, 2.0, 0.5];  // Position
let rotation = [0.0, 0.0, 0.707, 0.707];  // Quaternion (45° around Z)

viz_client.log_transform_3d("robot/base_link", translation, rotation)?;
```

### Text Messages

Log text information, status messages, and debug output.

```rust
// Status messages
viz_client.log_text("robot/status", "Navigation active")?;

// Error messages
viz_client.log_text("robot/errors", "Motor overheat detected")?;

// Debug information
viz_client.log_text("debug/state", &format!("Current state: {:?}", state))?;
```

## Custom Data Types

### Implementing Visualizable

Create custom visualization for your own data types by implementing the `Visualizable` trait.

```rust
use mini_ros::visualization::{VisualizationClient, Visualizable};
use mini_ros::error::Result;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BatteryStatus {
    voltage: f64,
    current: f64,
    temperature: f64,
    charge_percentage: f64,
}

impl Visualizable for BatteryStatus {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Log all battery metrics
        client.log_scalar(&format!("{}/voltage", entity_path), self.voltage)?;
        client.log_scalar(&format!("{}/current", entity_path), self.current)?;
        client.log_scalar(&format!("{}/temperature", entity_path), self.temperature)?;
        client.log_scalar(&format!("{}/charge", entity_path), self.charge_percentage)?;
        
        // Log status text
        let status = if self.charge_percentage > 20.0 {
            "Battery OK"
        } else {
            "Battery LOW"
        };
        client.log_text(&format!("{}/status", entity_path), status)?;
        
        Ok(())
    }
}

// Usage
let battery = BatteryStatus {
    voltage: 12.6,
    current: -2.3,
    temperature: 25.0,
    charge_percentage: 85.0,
};

battery.visualize(&viz_client, "power/battery")?;
```

### Complex Custom Types

For more complex data types, combine multiple Rerun primitives:

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotArm {
    joint_positions: Vec<f64>,  // Joint angles in radians
    end_effector_pose: [f32; 7], // [x, y, z, qx, qy, qz, qw]
    joint_torques: Vec<f64>,
}

impl Visualizable for RobotArm {
    fn visualize(&self, client: &VisualizationClient, entity_path: &str) -> Result<()> {
        // Log end effector pose
        let translation = [
            self.end_effector_pose[0],
            self.end_effector_pose[1],
            self.end_effector_pose[2],
        ];
        let rotation = [
            self.end_effector_pose[3],
            self.end_effector_pose[4],
            self.end_effector_pose[5],
            self.end_effector_pose[6],
        ];
        client.log_transform_3d(&format!("{}/end_effector", entity_path), translation, rotation)?;
        
        // Log joint positions and torques
        for (i, (&position, &torque)) in self.joint_positions.iter()
            .zip(self.joint_torques.iter())
            .enumerate() {
            client.log_scalar(&format!("{}/joint_{}/position", entity_path, i), position)?;
            client.log_scalar(&format!("{}/joint_{}/torque", entity_path, i), torque)?;
        }
        
        Ok(())
    }
}
```

## Entity Paths and Organization

### Hierarchical Organization

Organize your data logically using hierarchical entity paths:

```rust
// Good organization
viz_client.log_transform_3d("robot/base_link", pos, rot)?;
viz_client.log_points_3d("robot/sensors/lidar/points", points)?;
viz_client.log_scalar("robot/sensors/imu/accel_x", accel_x)?;
viz_client.log_text("robot/navigation/status", "Path planning")?;

// Poor organization (too flat)
viz_client.log_transform_3d("pose", pos, rot)?;
viz_client.log_points_3d("points", points)?;
viz_client.log_scalar("accel", accel_x)?;
```

### Naming Conventions

Follow these conventions for consistent entity naming:

- Use forward slashes (`/`) to separate hierarchy levels
- Use snake_case for multi-word names
- Group related data under common prefixes
- Be descriptive but concise

**Examples:**
```rust
// Sensors
"robot/sensors/lidar/scan"
"robot/sensors/camera/image"
"robot/sensors/imu/acceleration"

// Actuators
"robot/actuators/left_wheel/velocity"
"robot/actuators/right_wheel/velocity"
"robot/actuators/arm/joint_0/position"

// Navigation
"robot/navigation/goal_pose"
"robot/navigation/path"
"robot/navigation/obstacles"

// System
"robot/system/battery/voltage"
"robot/system/cpu/temperature"
"robot/system/memory/usage"
```

## Performance Considerations

### Update Frequency

Consider the appropriate update frequency for different data types:

```rust
// High frequency (10-100 Hz) - critical real-time data
viz_client.log_scalar("control/error", control_error)?;

// Medium frequency (1-10 Hz) - monitoring data
viz_client.log_scalar("sensors/temperature", temp)?;

// Low frequency (0.1-1 Hz) - status information
viz_client.log_text("system/status", status)?;
```

### Data Size

Be mindful of large data structures:

```rust
// Large point clouds - consider downsampling
if points.len() > 10000 {
    // Downsample to every 10th point
    let downsampled: Vec<_> = points.iter().step_by(10).cloned().collect();
    viz_client.log_points_3d("lidar/downsampled", downsampled)?;
} else {
    viz_client.log_points_3d("lidar/full", points)?;
}
```

## Data Type Summary

| Type | Rerun Primitive | Use Case | Performance |
|------|-----------------|----------|-------------|
| `RobotPose` | Transform3D | Robot poses, goals | Medium |
| `PointCloud` | Points3D | LIDAR, 3D sensing | High* |
| `LaserScan` | Points2D | 2D LIDAR, proximity | Medium |
| Scalar | Scalar | Sensors, metrics | Low |
| Points3D | Points3D | Waypoints, features | Medium* |
| Points2D | Points2D | Map features, 2D data | Low |
| Transform3D | Transform3D | Poses, frames | Low |
| Text | TextLog | Status, debug | Low |

*Performance impact depends on number of points

## Next Steps

- See [Examples](./examples.md) for complete usage examples
- Check [Rerun Integration](./rerun.md) for setup and configuration
- Explore the [API Reference](../api/visualization.md) for detailed function documentation 