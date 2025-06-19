# 3D Visualization

miniROS-rs includes **built-in 3D visualization** powered by [Rerun](https://rerun.io), providing real-time data visualization for robotics applications without external dependencies.

## 🎯 Why Built-in Visualization?

- **Zero Setup**: No separate tools or installations required
- **Real-time**: Live data streaming with automatic updates
- **3D Native**: Built for robotics coordinate frames and transforms
- **Performance**: Optimized for high-frequency robot data
- **Integration**: Deep integration with miniROS-rs message types

## 🚀 Quick Start

### Enable Visualization

Add the visualization feature to your `Cargo.toml`:

```toml
[dependencies]
mini-ros = { version = "0.1.2", features = ["visualization"] }
```

### Basic Example

```rust
use mini_ros::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let mut node = Node::new("viz_demo")?;
    node.init().await?;
    
    // Create visualizer
    #[cfg(feature = "visualization")]
    {
        let viz = node.create_visualizer("robot_viz").await?;
        
        // Log a 3D point
        viz.log_point("robot/position", [1.0, 2.0, 0.5]).await?;
        
        // Log a trajectory
        let trajectory = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, 1.0],
        ];
        viz.log_path("robot/trajectory", &trajectory).await?;
        
        // Log text information
        viz.log_text("robot/status", "Navigation active").await?;
    }
    
    Ok(())
}
```

Run with:
```bash
cargo run --example basic_viz --features visualization
```

## 📊 Visualization API

### Core Visualizer

```rust
// Create visualizer instance
let viz = node.create_visualizer("app_name").await?;

// The visualizer automatically:
// - Spawns Rerun viewer
// - Manages connections
// - Handles data streaming
```

### 3D Points

```rust
// Single point
viz.log_point("entity/path", [x, y, z]).await?;

// Multiple points
let points = vec![
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
];
viz.log_points("entity/path", &points).await?;

// Points with colors
viz.log_points_with_colors("markers", &points, &colors).await?;
```

### Trajectories and Paths

```rust
// Robot trajectory
let path = vec![
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [2.0, 1.0, 0.0],
    [3.0, 1.0, 1.0],
];
viz.log_path("robot/trajectory", &path).await?;

// Planned vs actual paths
viz.log_path("planning/planned_path", &planned_path).await?;
viz.log_path("navigation/actual_path", &actual_path).await?;
```

### Text and Labels

```rust
// Status information
viz.log_text("robot/status", "Navigating to waypoint").await?;

// Error messages
viz.log_text("system/errors", "Battery low: 15%").await?;

// Dynamic information
viz.log_text("telemetry/speed", &format!("Speed: {:.2} m/s", speed)).await?;
```

### 3D Meshes and Models

```rust
// Robot model (placeholder)
viz.log_mesh("robot/model", mesh_data).await?;

// Obstacle representation
viz.log_mesh("environment/obstacles", obstacle_mesh).await?;
```

### Coordinate Frames

```rust
// Robot base frame
viz.log_transform("robot/base_link", position, orientation).await?;

// Sensor frames
viz.log_transform("robot/camera_link", cam_pos, cam_orient).await?;
viz.log_transform("robot/lidar_link", lidar_pos, lidar_orient).await?;
```

## 🤖 Robotics-Specific Features

### Sensor Data Visualization

#### LIDAR Point Clouds
```rust
// LIDAR scan visualization
let scan_points: Vec<[f32; 3]> = lidar_data
    .ranges
    .iter()
    .enumerate()
    .map(|(i, &range)| {
        let angle = lidar_data.angle_min + i as f32 * lidar_data.angle_increment;
        [
            range * angle.cos(),
            range * angle.sin(),
            0.0,
        ]
    })
    .collect();

viz.log_points("sensors/lidar", &scan_points).await?;
```

#### Camera Data
```rust
// Camera pose and field of view
viz.log_transform("sensors/camera", camera_pos, camera_orient).await?;
viz.log_text("sensors/camera/info", &format!("FOV: {}°", fov)).await?;
```

#### IMU Data
```rust
// Orientation visualization
viz.log_transform("sensors/imu", position, imu_orientation).await?;
viz.log_text("sensors/imu/data", &format!("Accel: {:.2}", acceleration)).await?;
```

### Robot State Visualization

#### Joint States
```rust
// Robot arm joint positions
for (i, joint_pos) in joint_positions.iter().enumerate() {
    viz.log_text(
        &format!("robot/joints/joint_{}", i),
        &format!("{:.2}°", joint_pos.to_degrees())
    ).await?;
}
```

#### Velocity and Forces
```rust
// Velocity vector
let velocity_end = [
    position[0] + velocity[0],
    position[1] + velocity[1], 
    position[2] + velocity[2],
];
viz.log_path("robot/velocity", &[position, velocity_end]).await?;
```

### Navigation Visualization

#### Path Planning
```rust
// Global path
viz.log_path("navigation/global_path", &global_path).await?;

// Local path
viz.log_path("navigation/local_path", &local_path).await?;

// Waypoints
viz.log_points("navigation/waypoints", &waypoints).await?;
```

#### Obstacle Avoidance
```rust
// Detected obstacles
viz.log_points("navigation/obstacles", &obstacle_points).await?;

// Safety zones
viz.log_mesh("navigation/safety_zone", safety_zone_mesh).await?;
```

## 📈 Real-time Data Streaming

### Continuous Updates

```rust
use tokio::time::{interval, Duration};

let mut timer = interval(Duration::from_millis(100)); // 10 Hz

loop {
    timer.tick().await;
    
    // Update robot position
    let current_pos = get_robot_position();
    viz.log_point("robot/position", current_pos).await?;
    
    // Update sensor data
    let lidar_data = get_lidar_scan();
    viz.log_points("sensors/lidar", &lidar_data).await?;
    
    // Update status
    let status = get_robot_status();
    viz.log_text("robot/status", &status).await?;
}
```

### Performance Optimization

```rust
// Batch updates for efficiency
let updates = vec![
    ("robot/position", current_position),
    ("robot/velocity", velocity_vector),
    ("robot/acceleration", accel_vector),
];

for (entity, data) in updates {
    viz.log_point(entity, data).await?;
}

// Use appropriate update frequencies
// High frequency: Robot pose (100Hz)
// Medium frequency: Sensor data (30Hz) 
// Low frequency: Status text (1Hz)
```

## 🔧 Integration Examples

### Complete Robot Visualizer

```rust
use mini_ros::prelude::*;
use tokio::time::{interval, Duration};

struct RobotVisualizer {
    viz: Visualizer,
    position_sub: Subscriber<RobotPose>,
    lidar_sub: Subscriber<LaserScan>,
    status_sub: Subscriber<StringMsg>,
}

impl RobotVisualizer {
    async fn new(node: &mut Node) -> Result<Self> {
        let viz = node.create_visualizer("robot_system").await?;
        
        let position_sub = node.create_subscriber("/robot/pose").await?;
        let lidar_sub = node.create_subscriber("/sensors/lidar").await?;
        let status_sub = node.create_subscriber("/robot/status").await?;
        
        Ok(Self {
            viz,
            position_sub,
            lidar_sub,
            status_sub,
        })
    }
    
    async fn start(&self) -> Result<()> {
        // Set up callbacks
        self.position_sub.on_message({
            let viz = self.viz.clone();
            move |pose: RobotPose| {
                tokio::spawn(async move {
                    viz.log_point("robot/position", pose.position).await?;
                    viz.log_transform("robot/base_link", pose.position, pose.orientation).await?;
                });
            }
        })?;
        
        self.lidar_sub.on_message({
            let viz = self.viz.clone();
            move |scan: LaserScan| {
                tokio::spawn(async move {
                    let points = convert_scan_to_points(&scan);
                    viz.log_points("sensors/lidar", &points).await?;
                });
            }
        })?;
        
        self.status_sub.on_message({
            let viz = self.viz.clone();
            move |status: StringMsg| {
                tokio::spawn(async move {
                    viz.log_text("robot/status", &status.data).await?;
                });
            }
        })?;
        
        Ok(())
    }
}
```

### Multi-Robot Visualization

```rust
// Visualize multiple robots
for (i, robot_pose) in robot_poses.iter().enumerate() {
    let entity_path = format!("robots/robot_{}", i);
    viz.log_point(&format!("{}/position", entity_path), robot_pose.position).await?;
    viz.log_transform(&format!("{}/base_link", entity_path), 
                     robot_pose.position, robot_pose.orientation).await?;
    viz.log_text(&format!("{}/id", entity_path), &format!("Robot {}", i)).await?;
}
```

## 🎮 Interactive Features

### Viewing Controls

The Rerun viewer provides:
- **3D Navigation**: Mouse-based camera control
- **Timeline**: Scrub through historical data
- **Entity Tree**: Toggle visibility of different entities
- **Measurements**: Distance and angle measurements
- **Screenshots**: Export visualizations

### Entity Organization

```rust
// Organize entities hierarchically
viz.log_point("robots/robot_1/sensors/lidar", lidar_pos).await?;
viz.log_point("robots/robot_1/actuators/gripper", gripper_pos).await?;
viz.log_point("robots/robot_2/sensors/camera", camera_pos).await?;

// Environment entities
viz.log_points("environment/obstacles", &obstacles).await?;
viz.log_points("environment/landmarks", &landmarks).await?;
```

## 📱 Running Examples

### Basic Visualization
```bash
cargo run --example 04_visualization_basic --features visualization
```

### Advanced 3D Scene
```bash
cargo run --example 06_visualization_advanced --features visualization
```

### Integrated System with Visualization
```bash
cargo run --example 07_integrated_system --features visualization
```

## 🔧 Configuration and Setup

### Automatic Viewer Launch

The visualizer automatically:
1. Spawns the Rerun viewer application
2. Establishes connection
3. Begins streaming data
4. Handles viewer lifecycle

### Manual Viewer Control

```rust
// For custom setups
use rerun as rr;

rr::init("custom_app")?;
rr::spawn()?; // Launch viewer manually

// Then use standard miniROS viz API
let viz = node.create_visualizer("custom_app").await?;
```

### Environment Configuration

```bash
# Disable automatic viewer launch
export RERUN_NO_SPAWN=1

# Custom viewer settings
export RERUN_HOST=localhost
export RERUN_PORT=9876
```

## 🚧 Limitations and Future Work

### Current Limitations
- **Mesh Support**: Limited 3D mesh capabilities
- **Textures**: No texture mapping support
- **Animation**: No built-in animation features
- **Custom Shaders**: No custom rendering pipeline

### Planned Features
- [ ] Advanced mesh rendering
- [ ] Texture and material support
- [ ] Robot URDF model loading
- [ ] Custom visualization widgets
- [ ] Web-based viewer option
- [ ] VR/AR visualization support

## 💡 Tips and Best Practices

1. **Entity Naming**: Use clear, hierarchical entity paths
2. **Update Frequency**: Match visualization frequency to data importance
3. **Performance**: Batch updates when possible
4. **Organization**: Group related entities logically
5. **Testing**: Use visualization to debug robot behavior
6. **Documentation**: Label entities clearly for team collaboration

## 🔗 Integration with External Tools

### ROS2 Bridge
```rust
// Visualize ROS2 data through miniROS-rs
let ros2_bridge = Ros2Bridge::new().await?;
ros2_bridge.subscribe("/turtle1/pose", |pose| {
    viz.log_point("turtle/position", [pose.x, pose.y, 0.0]).await?;
});
```

### Gazebo Integration
```rust
// Sync with Gazebo simulation
let gazebo_sync = GazeboSync::new().await?;
gazebo_sync.on_model_update(|model| {
    viz.log_transform(&format!("gazebo/{}", model.name), 
                     model.position, model.orientation).await?;
});
```

The built-in visualization makes miniROS-rs a complete robotics development platform, providing immediate visual feedback for debugging, development, and demonstration! 🎨🤖 