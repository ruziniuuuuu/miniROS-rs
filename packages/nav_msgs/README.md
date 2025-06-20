# nav_msgs Package

Navigation message definitions for miniROS, fully compatible with ROS2 nav_msgs package.

## Philosophy

Core navigation messages for mobile robotics - odometry, paths, and occupancy grids. Designed for high-frequency navigation data with minimal overhead.

## Message Types

### Robot State
- **Odometry** - Complete robot state (pose + velocity with uncertainty)

### Path Planning  
- **Path** - Sequence of poses representing a trajectory
- **OccupancyGrid** - 2D map representation for navigation
- **MapMetaData** - Map metadata (resolution, origin, dimensions)

## Usage Examples

### Rust
```rust
use mini_ros::types::{nav_msgs::*, geometry_msgs::*, std_msgs::*};

// Robot odometry
let odom = Odometry {
    header: Header {
        stamp: current_time_ns(),
        frame_id: "odom".to_string(),
    },
    child_frame_id: "base_link".to_string(),
    pose: PoseWithCovariance {
        pose: Pose {
            position: Point { x: 1.0, y: 2.0, z: 0.0 },
            orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        },
        covariance: vec![0.0; 36], // 6x6 covariance matrix
    },
    twist: TwistWithCovariance {
        twist: Twist {
            linear: Vector3 { x: 0.5, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: 0.1 },
        },
        covariance: vec![0.0; 36],
    },
};

// Navigation path
let path = Path {
    header: Header {
        stamp: current_time_ns(),
        frame_id: "map".to_string(),
    },
    poses: vec![
        PoseStamped { /* waypoint 1 */ },
        PoseStamped { /* waypoint 2 */ },
        // ... more waypoints
    ],
};
```

### Python
```python
import mini_ros

# Robot odometry
odom = mini_ros.nav_msgs.Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

# Set pose
odom.pose.pose.position.x = 1.0
odom.pose.pose.position.y = 2.0
odom.pose.pose.orientation.w = 1.0

# Set velocity
odom.twist.twist.linear.x = 0.5
odom.twist.twist.angular.z = 0.1
```

## Features

### High-Performance Navigation
- **Efficient Serialization** - Optimized for high-frequency odometry updates
- **Covariance Support** - Full uncertainty representation for sensor fusion
- **Path Validation** - Automatic validation of path lengths and pose sequences

### Safety & Validation
- **Finite Value Checking** - Ensures all coordinates are valid numbers
- **Path Length Limits** - Prevents excessively long paths (>10,000 poses)
- **Covariance Matrix Validation** - Checks uncertainty matrices for validity

## Common Use Cases

1. **Robot Localization** - Publishing odometry from wheel encoders + IMU
2. **Path Planning** - Sending navigation goals and planned trajectories  
3. **SLAM** - Mapping and localization with uncertainty estimates
4. **Navigation Stack** - Full ROS2 nav2 compatibility

## ROS2 Compatibility

Fully compatible with ROS2 navigation stack:
- Works with nav2 navigation framework
- Compatible with SLAM toolbox
- Supports move_base replacement workflows
- Standard message format for all navigation tools 