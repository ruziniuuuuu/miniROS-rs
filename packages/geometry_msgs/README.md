# geometry_msgs Package

Geometric message definitions for miniROS, fully compatible with ROS2 geometry_msgs package.

## Philosophy

Essential geometric types for robotics applications - points, vectors, poses, and transformations. Optimized for performance while maintaining mathematical correctness and ROS2 compatibility.

## Message Types

### Basic Geometric Types
- **Point** - 3D position (x, y, z) with double precision
- **Vector3** - 3D vector representation for directions and velocities
- **Quaternion** - Normalized quaternion for 3D rotations

### Composite Types
- **Pose** - Position + orientation in 3D space
- **PoseStamped** - Pose with header (timestamp + frame)
- **Twist** - Linear and angular velocity
- **PoseWithCovariance** - Pose with uncertainty matrix
- **TwistWithCovariance** - Twist with uncertainty matrix

### Transform Types
- **Transform** - 3D transformation (translation + rotation)
- **TransformStamped** - Transform with header and child frame

## Usage Examples

### Rust
```rust
use mini_ros::types::geometry_msgs::*;

// Create a 3D point
let point = Point {
    x: 1.0,
    y: 2.0,
    z: 3.0,
};

// Create a normalized quaternion (identity rotation)
let quat = Quaternion {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0,
};

// Velocity command for robot
let twist = Twist {
    linear: Vector3 { x: 0.5, y: 0.0, z: 0.0 },  // 0.5 m/s forward
    angular: Vector3 { x: 0.0, y: 0.0, z: 1.0 }, // 1 rad/s turn
};
```

### Python
```python
import mini_ros

# 3D point
point = mini_ros.geometry_msgs.Point()
point.x, point.y, point.z = 1.0, 2.0, 3.0

# Robot pose
pose = mini_ros.geometry_msgs.Pose()
pose.position = point
pose.orientation.w = 1.0  # Identity quaternion
```

## Validation Features

- **Finite Values** - Automatic validation that coordinates are finite (not NaN/Inf)
- **Quaternion Normalization** - Ensures quaternions are properly normalized
- **Velocity Limits** - Safety limits for robot velocity commands
- **Covariance Matrix** - Validation for uncertainty representations

## Safety Features

The Twist message includes built-in safety limits:
- Maximum linear velocity: 2.0 m/s
- Maximum angular velocity: 4.0 rad/s

These can be customized for specific robot platforms while maintaining type safety.

## ROS2 Compatibility

All message types are fully compatible with ROS2 geometry_msgs:
- Same field names and types
- Identical serialization format
- Compatible with existing ROS2 nodes and tools 