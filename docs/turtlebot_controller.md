# Turtlebot Controller Examples

## Overview

The turtlebot controller examples demonstrate classic ROS/ROS2 robotics patterns using miniROS-rs. These examples showcase essential robot control functionality with minimal complexity, following the miniROS "mini" philosophy.

## Features

### Core Robotics Communication
- **Velocity Commands**: Publishes `Twist` messages to `/cmd_vel` topic
- **Odometry Feedback**: Subscribes to `Odometry` messages from `/odom` topic
- **Safety Validation**: Built-in velocity limits and message validation

### Movement Patterns
- **Forward/Backward**: Linear motion control
- **Rotation**: In-place turning (left/right)
- **Circle**: Continuous circular motion
- **Square**: Programmatic square path following

### Message Types
```rust
// Velocity commands (linear + angular)
pub struct TwistMessage {
    pub linear: Vector3,   // m/s
    pub angular: Vector3,  // rad/s
}

// Robot state feedback
pub struct OdometryMessage {
    pub pose: PoseMessage,      // Position + orientation
    pub twist: TwistMessage,    // Current velocity
    pub header: Header,         // Timestamp + frame
}
```

## Usage

### Rust Example
```bash
cargo run --example 12_turtlebot_controller
```

Key features:
- Async/await patterns for modern Rust
- Type-safe message handling
- Built-in validation with safety limits
- Clean state machine for movement patterns

### Python Example  
```bash
cd python/examples
python turtlebot_controller.py
```

Key features:
- ROS2-compatible API (`rclpy`-style)
- Simple callback-based design
- Automatic message validation
- Easy-to-extend movement patterns

## Code Structure

### Rust Implementation
```rust
impl TurtlebotController {
    // Core velocity publishing
    pub async fn publish_velocity(&self, linear: Vector3, angular: Vector3) -> Result<()>
    
    // Movement pattern execution
    pub async fn execute_pattern(&mut self, pattern: MovementPattern, duration: f64) -> Result<()>
    
    // Odometry callback setup
    pub async fn setup_odometry_callback(&mut self) -> Result<()>
}
```

### Python Implementation
```python
class TurtlebotController:
    # Simple velocity commands
    def publish_velocity(self, linear_x=0.0, angular_z=0.0)
    
    # Pattern-based movement
    def execute_pattern(self, pattern, duration)
    
    # ROS2-style callbacks
    def _odom_callback(self, odom_msg)
```

## Safety Features

### Velocity Limits
- **Linear**: 2.0 m/s maximum
- **Angular**: 4.0 rad/s maximum
- **Validation**: Automatic safety checking

### Message Validation
- Finite value checking (no NaN/infinity)
- Quaternion normalization for poses
- Reasonable velocity bounds

## Demo Sequence

Both examples run the same demonstration:

1. **Forward Motion** (3 seconds)
   - Linear velocity: 0.5 m/s
   
2. **Circular Motion** (5 seconds)  
   - Linear: 0.3 m/s, Angular: 0.5 rad/s
   
3. **Square Pattern** (10 seconds)
   - 75% forward, 25% turning (90° turns)
   
4. **Automatic Stop**
   - Zero velocities published

## Integration

### With Real Robots
Replace mock odometry with real sensor data:
```rust
// Remove mock publishing
// self.publish_mock_odometry().await?;

// Connect to real robot's odometry stream
```

### With Simulators
Works directly with:
- Gazebo (ROS2 integration)
- Webots (via ROS2 bridge)
- Custom physics engines

## miniROS Philosophy

These examples embody miniROS core principles:

- **Essential Functionality**: Focus on core robotics communication
- **Minimal Complexity**: Clear, readable code without bloat
- **Cross-Language**: Same concepts in Rust and Python
- **Type Safety**: Compile-time guarantees where possible
- **Performance**: Efficient message passing and validation

Perfect starting point for robotics applications requiring reliable, high-performance communication with straightforward APIs. 