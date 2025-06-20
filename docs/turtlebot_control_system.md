# Turtlebot Control System

## Overview

The miniROS-rs turtlebot control system consists of three main components that work together to provide a complete robotics control and visualization experience:

1. **Turtlebot Simulator** - Physics simulation with Rerun visualization
2. **Teleop Controller** - Keyboard-based robot control  
3. **Basic Controller** - Programmatic movement patterns

## Quick Start

### Complete Control System (Recommended)

Run the full system with two terminals:

```bash
# Terminal 1: Start simulator with visualization
cargo run --example 14_turtlebot_simulator --features visualization

# Terminal 2: Start keyboard control
cargo run --example 13_turtlebot_teleop
```

### Individual Components

```bash
# Standalone programmatic controller
cargo run --example 12_turtlebot_controller

# Teleop only (publishes to /cmd_vel)
cargo run --example 13_turtlebot_teleop

# Simulator only (subscribes to /cmd_vel, publishes /odom)
cargo run --example 14_turtlebot_simulator --features visualization
```

## System Architecture

```
┌─────────────────┐    /cmd_vel    ┌─────────────────┐
│  Teleop Node    │───────────────▶│ Simulator Node  │
│ (Keyboard Input)│                │  (Physics +     │
└─────────────────┘                │  Visualization) │
                                   └─────────────────┘
                                           │
                                           ▼ /odom
                                   ┌─────────────────┐
                                   │   Rerun Viewer  │
                                   │ (3D Visualization)│
                                   └─────────────────┘
```

## Component Details

### 1. Turtlebot Simulator (`14_turtlebot_simulator`)

**Functionality:**
- Subscribes to `/cmd_vel` for velocity commands
- Publishes `/odom` for robot state feedback
- Real-time physics simulation with differential drive kinematics
- Rerun-based 3D visualization

**Features:**
- Robot position and orientation tracking
- Path visualization (trajectory history)
- Velocity vector display
- Real-time telemetry plots
- 50Hz simulation rate

**Visualization Elements:**
- Robot pose (position + orientation)
- Movement trajectory trail
- Velocity vectors
- Coordinate system
- Real-time position/velocity plots

### 2. Teleop Controller (`13_turtlebot_teleop`)

**Functionality:**
- Real-time keyboard input capture
- Publishes velocity commands to `/cmd_vel`
- Safety limits and emergency stop
- Dynamic speed adjustment

**Controls:**
- `W/S`: Forward/Backward
- `A/D`: Turn Left/Right
- `Q/E`: Increase/Decrease speed
- `Spacebar`: Emergency stop
- `ESC`: Exit

**Safety Features:**
- Maximum velocity limits (1.0 m/s linear, 2.0 rad/s angular)
- Message validation before publishing
- Clean shutdown handling
- Terminal restoration on exit

### 3. Basic Controller (`12_turtlebot_controller`)

**Functionality:**
- Programmatic movement patterns
- Autonomous demo sequences
- Mock odometry for testing

**Movement Patterns:**
- Forward/backward motion
- Circular trajectories
- Square path following
- Emergency stop

## Message Types

### Twist Message (Velocity Commands)
```rust
pub struct TwistMessage {
    pub linear: Vector3,   // Linear velocity [x, y, z] in m/s
    pub angular: Vector3,  // Angular velocity [x, y, z] in rad/s
}
```

### Odometry Message (Robot State)
```rust
pub struct OdometryMessage {
    pub pose: PoseMessage,      // Position + orientation
    pub twist: TwistMessage,    // Current velocity
    pub header: Header,         // Timestamp + frame
}
```

## Usage Examples

### Real-time Control
```bash
# Start simulator
cargo run --example 14_turtlebot_simulator --features visualization

# In another terminal, control the robot
cargo run --example 13_turtlebot_teleop
# Use WASD keys to drive the robot
# Watch real-time visualization in Rerun viewer
```

### Programmatic Control
```bash
# Run demo patterns
cargo run --example 12_turtlebot_controller
# Watch automated movement sequences
```

### Custom Integration
```rust
use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, Vector3};

// Create custom controller
let mut node = Node::new("my_controller")?;
node.init().await?;

let cmd_vel_pub = node.create_publisher::<TwistMessage>("/cmd_vel").await?;

// Send movement command
let twist = TwistMessage {
    linear: Vector3 { x: 0.5, y: 0.0, z: 0.0 },   // 0.5 m/s forward
    angular: Vector3 { x: 0.0, y: 0.0, z: 0.2 },  // 0.2 rad/s turn
};

cmd_vel_pub.publish(&twist).await?;
```

## Visualization

### Rerun Features
When running with `--features visualization`:
- **3D Scene**: Robot, path, and coordinate system
- **Real-time Plots**: Position, velocity, and orientation over time
- **Interactive Viewer**: Pan, zoom, and inspect robot state
- **Time Scrubbing**: Review robot movement history

### Visualization Elements
- Robot pose indicator (position + orientation arrow)
- Trajectory path (breadcrumb trail)
- Velocity vectors (current motion direction)
- Telemetry plots (position, velocity, angle over time)
- Ground plane reference grid

## Development Notes

### Safety Features
- Velocity limits prevent dangerous commands
- Message validation ensures data integrity
- Emergency stop functionality
- Clean shutdown procedures

### Performance
- 50Hz simulation rate for smooth operation
- Efficient message passing with binary serialization
- Minimal latency between command and response
- Real-time visualization updates

### Extensibility
- Easy to add new movement patterns
- Pluggable visualization backends
- Custom message types support
- Integration with external simulators

## Troubleshooting

### Common Issues

**Keyboard not responsive:**
- Ensure terminal has focus
- Check if running on supported platform (Unix)
- Verify libc dependency is available

**No visualization:**
- Run with `--features visualization`
- Check Rerun installation
- Verify graphics drivers

**No communication between nodes:**
- Ensure both nodes use same domain
- Check network connectivity
- Verify port availability (7400)

### Platform Support
- **Linux**: Full support (keyboard + visualization)
- **macOS**: Full support (keyboard + visualization)  
- **Windows**: Visualization only (no keyboard control)

## miniROS Philosophy

This turtlebot control system exemplifies miniROS principles:

- **Essential Functionality**: Core robotics control without bloat
- **Minimal Complexity**: Clear, readable implementation
- **Cross-Platform**: Works on major operating systems
- **Real-time Capable**: Low-latency, high-frequency operation
- **Extensible**: Easy to customize and extend
- **Educational**: Clear examples for learning robotics concepts

Perfect foundation for robotics education, research prototyping, and production systems requiring reliable robot control. 