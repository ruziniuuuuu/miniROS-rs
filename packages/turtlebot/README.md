# Turtlebot Package

A comprehensive turtlebot simulation and control package for miniROS demonstrating robotics functionality with minimal complexity.

## Overview

This package provides essential turtlebot functionality including:
- **Controller**: Movement pattern execution with safety features
- **Teleop**: Keyboard-based robot control interface  
- **Simulator**: Physics simulation with optional visualization
- **Python Support**: Cross-language robotics development

## Quick Start

### Using mini_ros CLI (Recommended)

```bash
# List available packages
mini_ros pkg list

# Get package information
mini_ros pkg info turtlebot

# Run individual components
mini_ros run turtlebot controller
mini_ros run turtlebot teleop
mini_ros run turtlebot simulator

# Launch complete systems
mini_ros launch turtlebot simulation
mini_ros launch turtlebot teleop  
mini_ros launch turtlebot full_system
```

### Traditional Cargo Commands

```bash
# Run from package directory
cd packages/turtlebot

# Rust executables
cargo run --bin controller
cargo run --bin teleop
cargo run --bin simulator --features visualization

# Python scripts
python3 scripts/controller.py
```

## Package Structure

```
packages/turtlebot/
├── package.yaml          # Package manifest
├── README.md             # This file
├── src/                  # Rust source code
│   ├── controller.rs     # Advanced movement controller
│   ├── teleop.rs        # Keyboard teleoperation
│   └── simulator.rs     # Physics simulation
├── scripts/             # Python scripts
│   └── controller.py    # Python controller implementation
├── launch/              # Launch configurations
│   ├── simulation.yaml  # Simulator only
│   ├── teleop.yaml     # Teleop only  
│   └── full_system.yaml # Complete system
└── config/              # Configuration files
```

## Executables

### Controller (`controller`)
**Language**: Rust  
**Description**: Advanced turtlebot movement controller

**Features**:
- Multiple movement patterns (forward, circle, square, figure-8)
- Odometry feedback processing
- Safety speed limits and emergency stop
- Real-time status monitoring

**Topics**:
- Publishes: `/cmd_vel` (geometry_msgs/Twist)
- Subscribes: `/odom` (nav_msgs/Odometry)

**Usage**:
```bash
# Via CLI
mini_ros run turtlebot controller

# Direct cargo
cd packages/turtlebot && cargo run --bin controller
```

### Teleop (`teleop`)
**Language**: Rust  
**Description**: Keyboard teleoperation interface

**Features**:
- Cross-platform keyboard input handling
- Configurable speed limits and scaling
- Safety timeout protection
- Real-time status display

**Controls**:
- `W/S`: Forward/Backward
- `A/D`: Turn Left/Right
- `Q/E`: Increase/Decrease speed
- `Space`: Emergency stop
- `X`: Exit

**Usage**:
```bash
# Via CLI  
mini_ros run turtlebot teleop

# Direct cargo
cd packages/turtlebot && cargo run --bin teleop
```

### Simulator (`simulator`)
**Language**: Rust  
**Description**: Physics-based turtlebot simulation

**Features**:
- Realistic differential drive kinematics
- Odometry data generation
- Optional Rerun visualization
- Path tracking and trajectory display

**Requirements**:
- `--features visualization` for Rerun support

**Usage**:
```bash
# Via CLI (with visualization)
mini_ros run turtlebot simulator

# Direct cargo  
cd packages/turtlebot && cargo run --bin simulator --features visualization
```

### Python Controller (`py_controller`)
**Language**: Python  
**Description**: Python implementation of turtlebot controller

**Features**:
- Same movement patterns as Rust version
- Cross-language compatibility demonstration
- Signal handling for graceful shutdown
- Enhanced error handling

**Usage**:
```bash
# Via CLI
mini_ros run turtlebot py_controller

# Direct python
cd packages/turtlebot && python3 scripts/controller.py
```

## Launch Configurations

### Simulation (`simulation.yaml`)
Starts the turtlebot simulator with visualization support.

```bash
mini_ros launch turtlebot simulation
```

**Nodes**:
- `turtlebot_simulator`: Physics simulation and visualization

### Teleop (`teleop.yaml`)  
Starts keyboard teleoperation interface.

```bash
mini_ros launch turtlebot teleop
```

**Nodes**:
- `turtlebot_teleop`: Keyboard control interface

### Full System (`full_system.yaml`)
Complete turtlebot system with simulation and teleop.

```bash
mini_ros launch turtlebot full_system
```

**Nodes**:
- `turtlebot_simulator`: Physics simulation (started first)
- `turtlebot_teleop`: Keyboard control (started after 2s delay)

**Usage Workflow**:
1. Simulator starts and displays robot visualization
2. Teleop interface starts after brief delay
3. Use keyboard controls to drive the simulated robot
4. Observe robot movement in visualization
5. Press Ctrl+C to stop all nodes

## Development

### Adding New Executables

1. **Create the executable** (Rust or Python)
2. **Update package.yaml**:
```yaml
executables:
  new_node:
    name: new_node
    path: "src/new_node.rs"  # or "scripts/new_node.py"
    python: false  # or true for Python
    description: "Description of new node"
    features: []  # Required cargo features
```

3. **Test the executable**:
```bash
mini_ros run turtlebot new_node
```

### Creating Launch Files

Create `.yaml` files in `launch/` directory:

```yaml
name: "My Launch Config"
global_env:
  RUST_LOG: "info"
timeout: null

nodes:
  - name: "node_1"
    package: "turtlebot"
    executable: "controller"
    args: []
    env: {}
    cwd: null
    respawn: false
    delay: 0.0
    python: false
```

## Tips & Best Practices

### Performance
- Use Rust executables for high-frequency control loops
- Use Python for rapid prototyping and scripting
- Enable visualization features only when needed

### Safety  
- All executables include emergency stop functionality
- Speed limits are enforced at multiple levels
- Timeout protection prevents runaway robots

### Debugging
- Set `RUST_LOG=debug` for detailed logging
- Use `mini_ros pkg info turtlebot` to verify package setup
- Check `/cmd_vel` and `/odom` topics with standard ROS tools

### Integration
- Package works with standard ROS2 tools and rviz
- Compatible with real turtlebot hardware (with appropriate drivers)
- Modular design enables easy extension and customization

## Examples

### Basic Movement Demo
```bash
# Start simulator in one terminal
mini_ros launch turtlebot simulation

# Run controller demo in another terminal  
mini_ros run turtlebot controller
```

### Interactive Control
```bash
# Complete interactive system
mini_ros launch turtlebot full_system

# Use WASD keys to control robot
# Observe movement in visualization window
```

### Cross-Language Demo
```bash
# Run Python controller with Rust simulator
mini_ros launch turtlebot simulation  # Terminal 1
mini_ros run turtlebot py_controller  # Terminal 2
```

## Troubleshooting

**Common Issues**:

1. **"Package not found"**: Run `mini_ros pkg list` to verify package discovery
2. **"Executable not found"**: Check `package.yaml` configuration
3. **Visualization not working**: Ensure `--features visualization` is enabled
4. **Permission denied**: Make Python scripts executable with `chmod +x scripts/*.py`

**Getting Help**:
- Check package info: `mini_ros pkg info turtlebot`
- View available commands: `mini_ros --help`
- Enable debug logging: `RUST_LOG=debug mini_ros run turtlebot controller`

---

*Built with miniROS - Maximum robotics performance, minimum complexity* 🤖⚡ 