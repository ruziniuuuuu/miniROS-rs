# Python Bindings

miniROS provides **minimal, fast** Python bindings that are compatible with ROS2 APIs. Get the performance of Rust with the simplicity of Python.

## Installation

```bash
# Requires Rust toolchain
pip install maturin
maturin develop --features python
```

## Core API

### Basic Usage

```python
import mini_ros

# Initialize once
mini_ros.init()

# Create node
node = mini_ros.Node('my_node')

# Publisher
pub = node.create_publisher(mini_ros.String, 'topic', 10)
msg = mini_ros.String()
msg.data = 'Hello miniROS!'
pub.publish(msg)

# Subscriber
def callback(msg):
    print(f'Received: {msg.data}')

sub = node.create_subscription(mini_ros.String, 'topic', callback, 10)

# Run
mini_ros.spin(node)
```

## Message Types

### Built-in Types
```python
# String
msg = mini_ros.String()
msg.data = "text"

# Numbers
int_msg = mini_ros.Int32()
int_msg.data = 42

float_msg = mini_ros.Float64()
float_msg.data = 3.14159

# Boolean
bool_msg = mini_ros.Bool()
bool_msg.data = True
```

### Custom Messages
Python bindings automatically handle any JSON-serializable data:

```python
# Dictionary as message
custom_msg = {
    'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
    'timestamp': time.time()
}
pub.publish(custom_msg)
```

## Examples

### Simple Talker/Listener

**Talker:**
```python
#!/usr/bin/env python3
import mini_ros
import time

mini_ros.init()
node = mini_ros.Node('talker')
pub = node.create_publisher(mini_ros.String, 'chatter', 10)

msg = mini_ros.String()
count = 0

while mini_ros.ok():
    msg.data = f'Hello World {count}'
    pub.publish(msg)
    node.get_logger().info(f'Publishing: {msg.data}')
    count += 1
    time.sleep(1)
```

**Listener:**
```python
#!/usr/bin/env python3
import mini_ros

def callback(msg):
    print(f'I heard: {msg.data}')

mini_ros.init()
node = mini_ros.Node('listener')
sub = node.create_subscription(mini_ros.String, 'chatter', callback, 10)
mini_ros.spin(node)
```

### Image Processing

```python
#!/usr/bin/env python3
import mini_ros
import cv2
import numpy as np
import json
import time

mini_ros.init()
node = mini_ros.Node('image_publisher')
pub = node.create_publisher(mini_ros.String, 'camera/image_raw', 10)

# Generate synthetic image
height, width = 480, 640
frame_count = 0

while mini_ros.ok():
    # Create test pattern
    image = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(image, f'Frame {frame_count}', (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Create ROS-like image message
    image_msg = {
        'header': {
            'stamp': time.time(),
            'frame_id': 'camera_frame'
        },
        'height': height,
        'width': width,
        'encoding': 'bgr8',
        'is_bigendian': False,
        'step': width * 3,
        'data_size': image.size
    }
    
    # Publish as JSON
    msg = mini_ros.String()
    msg.data = json.dumps(image_msg)
    pub.publish(msg)
    
    frame_count += 1
    time.sleep(0.1)  # 10 FPS
```

### Robot Visualization

```python
#!/usr/bin/env python3
import mini_ros
import rerun as rr
import numpy as np
import time
import json

# Initialize rerun for 3D visualization
rr.init("miniROS Robot Visualization")
rr.spawn()

mini_ros.init()
node = mini_ros.Node('robot_visualizer')

# Publishers for different data types
robot_pub = node.create_publisher(mini_ros.String, 'robot_state', 10)
odom_pub = node.create_publisher(mini_ros.String, 'odometry', 10)
laser_pub = node.create_publisher(mini_ros.String, 'laser_scan', 10)

# Simulation state
position = np.array([0.0, 0.0, 0.0])
velocity = np.array([0.5, 0.2, 0.0])
yaw = 0.0
yaw_rate = 0.3

while mini_ros.ok():
    # Update robot state
    position += velocity * 0.1
    yaw += yaw_rate * 0.1
    
    # Robot state message
    robot_state = {
        'position': position.tolist(),
        'orientation': [0, 0, np.sin(yaw/2), np.cos(yaw/2)],
        'joint_positions': [np.sin(time.time() + i) for i in range(4)],
        'timestamp': time.time()
    }
    
    # Odometry message
    odometry = {
        'position': position.tolist(),
        'velocity': velocity.tolist(),
        'yaw': yaw,
        'yaw_rate': yaw_rate,
        'timestamp': time.time()
    }
    
    # Laser scan (360 degree)
    ranges = [np.random.uniform(0.5, 10.0) for _ in range(360)]
    laser_scan = {
        'angle_min': -np.pi,
        'angle_max': np.pi,
        'angle_increment': 2 * np.pi / 360,
        'ranges': ranges,
        'timestamp': time.time()
    }
    
    # Publish all data
    robot_pub.publish(mini_ros.String(data=json.dumps(robot_state)))
    odom_pub.publish(mini_ros.String(data=json.dumps(odometry)))
    laser_pub.publish(mini_ros.String(data=json.dumps(laser_scan)))
    
    # Log to rerun for 3D visualization
    rr.log("robot/position", rr.Points3D([position]))
    rr.log("robot/trajectory", rr.LineStrips3D([position.reshape(1, -1)]))
    
    node.get_logger().info(f'Robot pos: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})')
    time.sleep(0.1)
```

## Utilities

```python
# Check if system is running
if mini_ros.ok():
    # Do work
    pass

# Process callbacks once
mini_ros.spin_once(node, timeout_ms=100)

# Shutdown cleanly
mini_ros.shutdown()
```

## Logging

```python
logger = node.get_logger()
logger.info("Information message")
logger.warn("Warning message") 
logger.error("Error message")
```

## Performance Tips

1. **Minimize message size** - Use efficient data structures
2. **Batch operations** - Group related data in single messages
3. **Use appropriate QoS** - Default settings work for most cases
4. **Avoid blocking operations** - Keep callbacks fast

## ROS2 Compatibility

miniROS Python API is designed to be familiar to ROS2 users:

| miniROS | ROS2 equivalent |
|---------|-----------------|
| `mini_ros.init()` | `rclpy.init()` |
| `mini_ros.Node()` | `rclpy.create_node()` |
| `create_publisher()` | `create_publisher()` |
| `create_subscription()` | `create_subscription()` |
| `mini_ros.spin()` | `rclpy.spin()` |

The key difference: **miniROS is simpler and faster**.

---

*Python bindings: ROS2 familiarity with Rust performance* 