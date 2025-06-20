# miniROS-rs Python Bindings

High-performance ROS2-compatible Python bindings for miniROS-rs.

## Features

- **ROS2 Compatible**: Drop-in replacement for rclpy with same API
- **High Performance**: Rust-powered backend with Python convenience
- **Complete Message Support**: Full std_msgs, geometry_msgs, nav_msgs, sensor_msgs packages
- **Type Safety**: Runtime validation and type checking
- **Cross-Language**: Seamless Rust ↔ Python interoperability
- **Zero Dependencies**: Pure Python API, no ROS2 installation required

## Installation

### From Source
```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Build Python bindings
cd python/
python -m pip install -e .
```

### Using pre-built wheels (coming soon)
```bash
pip install mini-ros-python
```

## Quick Start

### Basic Publishing and Subscribing

```python
import mini_ros
import time

# Initialize miniROS
mini_ros.init()

# Create node
node = mini_ros.Node('my_node')

# Create publisher using std_msgs
pub = node.create_publisher(mini_ros.std_msgs.String, 'chatter', 10)

# Create subscriber
def callback(msg):
    print(f"Received: {msg.data}")

sub = node.create_subscription(mini_ros.std_msgs.String, 'chatter', callback, 10)

# Publish messages
for i in range(10):
    msg = mini_ros.std_msgs.String()
    msg.data = f"Hello World {i}"
    pub.publish(msg)
    time.sleep(1)

# Cleanup
node.destroy_node()
mini_ros.shutdown()
```

## ROS2 Message Packages

miniROS-rs provides full ROS2 message compatibility with the following packages:

### std_msgs - Basic Data Types
```python
# String message
string_msg = mini_ros.std_msgs.String()
string_msg.data = "Hello World"

# Numeric messages
int32_msg = mini_ros.std_msgs.Int32()
int32_msg.data = 42

float64_msg = mini_ros.std_msgs.Float64()
float64_msg.data = 3.14159

# Boolean message
bool_msg = mini_ros.std_msgs.Bool()
bool_msg.data = True

# Header with timestamp
header_msg = mini_ros.std_msgs.Header()
header_msg.frame_id = "base_link"
header_msg.stamp_sec = int(time.time())
header_msg.stamp_nanosec = int((time.time() % 1) * 1e9)
```

### geometry_msgs - Geometric Data Types
```python
# Point in 3D space
point_msg = mini_ros.geometry_msgs.Point()
point_msg.x = 1.0
point_msg.y = 2.0
point_msg.z = 3.0

# Vector3 for directions/velocities
vector_msg = mini_ros.geometry_msgs.Vector3()
vector_msg.x = 0.5
vector_msg.y = -0.3
vector_msg.z = 0.8

# Quaternion for orientation
quat_msg = mini_ros.geometry_msgs.Quaternion()
quat_msg.x = 0.0
quat_msg.y = 0.0
quat_msg.z = 0.707  # 90 degrees around Z-axis
quat_msg.w = 0.707
quat_msg.normalize()  # Ensure unit quaternion

# Pose (position + orientation)
pose_msg = mini_ros.geometry_msgs.Pose()
pose_msg.position = point_msg
pose_msg.orientation = quat_msg

# PoseStamped (pose with timestamp)
pose_stamped_msg = mini_ros.geometry_msgs.PoseStamped()
pose_stamped_msg.header.frame_id = "map"
pose_stamped_msg.pose = pose_msg

# Twist (linear and angular velocity)
twist_msg = mini_ros.geometry_msgs.Twist()
twist_msg.linear.x = 1.0   # Forward velocity
twist_msg.angular.z = 0.5  # Turning velocity

# Validate message
if twist_msg.validate():
    print("Twist message is within safety limits")
```

### nav_msgs - Navigation Data Types
```python
# Odometry (robot state)
odom_msg = mini_ros.nav_msgs.Odometry()
odom_msg.header.frame_id = "odom"
odom_msg.child_frame_id = "base_link"

# Set robot pose
odom_msg.pose.position.x = 5.0
odom_msg.pose.position.y = 3.0
odom_msg.pose.orientation.w = 1.0

# Set robot velocity
odom_msg.twist.linear.x = 0.5
odom_msg.twist.angular.z = 0.1

# Convenience methods
yaw = odom_msg.get_yaw()  # Extract yaw angle
odom_msg.set_pose_2d(10.0, 5.0, math.pi/4)  # Set 2D pose
```

### sensor_msgs - Sensor Data Types
```python
# Available message types (defined in package.yaml):
# - LaserScan: Laser range finder data
# - PointCloud2: 3D point cloud data  
# - Imu: Inertial measurement unit data
# - Image: Camera image data
```

## Multiple API Styles

miniROS-rs supports multiple ways to access message types for maximum compatibility:

```python
# Method 1: Package-based access (recommended for new code)
msg1 = mini_ros.std_msgs.String()

# Method 2: Direct access (legacy compatibility)
msg2 = mini_ros.String()

# Method 3: Legacy naming (backward compatibility)
msg3 = mini_ros.StringMessage()

# All three create the same message type!
assert type(msg1) == type(msg2) == type(msg3)
```

## Advanced Features

### Message Validation
```python
# Create a twist message
twist_msg = mini_ros.geometry_msgs.Twist()
twist_msg.linear.x = 100.0  # Very high speed
twist_msg.angular.z = 10.0  # Very high rotation

# Validate message (checks safety limits)
if not twist_msg.validate():
    print("Warning: Twist command exceeds safety limits!")
```

### Convenience Methods
```python
# Odometry convenience methods
odom_msg = mini_ros.nav_msgs.Odometry()

# Set 2D pose easily
odom_msg.set_pose_2d(x=5.0, y=3.0, yaw=math.pi/2)

# Extract yaw angle
current_yaw = odom_msg.get_yaw()

# Twist convenience methods
twist_msg = mini_ros.geometry_msgs.Twist()
twist_msg.set_linear_xyz(1.0, 0.0, 0.0)   # Set linear velocity
twist_msg.set_angular_xyz(0.0, 0.0, 0.5)  # Set angular velocity
```

### Type Safety and Schemas
```python
# All message types implement schema information
string_msg = mini_ros.std_msgs.String()
print(f"Message type: {string_msg.message_type()}")  # "std_msgs/String"
print(f"Schema: {string_msg.schema()}")              # Field definitions

# Runtime type checking
pose_msg = mini_ros.geometry_msgs.Pose()
if pose_msg.validate():
    print("Pose message is valid")
```

## Performance

miniROS-rs Python bindings provide excellent performance:

- **Fast serialization**: Binary encoding with bincode
- **Zero-copy**: Direct memory access where possible  
- **Low latency**: Rust backend eliminates Python overhead
- **High throughput**: Optimized for real-time robotics applications

## Examples

### Basic Talker/Listener
```bash
# Terminal 1: Start talker
python examples/talker.py

# Terminal 2: Start listener  
python examples/listener.py
```

### ROS2 Message Demo
```bash
# Comprehensive demo of all message types
python examples/ros2_message_demo.py
```

### Robot Control Example
```python
import mini_ros
import time

mini_ros.init()
node = mini_ros.Node('robot_controller')

# Create publishers
cmd_vel_pub = node.create_publisher(mini_ros.geometry_msgs.Twist, 'cmd_vel', 10)
odom_pub = node.create_publisher(mini_ros.nav_msgs.Odometry, 'odom', 10)

# Create subscriber for pose updates
def pose_callback(msg):
    print(f"Robot at: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

pose_sub = node.create_subscription(
    mini_ros.geometry_msgs.PoseStamped, 'robot_pose', pose_callback, 10
)

# Control loop
for i in range(100):
    # Send velocity command
    twist_msg = mini_ros.geometry_msgs.Twist()
    twist_msg.linear.x = 0.5  # Move forward
    twist_msg.angular.z = 0.1  # Turn slightly
    cmd_vel_pub.publish(twist_msg)
    
    # Publish odometry
    odom_msg = mini_ros.nav_msgs.Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    odom_msg.set_pose_2d(i * 0.1, 0.0, i * 0.01)  # Simulate movement
    odom_pub.publish(odom_msg)
    
    time.sleep(0.1)

node.destroy_node()
mini_ros.shutdown()
```

## Visualization Support

```python
# Create visualization client
viz = mini_ros.VisualizationClient("robot_demo", spawn_viewer=True)

# Log robot trajectory
points = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]
viz.log_points("trajectory", points)

# Log robot pose
viz.log_transform("robot", [1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0])

# Log scalar data
viz.log_scalar("speed", 1.5)
```

## Migration from ROS2

miniROS-rs provides a drop-in replacement for common rclpy patterns:

```python
# ROS2 rclpy code:
# import rclpy
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist

# miniROS-rs equivalent:
import mini_ros
# Message types are built-in: mini_ros.std_msgs.String, mini_ros.geometry_msgs.Twist

# Same API patterns work:
mini_ros.init()  # instead of rclpy.init()
node = mini_ros.Node('my_node')  # same as rclpy
# ... rest of the API is identical
```

## Troubleshooting

### Import Errors
If you get import errors, ensure the Rust bindings are built:
```bash
cd python/
python -m pip install -e .
```

### Performance Issues
For maximum performance:
- Use binary message serialization (enabled by default)
- Avoid excessive message validation calls
- Use batch publishing for high-throughput scenarios

### Compatibility Issues
- Use package-based message access for new code: `mini_ros.std_msgs.String()`
- Legacy access still works: `mini_ros.StringMessage()`
- All ROS2 message field names and types are identical

## Documentation

- [API Reference](https://docs.rs/mini-ros)
- [Message Type Reference](../packages/)
- [Rust Documentation](https://docs.rs/mini-ros/latest/mini_ros/)

## Contributing

See the main project [CONTRIBUTING.md](../CONTRIBUTING.md) for development guidelines.

## License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details. 