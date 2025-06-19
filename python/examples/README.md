# miniROS Python Examples

Simple examples demonstrating miniROS Python API usage, similar to ROS2 rclpy.

## Examples

### Basic Pub/Sub
- `minimal_publisher.py` - Simplest publisher example
- `minimal_subscriber.py` - Simplest subscriber example  
- `talker.py` - Publisher with logging
- `listener.py` - Subscriber with logging
- `simple_pubsub.py` - Combined pub/sub in one node

### Parameters
- `simple_param.py` - Basic parameter usage demonstration

## Usage

```bash
# Install miniROS Python package
pip install mini-ros

# Run examples
python minimal_publisher.py
python minimal_subscriber.py
```

## API Style

The miniROS Python API follows ROS2 rclpy conventions:

```python
import mini_ros

# Initialize
mini_ros.init()

# Create node
node = mini_ros.Node('my_node')

# Create publisher
pub = node.create_publisher(mini_ros.StringMessage, 'topic', 10)

# Create subscriber  
sub = node.create_subscription(mini_ros.StringMessage, 'topic', callback, 10)

# Publish message
msg = mini_ros.StringMessage()
msg.data = 'Hello World'
pub.publish(msg)

# Spin
mini_ros.spin(node)

# Cleanup
node.destroy_node()
mini_ros.shutdown()
``` 