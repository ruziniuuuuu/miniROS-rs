#!/usr/bin/env python3
"""
Listener Node Example

This example demonstrates a basic subscriber node using miniROS Python API.
The syntax is identical to ROS2 rclpy for easy migration.
"""

import mini_ros


class MinimalSubscriber(mini_ros.Node):
    """
    Minimal subscriber node class
    
    This demonstrates object-oriented node design compatible with ROS2.
    """

    def __init__(self):
        super().__init__('listener')
        
        # Create subscription
        self.subscription = self.create_subscription(
            mini_ros.String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """Callback function for incoming messages"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create subscriber node
    minimal_subscriber = MinimalSubscriber()
    
    # Spin the node
    mini_ros.spin(minimal_subscriber)
    
    # Cleanup
    minimal_subscriber.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 