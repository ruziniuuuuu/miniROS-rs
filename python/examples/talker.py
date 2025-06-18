#!/usr/bin/env python3
"""
Talker Node Example

This example demonstrates a basic publisher node using miniROS Python API.
The syntax is identical to ROS2 rclpy for easy migration.
"""

import mini_ros
import time


def main():
    # Initialize miniROS (equivalent to rclpy.init())
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('talker')
    
    # Create publisher 
    publisher = node.create_publisher(mini_ros.String, 'chatter', 10)
    
    # Create message
    msg = mini_ros.String()
    
    # Publish loop
    i = 0
    while mini_ros.ok():
        msg.data = f'Hello World: {i}'
        publisher.publish(msg)
        node.get_logger().info(f'Publishing: "{msg.data}"')
        i += 1
        time.sleep(1)
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 