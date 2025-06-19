#!/usr/bin/env python3
"""
Minimal Publisher Example for miniROS-rs

This demonstrates the most basic usage - just 20 lines of code!
Compatible with ROS2 rclpy API for easy migration.
"""

import mini_ros
import time

def main():
    # Initialize miniROS (same as rclpy.init())
    mini_ros.init()
    
    # Create node (same as rclpy.create_node())
    node = mini_ros.Node('minimal_publisher')
    
    # Create publisher (same as node.create_publisher())
    pub = node.create_publisher(mini_ros.StringMessage, 'topic', 10)
    
    # Publish messages
    for i in range(5):
        msg = mini_ros.StringMessage()
        msg.data = f'Hello World: {i}'
        pub.publish(msg)
        print(f'Published: {msg.data}')
        time.sleep(1)
    
    # Cleanup (same as rclpy.shutdown())
    mini_ros.shutdown()

if __name__ == '__main__':
    main() 