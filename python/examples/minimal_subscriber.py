#!/usr/bin/env python3
"""
Minimal Subscriber Example for miniROS-rs

This demonstrates the most basic subscriber usage.
Compatible with ROS2 rclpy API for easy migration.
"""

import mini_ros

def listener_callback(msg):
    """Callback function for received messages"""
    print(f'Received: {msg.data}')

def main():
    # Initialize miniROS (same as rclpy.init())
    mini_ros.init()
    
    # Create node (same as rclpy.create_node())
    node = mini_ros.Node('minimal_subscriber')
    
    # Create subscriber (same as node.create_subscription())
    sub = node.create_subscription(
        mini_ros.StringMessage,
        'topic',
        listener_callback,
        10
    )
    
    print('Listening for messages...')
    
    # Spin node (same as rclpy.spin())
    mini_ros.spin(node)
    
    # Cleanup (same as rclpy.shutdown())
    mini_ros.shutdown()

if __name__ == '__main__':
    main() 