#!/usr/bin/env python3
"""
Listener Example - miniROS Python API

This example demonstrates basic subscription functionality.
Similar to ROS2 rclpy tutorials.
"""

import mini_ros


def listener_callback(msg):
    """Callback function for received messages"""
    print(f'I heard: "{msg.data}"')


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('listener')
    
    # Create subscription
    subscription = node.create_subscription(
        mini_ros.StringMessage,
        'chatter',
        listener_callback,
        10
    )
    
    # Get logger
    logger = node.get_logger()
    logger.info('Listener node started, waiting for messages...')
    
    try:
        # Spin to process callbacks
        mini_ros.spin(node)
    except KeyboardInterrupt:
        logger.info('Listener interrupted')
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 