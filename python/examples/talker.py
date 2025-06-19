#!/usr/bin/env python3
"""
Talker Example - miniROS Python API

This example demonstrates basic publishing functionality.
Similar to ROS2 rclpy tutorials.
"""

import mini_ros
import time


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('talker')
    
    # Create publisher
    publisher = node.create_publisher(mini_ros.StringMessage, 'chatter', 10)
    
    # Get logger
    logger = node.get_logger()
    logger.info('Talker node started')
    
    # Publishing loop
    count = 0
    try:
        while mini_ros.ok():
            # Create and publish message
            msg = mini_ros.StringMessage()
            msg.data = f'Hello World: {count}'
            
            publisher.publish(msg)
            logger.info(f'Publishing: "{msg.data}"')
            
            count += 1
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        logger.info('Talker interrupted')
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 