#!/usr/bin/env python3
"""
Talker Example - miniROS Python API

This example demonstrates basic publishing functionality using the new ROS2-compatible message types.
Shows compatibility with both old and new message APIs.
Similar to ROS2 rclpy tutorials.
"""

import mini_ros
import time


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('talker')
    
    # Get logger
    logger = node.get_logger()
    logger.info('Talker node started with ROS2 message support')
    
    # Method 1: Using new std_msgs package (recommended)
    publisher1 = node.create_publisher(mini_ros.std_msgs.String, 'chatter', 10)
    
    # Method 2: Using legacy direct access (backward compatibility)
    publisher2 = node.create_publisher(mini_ros.StringMessage, 'chatter_legacy', 10)
    
    # Method 3: Using new geometry_msgs for more complex data
    pose_publisher = node.create_publisher(mini_ros.geometry_msgs.Pose, 'robot_pose', 10)
    twist_publisher = node.create_publisher(mini_ros.geometry_msgs.Twist, 'cmd_vel', 10)
    
    logger.info('Created publishers for different message types:')
    logger.info('  - std_msgs/String on topic "chatter"')
    logger.info('  - StringMessage on topic "chatter_legacy" (legacy)')  
    logger.info('  - geometry_msgs/Pose on topic "robot_pose"')
    logger.info('  - geometry_msgs/Twist on topic "cmd_vel"')
    
    # Publishing loop
    count = 0
    try:
        while mini_ros.ok():
            # Method 1: New std_msgs API
            msg1 = mini_ros.std_msgs.String()
            msg1.data = f'Hello World from std_msgs: {count}'
            publisher1.publish(msg1)
            logger.info(f'Publishing (std_msgs): "{msg1.data}"')
            
            # Method 2: Legacy API (still works)
            msg2 = mini_ros.StringMessage()
            msg2.data = f'Hello World from legacy: {count}'
            publisher2.publish(msg2)
            logger.info(f'Publishing (legacy): "{msg2.data}"')
            
            # Method 3: geometry_msgs examples
            
            # Publish a pose (robot position and orientation)
            pose_msg = mini_ros.geometry_msgs.Pose()
            pose_msg.position.x = float(count)
            pose_msg.position.y = float(count * 0.5)
            pose_msg.position.z = 0.0
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0  # No rotation
            pose_publisher.publish(pose_msg)
            logger.info(f'Publishing pose: position=({pose_msg.position.x:.1f}, {pose_msg.position.y:.1f}, {pose_msg.position.z:.1f})')
            
            # Publish a twist command (velocity)
            twist_msg = mini_ros.geometry_msgs.Twist()
            twist_msg.linear.x = 1.0  # Move forward at 1 m/s
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.1 * (count % 10 - 5)  # Oscillating turn
            twist_publisher.publish(twist_msg)
            logger.info(f'Publishing twist: linear.x={twist_msg.linear.x:.1f}, angular.z={twist_msg.angular.z:.2f}')
            
            # Show message validation (if available)
            if hasattr(twist_msg, 'validate'):
                is_valid = twist_msg.validate()
                if not is_valid:
                    logger.warn('Generated twist command failed validation!')
            
            count += 1
            time.sleep(2.0)  # Slower rate to see all messages
            
    except KeyboardInterrupt:
        logger.info('Talker interrupted')
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()
    logger.info('Talker shutdown complete')


if __name__ == '__main__':
    main() 