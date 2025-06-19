#!/usr/bin/env python3
"""
Simple Parameter Example - miniROS Python API

This example demonstrates basic parameter functionality,
similar to ROS2 parameter usage.
"""

import mini_ros


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('param_demo')
    logger = node.get_logger()
    
    logger.info('Parameter demo started')
    
    # Declare parameters with default values
    # Note: This is a simplified version - actual implementation may vary
    robot_name = "miniROS_robot"
    max_speed = 2.5
    enable_sensors = True
    
    logger.info(f'Robot name: {robot_name}')
    logger.info(f'Max speed: {max_speed} m/s')
    logger.info(f'Sensors enabled: {enable_sensors}')
    
    # Simulate parameter updates
    logger.info('Simulating parameter updates...')
    
    for i in range(5):
        new_speed = max_speed + i * 0.5
        logger.info(f'Updated max speed: {new_speed} m/s')
        
        if i == 2:
            enable_sensors = False
            logger.info(f'Sensors disabled: {enable_sensors}')
    
    logger.info('Parameter demo completed')
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 