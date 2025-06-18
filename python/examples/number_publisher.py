#!/usr/bin/env python3
"""
Number Publisher Node Example

This example demonstrates publishing numeric data using miniROS Python API.
Shows how to use different message types (Int32, Float64).
"""

import mini_ros
import time
import math


class NumberPublisher(mini_ros.Node):
    """Number publisher node that sends various numeric data"""

    def __init__(self):
        super().__init__('number_publisher')
        
        # Create publishers for different message types
        self.int_publisher = self.create_publisher(mini_ros.Int32, 'random_numbers', 10)
        self.float_publisher = self.create_publisher(mini_ros.Float64, 'sensor_data', 10)
        
        # Timer counter
        self.count = 0
        
        self.get_logger().info('Number Publisher Node started')

    def publish_numbers(self):
        """Publish numeric data"""
        # Publish integer counter
        int_msg = mini_ros.Int32()
        int_msg.data = self.count
        self.int_publisher.publish(int_msg)
        
        # Publish sine wave as sensor data
        float_msg = mini_ros.Float64()
        float_msg.data = math.sin(self.count * 0.1) * 100.0  # Sine wave scaled to +/-100
        self.float_publisher.publish(float_msg)
        
        self.get_logger().info(f'Published: count={int_msg.data}, sensor={float_msg.data:.2f}')
        
        self.count += 1


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create publisher node
    publisher_node = NumberPublisher()
    
    try:
        # Publishing loop
        while mini_ros.ok():
            publisher_node.publish_numbers()
            time.sleep(0.5)  # 2 Hz publishing rate
            
    except KeyboardInterrupt:
        publisher_node.get_logger().info('Shutting down...')
    
    # Cleanup
    publisher_node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 