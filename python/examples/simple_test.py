#!/usr/bin/env python3
"""
Simple Test Example

A simple demonstration that publishes a few messages and exits.
Perfect for testing the basic functionality.
"""

import mini_ros
import time


class SimpleTestNode(mini_ros.Node):
    """Simple test node that publishes and subscribes"""

    def __init__(self):
        super().__init__('simple_test_node')
        
        # Create publisher
        self.publisher = self.create_publisher(mini_ros.String, 'test_messages', 10)
        
        # Create subscription
        self.subscription = self.create_subscription(
            mini_ros.String,
            'test_messages', 
            self.message_callback,
            10
        )
        
        self.count = 0
        self.get_logger().info('Simple test node started')

    def message_callback(self, msg):
        """Handle incoming messages"""
        self.get_logger().info(f'Received: "{msg.data}"')

    def publish_message(self):
        """Publish a test message"""
        msg = mini_ros.String()
        msg.data = f'Test message #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1


def main():
    # Initialize miniROS
    mini_ros.init()
    
    try:
        # Create test node
        node = SimpleTestNode()
        
        # Publish a few messages
        for i in range(5):
            node.publish_message()
            time.sleep(0.5)
        
        node.get_logger().info('Test completed successfully!')
        
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        mini_ros.shutdown()
        print('miniROS shutdown complete')


if __name__ == '__main__':
    main() 