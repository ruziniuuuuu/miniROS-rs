#!/usr/bin/env python3
"""
Multi-Subscriber Node Example

This example demonstrates subscribing to multiple topics with different message types.
Shows advanced subscriber patterns compatible with ROS2.
"""

import mini_ros


class MultiSubscriber(mini_ros.Node):
    """Node that subscribes to multiple topics simultaneously"""

    def __init__(self):
        super().__init__('multi_subscriber')
        
        # Subscribe to string messages
        self.string_subscription = self.create_subscription(
            mini_ros.String,
            'chatter',
            self.string_callback,
            10
        )
        
        # Subscribe to integer messages
        self.int_subscription = self.create_subscription(
            mini_ros.Int32,
            'random_numbers',
            self.int_callback,
            10
        )
        
        # Subscribe to float messages
        self.float_subscription = self.create_subscription(
            mini_ros.Float64,
            'sensor_data',
            self.float_callback,
            10
        )
        
        # Prevent unused variable warnings
        self.string_subscription
        self.int_subscription  
        self.float_subscription
        
        # Statistics
        self.message_counts = {
            'string': 0,
            'int': 0,
            'float': 0
        }
        
        self.get_logger().info('Multi-Subscriber Node started - listening to multiple topics')

    def string_callback(self, msg):
        """Callback for string messages"""
        self.message_counts['string'] += 1
        self.get_logger().info(f'[STRING] Received: "{msg.data}" (total: {self.message_counts["string"]})')

    def int_callback(self, msg):
        """Callback for integer messages"""
        self.message_counts['int'] += 1
        self.get_logger().info(f'[INT32] Received: {msg.data} (total: {self.message_counts["int"]})')

    def float_callback(self, msg):
        """Callback for float messages"""
        self.message_counts['float'] += 1
        self.get_logger().info(f'[FLOAT64] Received: {msg.data:.2f} (total: {self.message_counts["float"]})')

    def print_statistics(self):
        """Print message statistics"""
        total = sum(self.message_counts.values())
        self.get_logger().info(f'Statistics - Total: {total}, String: {self.message_counts["string"]}, '
                             f'Int: {self.message_counts["int"]}, Float: {self.message_counts["float"]}')


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create multi-subscriber node
    multi_subscriber = MultiSubscriber()
    
    try:
        # Spin the node
        mini_ros.spin(multi_subscriber)
        
    except KeyboardInterrupt:
        multi_subscriber.get_logger().info('Shutting down...')
        multi_subscriber.print_statistics()
    
    # Cleanup
    multi_subscriber.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 