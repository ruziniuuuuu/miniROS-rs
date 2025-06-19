#!/usr/bin/env python3
"""
Simple Pub/Sub Example - miniROS Python API

This example demonstrates both publisher and subscriber in one node.
"""

import mini_ros
import time
import threading


def callback(msg):
    """Subscriber callback"""
    print(f'Subscriber received: "{msg.data}"')


def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('pubsub_demo')
    logger = node.get_logger()
    
    # Create publisher and subscriber
    publisher = node.create_publisher(mini_ros.StringMessage, 'demo_topic', 10)
    subscription = node.create_subscription(mini_ros.StringMessage, 'demo_topic', callback, 10)
    
    logger.info('Pub/Sub demo started')
    
    # Publishing in a separate thread to allow spinning
    def publish_messages():
        count = 0
        while count < 5:
            msg = mini_ros.StringMessage()
            msg.data = f'Message {count}'
            publisher.publish(msg)
            print(f'Publisher sent: "{msg.data}"')
            count += 1
            time.sleep(2)
    
    # Start publishing thread
    pub_thread = threading.Thread(target=publish_messages)
    pub_thread.daemon = True
    pub_thread.start()
    
    try:
        # Spin for a limited time
        for _ in range(10):
            mini_ros.spin_once(node, timeout_ms=1000)
    except KeyboardInterrupt:
        logger.info('Demo interrupted')
    
    logger.info('Pub/Sub demo completed')
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


if __name__ == '__main__':
    main() 