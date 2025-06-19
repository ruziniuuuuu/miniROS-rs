#!/usr/bin/env python3
"""
Simple Pub/Sub Demo for miniROS-rs

This single-file example demonstrates both publishing and subscribing
in the most minimal way possible - under 30 lines of code!
Perfect example of the "mini" philosophy.
"""

import mini_ros
import time
import threading

def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('simple_demo')
    
    # Create publisher and subscriber for same topic
    pub = node.create_publisher(mini_ros.StringMessage, 'hello', 10)
    
    def on_message(msg):
        print(f'📨 Received: {msg.data}')
    
    sub = node.create_subscription(mini_ros.StringMessage, 'hello', on_message, 10)
    
    print('🚀 Starting miniROS pub/sub demo...')
    
    # Publish some messages
    for i in range(3):
        msg = mini_ros.StringMessage()
        msg.data = f'Hello miniROS #{i+1}'
        pub.publish(msg)
        print(f'📤 Published: {msg.data}')
        time.sleep(0.5)
    
    print('✅ Demo completed!')
    mini_ros.shutdown()

if __name__ == '__main__':
    main() 