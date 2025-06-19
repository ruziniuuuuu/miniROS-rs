#!/usr/bin/env python3
"""
Minimal Subscriber Example - miniROS Python API

Simplest possible subscriber example.
"""

import mini_ros


def callback(msg):
    print(f'Received: {msg.data}')


def main():
    mini_ros.init()
    
    node = mini_ros.Node('minimal_subscriber')
    sub = node.create_subscription(mini_ros.StringMessage, 'topic', callback, 10)
    
    print('Waiting for messages...')
    mini_ros.spin(node)


if __name__ == '__main__':
    main() 