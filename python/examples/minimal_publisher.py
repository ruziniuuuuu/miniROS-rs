#!/usr/bin/env python3
"""
Minimal Publisher Example - miniROS Python API

Simplest possible publisher example.
"""

import mini_ros
import time


def main():
    mini_ros.init()
    
    node = mini_ros.Node('minimal_publisher')
    pub = node.create_publisher(mini_ros.StringMessage, 'topic', 10)
    
    count = 0
    while mini_ros.ok():
        msg = mini_ros.StringMessage()
        msg.data = f'Hello {count}'
        pub.publish(msg)
        print(f'Published: {msg.data}')
        count += 1
        time.sleep(1)


if __name__ == '__main__':
    main() 