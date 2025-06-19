#!/usr/bin/env python3
"""
Minimal Demo (Python)

This example demonstrates basic pub/sub functionality in MiniROS Python.
"""

import mini_ros
import time

def main():
    print("=== MiniROS Python Basic Demo ===")
    
    # Initialize MiniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node("py_basic_demo")
    print(f"Created node: {node.get_name()}")
    
    # Create publisher for status messages
    status_pub = node.create_publisher(mini_ros.StringMessage, "robot/status", 10)
    
    # Basic pub/sub demo
    for i in range(5):
        msg = mini_ros.StringMessage()
        msg.data = f"Demo message {i}"
        status_pub.publish(msg)
        print(f"Published: {msg.data}")
        time.sleep(1)
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()
    print("Demo completed successfully!")

if __name__ == "__main__":
    main() 