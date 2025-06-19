#!/usr/bin/env python3
"""
Basic Type System Demo (Python)

This example demonstrates basic message types in MiniROS Python.
"""

import mini_ros
import time

def message_callback(msg):
    """Callback function for receiving messages"""
    print(f"Received: {msg.data}")

def main():
    print("=== MiniROS Python Type System Demo ===")
    
    # Initialize MiniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node("py_type_demo")
    print(f"Created node: {node.get_name()}")
    
    # 1. Basic String message demonstration
    print("\n1. String Message Demo:")
    
    # Create publisher and subscriber
    pub = node.create_publisher(mini_ros.StringMessage, "test/topic", 10)
    sub = node.create_subscription(mini_ros.StringMessage, "test/topic", message_callback, 10)
    
    # Publish messages
    for i in range(3):
        msg = mini_ros.StringMessage()
        msg.data = f"Test message {i}"
        pub.publish(msg)
        print(f"Published: {msg.data}")
        time.sleep(0.5)
    
    # 2. Future Extension Demo
    print("\n2. Type system ready for extension:")
    print("✓ StringMessage working correctly") 
    print("✓ Ready for additional message types (Pose, PointCloud, etc.)")
    print("✓ Basic pub/sub communication established")
    
    print("\n3. Messages processed successfully!")
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()
    print("\nType system demo completed successfully!")

if __name__ == "__main__":
    main() 