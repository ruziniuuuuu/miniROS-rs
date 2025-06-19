#!/usr/bin/env python3
"""
Simple Performance Demo (Python)

This example demonstrates basic performance measurement in MiniROS Python.
"""

import mini_ros
import time

def latency_test():
    """Simple latency test"""
    mini_ros.init()
    node = mini_ros.Node("py_latency_test")
    
    pub = node.create_publisher(mini_ros.StringMessage, "test/latency", 10)
    
    latencies = []
    for i in range(10):
        start = time.time()
        msg = mini_ros.StringMessage()
        msg.data = f"Test message {i}"
        pub.publish(msg)
        latency = (time.time() - start) * 1_000_000  # microseconds
        latencies.append(latency)
        time.sleep(0.1)
    
    node.destroy_node()
    mini_ros.shutdown()
    
    avg_latency = sum(latencies) / len(latencies)
    return avg_latency

def throughput_test():
    """Simple throughput test"""
    mini_ros.init()
    node = mini_ros.Node("py_throughput_test")
    
    pub = node.create_publisher(mini_ros.StringMessage, "test/throughput", 10)
    
    start_time = time.time()
    messages_sent = 0
    
    # Send messages for 2 seconds
    end_time = start_time + 2.0
    while time.time() < end_time:
        msg = mini_ros.StringMessage()
        msg.data = f"Message {messages_sent}"
        pub.publish(msg)
        messages_sent += 1
    
    duration = time.time() - start_time
    
    node.destroy_node()
    mini_ros.shutdown()
    
    throughput = messages_sent / duration
    return throughput

def main():
    print("=== MiniROS Python Performance Demo ===")
    
    # 1. Latency Test
    print("\n1. Latency Test:")
    latency = latency_test()
    print(f"Average latency: {latency:.1f} μs")
    
    # 2. Throughput Test  
    print("\n2. Throughput Test:")
    throughput = throughput_test()
    print(f"Throughput: {throughput:.0f} messages/sec")
    
    print("\nPerformance demo completed successfully!")

if __name__ == "__main__":
    main() 