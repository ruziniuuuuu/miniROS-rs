#!/usr/bin/env python3
"""
Minimal Visualization Example for miniROS

This example demonstrates the core visualization capabilities using Rerun.
Shows how to log different data types for robot visualization.

Run with: python minimal_visualization.py
"""

import mini_ros
import time
import math

def main():
    # Initialize miniROS
    mini_ros.init()
    
    # Create visualization client
    # Set spawn_viewer=True to open Rerun viewer automatically
    viz = mini_ros.PyVisualizationClient("miniROS_demo", spawn_viewer=False)
    
    print("🎯 miniROS Minimal Visualization Demo")
    print("Logging data to Rerun (use rerun viewer to visualize)")
    
    # Simulate robot movement and sensor data
    for i in range(100):
        timestamp_ns = int(time.time() * 1e9)
        
        # Set timeline
        viz.set_time("sim_time", timestamp_ns)
        
        # 1. Log robot position (moving in circle)
        angle = i * 0.1
        x = 2.0 * math.cos(angle)
        y = 2.0 * math.sin(angle)
        z = 0.0
        
        # Log robot pose as transform
        position = [x, y, z]
        rotation = [0.0, 0.0, math.sin(angle/2), math.cos(angle/2)]  # Quaternion
        viz.log_transform("/robot/base_link", position, rotation)
        
        # 2. Log scalar sensors
        battery_level = 100.0 - (i * 0.5)  # Decreasing battery
        viz.log_scalar("/robot/battery", battery_level)
        
        velocity = 1.0 + 0.5 * math.sin(angle * 3)  # Varying velocity
        viz.log_scalar("/robot/velocity", velocity)
        
        # 3. Log point cloud (simple pattern)
        points = []
        for j in range(10):
            px = x + 0.5 * math.cos(j * 0.6)
            py = y + 0.5 * math.sin(j * 0.6)
            pz = 0.1 * j
            points.append([px, py, pz])
        
        viz.log_points("/robot/lidar/points", points)
        
        # 4. Log status text
        status = f"Robot step {i}: battery {battery_level:.1f}%, velocity {velocity:.2f}"
        viz.log_text("/robot/status", status)
        
        print(f"Step {i:3d}: pos=({x:.2f}, {y:.2f}), battery={battery_level:.1f}%")
        
        time.sleep(0.1)  # 10 Hz update rate
    
    print("\n✅ Visualization demo completed!")
    print("💡 To view the data, run: rerun --memory")
    
    # Cleanup
    mini_ros.shutdown()

if __name__ == "__main__":
    main() 