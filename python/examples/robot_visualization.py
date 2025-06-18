#!/usr/bin/env python3
"""
Robot Visualization Example for miniROS Python API

This example demonstrates:
- 3D robot visualization using rerun
- Publishing robot state and sensor data
- Real-time 3D plotting and animation
- Integration of multiple data streams
"""

import mini_ros
import numpy as np
import rerun as rr
import time
import math
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class RobotState:
    """Robot state message"""
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]  # quaternion
    joint_positions: List[float]
    timestamp: float

@dataclass
class LaserScan:
    """Laser scan message"""
    angle_min: float
    angle_max: float
    angle_increment: float
    ranges: List[float]
    timestamp: float

class RobotVisualizer(mini_ros.Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        # Publishers
        self.robot_state_pub = self.create_publisher(mini_ros.String, 'robot_state', 10)
        self.laser_pub = self.create_publisher(mini_ros.String, 'laser_scan', 10)
        self.odom_pub = self.create_publisher(mini_ros.String, 'odometry', 10)
        
        # Initialize rerun
        rr.init("miniROS_Robot_Visualization", spawn=True)
        
        # Robot parameters
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_vel = np.array([0.5, 0.2, 0.0])  # m/s
        self.robot_yaw = 0.0
        self.robot_yaw_rate = 0.3  # rad/s
        
        # Joint parameters for articulated robot
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # 4-DOF arm
        self.joint_velocities = [0.2, 0.15, 0.25, 0.1]
        
        # Laser scanner parameters
        self.laser_range = 10.0
        self.laser_angles = np.linspace(-np.pi, np.pi, 360)
        
        # Simulation time
        self.sim_time = 0.0
        self.dt = 0.1  # 10 Hz
        
        self.get_logger().info("Robot visualizer initialized")
    
    def update_robot_state(self):
        """Update robot simulation state"""
        # Update robot position (circular motion)
        self.robot_pos[0] = 3.0 * np.cos(self.sim_time * 0.2)
        self.robot_pos[1] = 2.0 * np.sin(self.sim_time * 0.2)
        self.robot_pos[2] = 0.5 + 0.2 * np.sin(self.sim_time * 0.5)
        
        # Update robot orientation
        self.robot_yaw = self.sim_time * 0.3
        
        # Update joint positions (sinusoidal motion)
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = np.sin(self.sim_time * self.joint_velocities[i] + i * np.pi/2)
        
        self.sim_time += self.dt
    
    def generate_laser_scan(self) -> LaserScan:
        """Generate simulated laser scan data"""
        ranges = []
        
        for angle in self.laser_angles:
            # Simulate obstacles at various distances
            base_range = self.laser_range
            
            # Add some obstacles
            if abs(angle) < 0.5:  # Front obstacle
                base_range = 3.0 + 0.5 * np.sin(self.sim_time * 2)
            elif abs(angle - np.pi/2) < 0.3:  # Side obstacle
                base_range = 2.0 + 0.3 * np.cos(self.sim_time * 1.5)
            elif abs(angle + np.pi/2) < 0.3:  # Other side
                base_range = 2.5 + 0.2 * np.sin(self.sim_time * 3)
            
            # Add noise
            noise = 0.1 * np.random.randn()
            ranges.append(max(0.1, base_range + noise))
        
        return LaserScan(
            angle_min=self.laser_angles[0],
            angle_max=self.laser_angles[-1],
            angle_increment=self.laser_angles[1] - self.laser_angles[0],
            ranges=ranges,
            timestamp=time.time()
        )
    
    def visualize_robot(self):
        """Visualize robot in rerun"""
        # Set time
        rr.set_time_sequence("sim_time", int(self.sim_time * 10))
        
        # Robot base
        rr.log("robot/base", rr.Transform3D(
            translation=self.robot_pos,
            rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=self.robot_yaw)
        ))
        
        # Robot base mesh (simple box)
        rr.log("robot/base/mesh", rr.Boxes3D(
            sizes=[[0.6, 0.4, 0.2]],
            colors=[[0, 100, 200]]
        ))
        
        # Robot arm joints
        arm_base = self.robot_pos + np.array([0, 0, 0.2])
        
        # Joint 1 (base rotation)
        j1_pos = arm_base
        rr.log("robot/arm/joint1", rr.Transform3D(
            translation=j1_pos,
            rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=self.joint_positions[0])
        ))
        
        # Link 1
        link1_end = j1_pos + np.array([0.3 * np.cos(self.joint_positions[0]), 
                                      0.3 * np.sin(self.joint_positions[0]), 0])
        rr.log("robot/arm/link1", rr.LineStrips3D([j1_pos, link1_end], colors=[255, 0, 0]))
        
        # Joint 2
        rr.log("robot/arm/joint2", rr.Transform3D(
            translation=link1_end,
            rotation=rr.RotationAxisAngle(axis=[0, 0, 1], 
                                        angle=self.joint_positions[0] + self.joint_positions[1])
        ))
        
        # Link 2
        link2_angle = self.joint_positions[0] + self.joint_positions[1]
        link2_end = link1_end + np.array([0.25 * np.cos(link2_angle), 
                                         0.25 * np.sin(link2_angle), 0])
        rr.log("robot/arm/link2", rr.LineStrips3D([link1_end, link2_end], colors=[0, 255, 0]))
        
        # End effector
        rr.log("robot/arm/end_effector", rr.Points3D(
            positions=[link2_end],
            colors=[[255, 255, 0]],
            radii=[0.05]
        ))
    
    def visualize_laser_scan(self, laser_scan: LaserScan):
        """Visualize laser scan data"""
        points = []
        
        for i, (angle, range_val) in enumerate(zip(self.laser_angles, laser_scan.ranges)):
            if range_val < self.laser_range:
                # Convert polar to cartesian
                x = self.robot_pos[0] + range_val * np.cos(angle + self.robot_yaw)
                y = self.robot_pos[1] + range_val * np.sin(angle + self.robot_yaw)
                z = self.robot_pos[2]
                points.append([x, y, z])
        
        if points:
            rr.log("robot/laser_scan", rr.Points3D(
                positions=points,
                colors=[[255, 0, 255]],
                radii=[0.02]
            ))
    
    def publish_data(self):
        """Publish robot data as miniROS messages"""
        import json
        
        # Robot state
        robot_state = RobotState(
            position=tuple(self.robot_pos),
            orientation=(0, 0, np.sin(self.robot_yaw/2), np.cos(self.robot_yaw/2)),
            joint_positions=self.joint_positions.copy(),
            timestamp=time.time()
        )
        
        robot_msg = mini_ros.String(data=json.dumps({
            'position': robot_state.position,
            'orientation': robot_state.orientation,
            'joint_positions': robot_state.joint_positions,
            'timestamp': robot_state.timestamp
        }))
        self.robot_state_pub.publish(robot_msg)
        
        # Laser scan
        laser_scan = self.generate_laser_scan()
        laser_msg = mini_ros.String(data=json.dumps({
            'angle_min': laser_scan.angle_min,
            'angle_max': laser_scan.angle_max,
            'angle_increment': laser_scan.angle_increment,
            'ranges': laser_scan.ranges,
            'timestamp': laser_scan.timestamp
        }))
        self.laser_pub.publish(laser_msg)
        
        # Odometry
        odom_msg = mini_ros.String(data=json.dumps({
            'position': tuple(self.robot_pos),
            'velocity': tuple(self.robot_vel),
            'yaw': self.robot_yaw,
            'yaw_rate': self.robot_yaw_rate,
            'timestamp': time.time()
        }))
        self.odom_pub.publish(odom_msg)
        
        # Visualize
        self.visualize_robot()
        self.visualize_laser_scan(laser_scan)
        
        # Log metrics
        rr.log("metrics/robot_speed", rr.Scalar(np.linalg.norm(self.robot_vel)))
        rr.log("metrics/robot_yaw", rr.Scalar(self.robot_yaw))
        rr.log("metrics/joint_angles", rr.BarChart(self.joint_positions))
    
    def run(self):
        """Main simulation loop"""
        rate = 1.0 / self.dt  # Hz
        
        self.get_logger().info(f"Starting robot visualization at {rate} Hz")
        
        while mini_ros.ok():
            start_time = time.time()
            
            # Update simulation
            self.update_robot_state()
            
            # Publish and visualize
            self.publish_data()
            
            # Log info
            if int(self.sim_time * 10) % 50 == 0:  # Every 5 seconds
                self.get_logger().info(f'Simulation time: {self.sim_time:.1f}s, '
                                     f'Robot pos: ({self.robot_pos[0]:.2f}, '
                                     f'{self.robot_pos[1]:.2f}, {self.robot_pos[2]:.2f})')
            
            # Maintain loop rate
            elapsed = time.time() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

def main():
    # Initialize miniROS
    mini_ros.init()
    
    try:
        # Create and run visualizer
        visualizer = RobotVisualizer()
        visualizer.run()
        
    except KeyboardInterrupt:
        print("\nShutting down robot visualizer...")
    finally:
        mini_ros.shutdown()

if __name__ == '__main__':
    main() 