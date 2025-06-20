#!/usr/bin/env python3
"""
Turtlebot Controller - Python miniROS Example

Classic ROS/ROS2 turtlebot control pattern:
- Publishes velocity commands to `/cmd_vel` 
- Subscribes to odometry from `/odom`
- Basic movement patterns (forward, circle, square)

Demonstrates miniROS core principle: essential robotics with minimal complexity
"""

import mini_ros
import time
import math
from enum import Enum

class MovementPattern(Enum):
    STOP = "stop"
    FORWARD = "forward" 
    CIRCLE = "circle"
    SQUARE = "square"

class TurtlebotController:
    def __init__(self, node_name="py_turtlebot"):
        print(f"🤖 Initializing {node_name}")
        
        mini_ros.init()
        self.node = mini_ros.Node(node_name)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(mini_ros.TwistMessage, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(mini_ros.OdometryMessage, '/odom', self._odom_callback, 10)
        
        self.current_pose = None
        self.start_time = time.time()
        print("✅ Controller ready")
        
    def _odom_callback(self, odom_msg):
        """Process odometry feedback"""
        self.current_pose = {
            'x': odom_msg.position[0],
            'y': odom_msg.position[1], 
            'yaw': odom_msg.get_yaw()
        }
        print(f"📍 Pose: x={odom_msg.position[0]:.2f}, y={odom_msg.position[1]:.2f}, yaw={math.degrees(odom_msg.get_yaw()):.1f}°")
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to robot"""
        twist = mini_ros.TwistMessage()
        twist.set_linear(linear_x, 0.0, 0.0)
        twist.set_angular(0.0, 0.0, angular_z)
        
        if twist.validate():
            self.cmd_vel_pub.publish(twist)
            print(f"🎮 Cmd: linear={linear_x:.2f}, angular={angular_z:.2f}")
        else:
            print("⚠️  Invalid velocity command")
    
    def execute_pattern(self, pattern, duration):
        """Execute movement pattern"""
        print(f"🤖 Executing {pattern.value} for {duration}s")
        end_time = time.time() + duration
        
        while time.time() < end_time:
            if pattern == MovementPattern.STOP:
                self.publish_velocity()
            elif pattern == MovementPattern.FORWARD:
                self.publish_velocity(linear_x=0.5)
            elif pattern == MovementPattern.CIRCLE:
                self.publish_velocity(linear_x=0.3, angular_z=0.5)
            elif pattern == MovementPattern.SQUARE:
                # Square pattern: forward -> turn -> repeat
                elapsed = time.time() - self.start_time
                phase = (elapsed % 8.0) / 8.0
                if phase < 0.75:
                    self.publish_velocity(linear_x=0.3)  # Forward
                else:
                    self.publish_velocity(angular_z=math.pi/2)  # Turn 90°
            
            time.sleep(0.1)
        
        self.publish_velocity()  # Stop
        print("✅ Pattern completed")
    
    def publish_mock_odometry(self):
        """Publish mock odometry for demo"""
        odom_pub = self.node.create_publisher(mini_ros.OdometryMessage, '/odom', 10)
        odom = mini_ros.OdometryMessage()
        odom.set_pose_2d(0.0, 0.0, 0.0)
        odom_pub.publish(odom)
        print("📡 Mock odometry published")
    
    def run_demo(self):
        """Run complete demo sequence"""
        print("🚀 Starting turtlebot demo")
        
        self.publish_mock_odometry()
        time.sleep(0.5)
        
        # Demo sequence
        self.execute_pattern(MovementPattern.FORWARD, 3.0)
        time.sleep(1.0)
        
        self.execute_pattern(MovementPattern.CIRCLE, 5.0) 
        time.sleep(1.0)
        
        self.execute_pattern(MovementPattern.SQUARE, 10.0)
        
        print("🎉 Demo completed!")
    
    def cleanup(self):
        """Clean up resources"""
        self.publish_velocity()  # Stop robot
        self.node.destroy_node()
        mini_ros.shutdown()

def main():
    print("=== miniROS Python: Turtlebot Controller ===")
    
    controller = None
    try:
        controller = TurtlebotController()
        controller.run_demo()
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if controller:
            controller.cleanup()
        print("🏁 Example finished")

if __name__ == "__main__":
    main() 