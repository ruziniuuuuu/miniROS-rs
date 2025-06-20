#!/usr/bin/env python3
"""
Turtlebot Controller - Python miniROS Package

Enhanced turtlebot control system with:
- Advanced movement patterns (forward, circle, square, figure-8)
- Odometry feedback processing
- Safety features and error handling
- Real-time status monitoring

Demonstrates miniROS Python bindings with robotics patterns.
"""

import mini_ros
import time
import math
import signal
import sys
from enum import Enum
from typing import Optional, Tuple

class MovementPattern(Enum):
    STOP = "stop"
    FORWARD = "forward"
    BACKWARD = "backward" 
    CIRCLE = "circle"
    SQUARE = "square"
    FIGURE8 = "figure8"
    SPIN = "spin"

class RobotPose:
    """Robot pose information"""
    def __init__(self):
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.linear_vel: float = 0.0
        self.angular_vel: float = 0.0
        self.timestamp: float = time.time()

class TurtlebotController:
    """Enhanced Turtlebot Controller with safety features"""
    
    def __init__(self, node_name: str = "py_turtlebot"):
        print(f"🤖 Initializing {node_name}")
        print("📦 Package: turtlebot")
        print("📍 Executable: py_controller")
        
        # Initialize miniROS
        mini_ros.init()
        self.node = mini_ros.Node(node_name)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(
            mini_ros.TwistMessage, '/cmd_vel', 10
        )
        self.odom_sub = self.node.create_subscription(
            mini_ros.OdometryMessage, '/odom', self._odom_callback, 10
        )
        
        # State management
        self.current_pose = RobotPose()
        self.start_time = time.time()
        self.running = True
        
        # Safety limits
        self.max_linear_speed = 1.0   # m/s
        self.max_angular_speed = 2.0  # rad/s
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("✅ Controller ready")
        
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n📡 Received signal {signum}, shutting down...")
        self.running = False
        self.emergency_stop()
        
    def _odom_callback(self, odom_msg):
        """Process odometry feedback with improved error handling"""
        try:
            # Extract pose information
            self.current_pose.x = odom_msg.position[0]
            self.current_pose.y = odom_msg.position[1]
            self.current_pose.yaw = odom_msg.get_yaw()
            
            # Extract velocity information
            self.current_pose.linear_vel = odom_msg.velocity[0]
            self.current_pose.angular_vel = odom_msg.velocity[5]
            self.current_pose.timestamp = time.time()
            
            # Periodic status reporting
            if hasattr(self, '_last_odom_report'):
                if time.time() - self._last_odom_report > 2.0:
                    self._print_pose_status()
                    self._last_odom_report = time.time()
            else:
                self._last_odom_report = time.time()
                
        except Exception as e:
            print(f"⚠️ Error processing odometry: {e}")
    
    def _print_pose_status(self):
        """Print current robot pose status"""
        print(f"📍 Pose: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, "
              f"yaw={math.degrees(self.current_pose.yaw):.1f}°, "
              f"vel={self.current_pose.linear_vel:.2f}m/s")
    
    def publish_velocity(self, linear_x: float = 0.0, angular_z: float = 0.0) -> bool:
        """Send velocity command with safety checks"""
        # Apply safety limits
        safe_linear = max(-self.max_linear_speed, 
                         min(self.max_linear_speed, linear_x))
        safe_angular = max(-self.max_angular_speed, 
                          min(self.max_angular_speed, angular_z))
        
        # Create twist message
        twist = mini_ros.TwistMessage()
        twist.set_linear(safe_linear, 0.0, 0.0)
        twist.set_angular(0.0, 0.0, safe_angular)
        
        # Validate and publish
        if twist.validate():
            self.cmd_vel_pub.publish(twist)
            
            # Only log non-zero commands to reduce spam
            if abs(safe_linear) > 0.01 or abs(safe_angular) > 0.01:
                print(f"🎮 Velocity: linear={safe_linear:.2f}m/s, angular={safe_angular:.2f}rad/s")
            
            return True
        else:
            print(f"⚠️ Invalid velocity command: linear={linear_x:.2f}, angular={angular_z:.2f}")
            return False
    
    def execute_pattern(self, pattern: MovementPattern, duration: float) -> bool:
        """Execute movement pattern with improved timing and safety"""
        print(f"🤖 Executing {pattern.value} pattern for {duration:.1f}s")
        
        start_time = time.time()
        end_time = start_time + duration
        last_status = time.time()
        
        try:
            while time.time() < end_time and self.running:
                elapsed = time.time() - start_time
                
                # Calculate velocities based on pattern
                if pattern == MovementPattern.STOP:
                    linear, angular = 0.0, 0.0
                    
                elif pattern == MovementPattern.FORWARD:
                    linear, angular = 0.4, 0.0
                    
                elif pattern == MovementPattern.BACKWARD:
                    linear, angular = -0.3, 0.0
                    
                elif pattern == MovementPattern.CIRCLE:
                    linear, angular = 0.3, 0.8
                    
                elif pattern == MovementPattern.SQUARE:
                    # Square pattern: forward -> turn -> repeat
                    side_time = 3.0  # 3 seconds per side
                    turn_time = 1.6  # 1.6 seconds per turn (90°)
                    cycle_time = side_time + turn_time
                    
                    phase = (elapsed % cycle_time) / cycle_time
                    if phase < (side_time / cycle_time):
                        linear, angular = 0.3, 0.0  # Forward
                    else:
                        linear, angular = 0.0, math.pi / 2  # Turn 90°
                        
                elif pattern == MovementPattern.FIGURE8:
                    # Figure-8 using sinusoidal motion
                    frequency = 0.3  # Complete figure-8 every ~3.3 seconds
                    linear = 0.3
                    angular = 1.5 * math.sin(2 * math.pi * frequency * elapsed)
                    
                elif pattern == MovementPattern.SPIN:
                    linear, angular = 0.0, 1.5
                    
                else:
                    linear, angular = 0.0, 0.0
                
                # Publish velocity command
                if not self.publish_velocity(linear, angular):
                    print("⚠️ Failed to publish velocity, stopping pattern")
                    break
                
                # Print status every 3 seconds
                if time.time() - last_status > 3.0:
                    remaining = end_time - time.time()
                    print(f"📊 Pattern progress: {elapsed:.1f}s elapsed, {remaining:.1f}s remaining")
                    last_status = time.time()
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            print("\n⏹️ Pattern interrupted by user")
            self.running = False
            
        finally:
            # Always stop the robot
            self.publish_velocity(0.0, 0.0)
        
        print("✅ Pattern execution completed")
        return True
    
    def publish_mock_odometry(self):
        """Publish mock odometry data for demonstration"""
        try:
            odom_pub = self.node.create_publisher(mini_ros.OdometryMessage, '/odom', 10)
            odom = mini_ros.OdometryMessage()
            odom.set_pose_2d(0.0, 0.0, 0.0)
            odom_pub.publish(odom)
            print("📡 Mock odometry published")
        except Exception as e:
            print(f"⚠️ Failed to publish mock odometry: {e}")
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all motion"""
        print("🛑 Emergency stop activated!")
        for _ in range(3):  # Send multiple stop commands for safety
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.1)
    
    def run_demo(self) -> bool:
        """Run comprehensive demonstration sequence"""
        print("🚀 Starting turtlebot Python controller demo")
        print("📋 Demo sequence - Advanced movement patterns:")
        
        try:
            # Publish initial mock odometry
            self.publish_mock_odometry()
            time.sleep(0.5)
            
            # Demo patterns with varying durations
            patterns = [
                (MovementPattern.FORWARD, 3.0, "1. Forward movement"),
                (MovementPattern.BACKWARD, 2.0, "2. Backward movement"),
                (MovementPattern.CIRCLE, 6.0, "3. Circular motion"),
                (MovementPattern.SQUARE, 18.0, "4. Square pattern (advanced)"),
                (MovementPattern.FIGURE8, 10.0, "5. Figure-8 pattern"),
                (MovementPattern.SPIN, 4.0, "6. Spin in place"),
            ]
            
            for i, (pattern, duration, description) in enumerate(patterns):
                if not self.running:
                    break
                    
                print(f"\n🔸 {description} ({duration}s)")
                
                if not self.execute_pattern(pattern, duration):
                    print(f"❌ Pattern {i+1} failed")
                    break
                
                # Brief pause between patterns
                if self.running and i < len(patterns) - 1:
                    print("⏸️ Pausing between patterns...")
                    time.sleep(1.0)
            
            if self.running:
                print("\n🎉 Python turtlebot controller demo completed successfully!")
                return True
            else:
                print("\n⏹️ Demo stopped by user")
                return False
                
        except Exception as e:
            print(f"❌ Demo failed with error: {e}")
            return False
        
        finally:
            self.emergency_stop()
    
    def cleanup(self):
        """Clean up resources"""
        print("🧹 Cleaning up resources...")
        self.emergency_stop()
        
        try:
            self.node.destroy_node()
            mini_ros.shutdown()
        except Exception as e:
            print(f"⚠️ Cleanup warning: {e}")
        
        print("🏁 Cleanup completed")

def main():
    """Main entry point"""
    print("=" * 50)
    print("🐍 miniROS Python: Turtlebot Controller")
    print("📦 Package: turtlebot")
    print("📍 Script: controller.py")
    print("=" * 50)
    
    controller = None
    try:
        # Create and run controller
        controller = TurtlebotController("py_turtlebot_ctrl")
        
        # Run the demonstration
        success = controller.run_demo()
        
        # Brief pause to see final messages
        if success:
            print("\n⏳ Demo completed, shutting down in 2 seconds...")
            time.sleep(2.0)
        
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user (Ctrl+C)")
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        
    finally:
        if controller:
            controller.cleanup()
        
        print("🏁 Python controller finished")

if __name__ == "__main__":
    main() 