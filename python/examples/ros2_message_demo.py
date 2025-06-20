#!/usr/bin/env python3
"""
ROS2 Message Packages Demo - miniROS Python API

This example demonstrates the new ROS2-compatible message packages:
- std_msgs: Basic data types (String, Int32, Float64, Bool, Header)
- geometry_msgs: Geometric data types (Point, Pose, Twist, etc.)
- nav_msgs: Navigation data types (Odometry, Path)

This shows how miniROS-rs provides drop-in compatibility with ROS2.
"""

import mini_ros
import time
import math


def demonstrate_std_msgs():
    """Demonstrate std_msgs package"""
    print("\n=== std_msgs Package Demo ===")
    
    # String message
    string_msg = mini_ros.std_msgs.String()
    string_msg.data = "Hello from miniROS-rs!"
    print(f"String message: {string_msg.data}")
    
    # Numeric messages
    int32_msg = mini_ros.std_msgs.Int32()
    int32_msg.data = 42
    print(f"Int32 message: {int32_msg.data}")
    
    float64_msg = mini_ros.std_msgs.Float64()
    float64_msg.data = 3.14159
    print(f"Float64 message: {float64_msg.data}")
    
    # Bool message
    bool_msg = mini_ros.std_msgs.Bool()
    bool_msg.data = True
    print(f"Bool message: {bool_msg.data}")
    
    # Header message
    header_msg = mini_ros.std_msgs.Header()
    header_msg.frame_id = "base_link"
    header_msg.stamp_sec = int(time.time())
    header_msg.stamp_nanosec = int((time.time() % 1) * 1e9)
    print(f"Header message: frame_id={header_msg.frame_id}, stamp={header_msg.stamp_sec}.{header_msg.stamp_nanosec}")


def demonstrate_geometry_msgs():
    """Demonstrate geometry_msgs package"""
    print("\n=== geometry_msgs Package Demo ===")
    
    # Point message
    point_msg = mini_ros.geometry_msgs.Point()
    point_msg.x = 1.0
    point_msg.y = 2.0
    point_msg.z = 3.0
    print(f"Point: ({point_msg.x}, {point_msg.y}, {point_msg.z})")
    
    # Vector3 message
    vector_msg = mini_ros.geometry_msgs.Vector3()
    vector_msg.x = 0.5
    vector_msg.y = -0.3
    vector_msg.z = 0.8
    print(f"Vector3: ({vector_msg.x}, {vector_msg.y}, {vector_msg.z})")
    
    # Quaternion message
    quat_msg = mini_ros.geometry_msgs.Quaternion()
    quat_msg.x = 0.0
    quat_msg.y = 0.0
    quat_msg.z = 0.707  # 90 degrees around Z-axis
    quat_msg.w = 0.707
    print(f"Quaternion: ({quat_msg.x}, {quat_msg.y}, {quat_msg.z}, {quat_msg.w})")
    
    # Pose message (combines Point and Quaternion)
    pose_msg = mini_ros.geometry_msgs.Pose()
    pose_msg.position = point_msg
    pose_msg.orientation = quat_msg
    print(f"Pose: position=({pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z})")
    print(f"      orientation=({pose_msg.orientation.x}, {pose_msg.orientation.y}, {pose_msg.orientation.z}, {pose_msg.orientation.w})")
    
    # PoseStamped message (Pose with timestamp)
    pose_stamped_msg = mini_ros.geometry_msgs.PoseStamped()
    pose_stamped_msg.header.frame_id = "map"
    pose_stamped_msg.header.stamp_sec = int(time.time())
    pose_stamped_msg.pose = pose_msg
    print(f"PoseStamped: frame_id={pose_stamped_msg.header.frame_id}")
    
    # Twist message (velocity)
    twist_msg = mini_ros.geometry_msgs.Twist()
    twist_msg.linear.x = 1.0  # Forward velocity
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.5  # Turning velocity
    print(f"Twist: linear=({twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.linear.z})")
    print(f"       angular=({twist_msg.angular.x}, {twist_msg.angular.y}, {twist_msg.angular.z})")
    
    # Demonstrate validation
    if hasattr(twist_msg, 'validate'):
        is_valid = twist_msg.validate()
        print(f"Twist message is valid: {is_valid}")


def demonstrate_nav_msgs():
    """Demonstrate nav_msgs package"""
    print("\n=== nav_msgs Package Demo ===")
    
    # Odometry message (robot state)
    odom_msg = mini_ros.nav_msgs.Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.header.stamp_sec = int(time.time())
    odom_msg.child_frame_id = "base_link"
    
    # Set robot pose
    odom_msg.pose.position.x = 5.0
    odom_msg.pose.position.y = 3.0
    odom_msg.pose.position.z = 0.0
    odom_msg.pose.orientation.x = 0.0
    odom_msg.pose.orientation.y = 0.0
    odom_msg.pose.orientation.z = 0.0
    odom_msg.pose.orientation.w = 1.0
    
    # Set robot velocity
    odom_msg.twist.linear.x = 0.5
    odom_msg.twist.linear.y = 0.0
    odom_msg.twist.linear.z = 0.0
    odom_msg.twist.angular.x = 0.0
    odom_msg.twist.angular.y = 0.0
    odom_msg.twist.angular.z = 0.1
    
    print(f"Odometry: frame_id={odom_msg.header.frame_id}")
    print(f"          child_frame_id={odom_msg.child_frame_id}")
    print(f"          position=({odom_msg.pose.position.x}, {odom_msg.pose.position.y}, {odom_msg.pose.position.z})")
    print(f"          linear_vel=({odom_msg.twist.linear.x}, {odom_msg.twist.linear.y}, {odom_msg.twist.linear.z})")
    
    # Demonstrate convenience methods
    if hasattr(odom_msg, 'get_yaw'):
        yaw = odom_msg.get_yaw()
        print(f"          yaw angle: {yaw} radians ({math.degrees(yaw):.1f} degrees)")
    
    if hasattr(odom_msg, 'set_pose_2d'):
        # Set 2D pose (common for ground robots)
        odom_msg.set_pose_2d(10.0, 5.0, math.pi/4)  # x=10, y=5, yaw=45°
        print(f"After set_pose_2d: position=({odom_msg.pose.position.x}, {odom_msg.pose.position.y})")
        print(f"                  yaw: {odom_msg.get_yaw():.3f} radians ({math.degrees(odom_msg.get_yaw()):.1f}°)")


def demonstrate_pub_sub():
    """Demonstrate publishing and subscribing with new message types"""
    print("\n=== Pub/Sub Demo with New Message Types ===")
    
    # Initialize miniROS
    mini_ros.init()
    
    # Create node
    node = mini_ros.Node('ros2_message_demo')
    logger = node.get_logger()
    
    # Create publishers for different message types
    string_pub = node.create_publisher(mini_ros.std_msgs.String, 'demo/string', 10)
    pose_pub = node.create_publisher(mini_ros.geometry_msgs.Pose, 'demo/pose', 10)
    twist_pub = node.create_publisher(mini_ros.geometry_msgs.Twist, 'demo/twist', 10)
    odom_pub = node.create_publisher(mini_ros.nav_msgs.Odometry, 'demo/odom', 10)
    
    # Create subscribers
    def string_callback(msg):
        logger.info(f"Received string: {msg.data}")
    
    def pose_callback(msg):
        logger.info(f"Received pose: pos=({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})")
    
    def twist_callback(msg):
        logger.info(f"Received twist: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
    
    def odom_callback(msg):
        logger.info(f"Received odometry: frame_id={msg.header.frame_id}")
    
    string_sub = node.create_subscription(mini_ros.std_msgs.String, 'demo/string', string_callback, 10)
    pose_sub = node.create_subscription(mini_ros.geometry_msgs.Pose, 'demo/pose', pose_callback, 10)
    twist_sub = node.create_subscription(mini_ros.geometry_msgs.Twist, 'demo/twist', twist_callback, 10)
    odom_sub = node.create_subscription(mini_ros.nav_msgs.Odometry, 'demo/odom', odom_callback, 10)
    
    logger.info("Created publishers and subscribers for ROS2 message types")
    
    # Publish some messages
    count = 0
    try:
        while count < 5:
            # String message
            string_msg = mini_ros.std_msgs.String()
            string_msg.data = f"Message {count}"
            string_pub.publish(string_msg)
            
            # Pose message
            pose_msg = mini_ros.geometry_msgs.Pose()
            pose_msg.position.x = float(count)
            pose_msg.position.y = float(count * 2)
            pose_msg.position.z = 0.0
            pose_msg.orientation.w = 1.0
            pose_pub.publish(pose_msg)
            
            # Twist message
            twist_msg = mini_ros.geometry_msgs.Twist()
            twist_msg.linear.x = 1.0
            twist_msg.angular.z = 0.1 * count
            twist_pub.publish(twist_msg)
            
            # Odometry message
            odom_msg = mini_ros.nav_msgs.Odometry()
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp_sec = int(time.time())
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose = pose_msg
            odom_msg.twist = twist_msg
            odom_pub.publish(odom_msg)
            
            count += 1
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        logger.info("Demo interrupted")
    
    # Cleanup
    node.destroy_node()
    mini_ros.shutdown()


def demonstrate_ros2_compatibility():
    """Show how miniROS-rs provides ROS2 compatibility"""
    print("\n=== ROS2 Compatibility Demo ===")
    
    # Show different ways to access message types (all equivalent)
    print("Different ways to access the same message type:")
    
    # Method 1: Package-based access (recommended)
    string_msg1 = mini_ros.std_msgs.String()
    string_msg1.data = "Method 1: Package-based"
    print(f"  {string_msg1.data}")
    
    # Method 2: Direct access (legacy compatibility)
    string_msg2 = mini_ros.String()
    string_msg2.data = "Method 2: Direct access"
    print(f"  {string_msg2.data}")
    
    # Method 3: Legacy naming (backward compatibility)
    string_msg3 = mini_ros.StringMessage()
    string_msg3.data = "Method 3: Legacy naming"
    print(f"  {string_msg3.data}")
    
    print("\nAll three methods create the same message type!")
    print(f"Type 1: {type(string_msg1)}")
    print(f"Type 2: {type(string_msg2)}")
    print(f"Type 3: {type(string_msg3)}")


def main():
    """Main demonstration function"""
    print("miniROS-rs ROS2 Message Packages Demo")
    print("=====================================")
    print("\nThis demo shows the new ROS2-compatible message packages:")
    print("- std_msgs: Basic data types")
    print("- geometry_msgs: Geometric data types")
    print("- nav_msgs: Navigation data types")
    
    # Demonstrate each package
    demonstrate_std_msgs()
    demonstrate_geometry_msgs()
    demonstrate_nav_msgs()
    demonstrate_ros2_compatibility()
    
    print("\n=== Interactive Pub/Sub Demo ===")
    print("Press Ctrl+C to skip the pub/sub demo")
    try:
        demonstrate_pub_sub()
    except KeyboardInterrupt:
        print("\nSkipping pub/sub demo")
    
    print("\n=== Demo Complete ===")
    print("miniROS-rs now provides full ROS2 message compatibility!")
    print("You can use these message types in your ROS2 applications as drop-in replacements.")


if __name__ == '__main__':
    main() 