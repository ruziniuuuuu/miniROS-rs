#!/usr/bin/env python3
"""
Complete Demo of miniROS Python API

This example demonstrates all the key features of the miniROS Python API:
- Node creation and management
- Publishers for different message types
- Subscriptions with callbacks
- ROS2-compatible syntax
- Proper cleanup
"""

import mini_ros
import time
import math


class ComprehensiveDemo(mini_ros.Node):
    """Comprehensive demonstration node"""

    def __init__(self):
        super().__init__('comprehensive_demo')
        
        # String publisher and subscriber
        self.string_pub = self.create_publisher(mini_ros.String, 'demo/strings', 10)
        self.string_sub = self.create_subscription(
            mini_ros.String, 'demo/strings', self.string_callback, 10
        )
        
        # Numeric publishers
        self.int_pub = self.create_publisher(mini_ros.Int32, 'demo/integers', 10)
        self.float_pub = self.create_publisher(mini_ros.Float64, 'demo/floats', 10)
        
        # Numeric subscribers
        self.int_sub = self.create_subscription(
            mini_ros.Int32, 'demo/integers', self.int_callback, 10
        )
        self.float_sub = self.create_subscription(
            mini_ros.Float64, 'demo/floats', self.float_callback, 10
        )
        
        # Demo state
        self.count = 0
        self.received_messages = {'string': 0, 'int': 0, 'float': 0}
        
        # Suppress unused variable warnings
        self.string_sub
        self.int_sub
        self.float_sub
        
        self.get_logger().info('🚀 Comprehensive demo node started!')
        self.get_logger().info(f'📝 Node name: {self.get_name()}')
        self.get_logger().info(f'📁 Node namespace: {self.get_namespace()}')

    def string_callback(self, msg):
        """Handle string messages"""
        self.received_messages['string'] += 1
        self.get_logger().info(f'📨 [STRING] Received: "{msg.data}"')

    def int_callback(self, msg):
        """Handle integer messages"""
        self.received_messages['int'] += 1
        self.get_logger().info(f'📨 [INT32] Received: {msg.data}')

    def float_callback(self, msg):
        """Handle float messages"""
        self.received_messages['float'] += 1
        self.get_logger().info(f'📨 [FLOAT64] Received: {msg.data:.3f}')

    def publish_string_message(self):
        """Publish a string message"""
        msg = mini_ros.String()
        msg.data = f'Hello World #{self.count}'
        self.string_pub.publish(msg)
        self.get_logger().info(f'📤 [STRING] Published: "{msg.data}"')

    def publish_int_message(self):
        """Publish an integer message"""
        msg = mini_ros.Int32()
        msg.data = self.count * 10
        self.int_pub.publish(msg)
        self.get_logger().info(f'📤 [INT32] Published: {msg.data}')

    def publish_float_message(self):
        """Publish a float message (sine wave)"""
        msg = mini_ros.Float64()
        msg.data = math.sin(self.count * 0.5) * 100.0
        self.float_pub.publish(msg)
        self.get_logger().info(f'📤 [FLOAT64] Published: {msg.data:.3f}')

    def publish_all_messages(self):
        """Publish messages of all types"""
        self.publish_string_message()
        self.publish_int_message() 
        self.publish_float_message()
        self.count += 1

    def print_statistics(self):
        """Print message statistics"""
        total = sum(self.received_messages.values())
        self.get_logger().info('📊 Message Statistics:')
        self.get_logger().info(f'   Total messages: {total}')
        self.get_logger().info(f'   String messages: {self.received_messages["string"]}')
        self.get_logger().info(f'   Integer messages: {self.received_messages["int"]}')
        self.get_logger().info(f'   Float messages: {self.received_messages["float"]}')


def demonstrate_message_types():
    """Demonstrate different message types"""
    print("\n🧪 Demonstrating message types:")
    
    # String message
    str_msg = mini_ros.String("Hello from miniROS!")
    print(f"   String message: {str_msg.data}")
    
    # Integer message
    int_msg = mini_ros.Int32(42)
    print(f"   Int32 message: {int_msg.data}")
    
    # Float message
    float_msg = mini_ros.Float64(3.14159)
    print(f"   Float64 message: {float_msg.data}")


def demonstrate_node_features():
    """Demonstrate Node class features"""
    print("\n🔧 Demonstrating Node features:")
    
    node = mini_ros.Node('feature_demo')
    print(f"   Node name: {node.get_name()}")
    print(f"   Node namespace: {node.get_namespace()}")
    
    # Test logger
    logger = node.get_logger()
    logger.info("This is an info message")
    logger.warn("This is a warning message") 
    logger.error("This is an error message")
    logger.debug("This is a debug message")
    
    node.destroy_node()
    print("   ✅ Node features demo complete")


def main():
    """Main demo function"""
    print("🎉 Welcome to miniROS Python API Comprehensive Demo!")
    print("=" * 60)
    
    # Initialize miniROS
    mini_ros.init()
    print("✅ miniROS initialized")
    
    try:
        # Demonstrate basic features
        demonstrate_message_types()
        demonstrate_node_features()
        
        print("\n🚀 Starting comprehensive node demo...")
        print("-" * 40)
        
        # Create demo node
        demo_node = ComprehensiveDemo()
        
        # Run demo for several iterations
        print(f"\n📡 Publishing and receiving messages (5 iterations)...")
        for i in range(5):
            print(f"\n--- Iteration {i+1} ---")
            demo_node.publish_all_messages()
            time.sleep(0.5)  # Small delay between messages
        
        # Print final statistics
        print("\n" + "="*50)
        demo_node.print_statistics()
        demo_node.get_logger().info('🎉 Demo completed successfully!')
        
    except KeyboardInterrupt:
        print('\n⚠️  Demo interrupted by user')
    
    except Exception as e:
        print(f'\n❌ Demo failed with error: {e}')
    
    finally:
        # Cleanup
        if 'demo_node' in locals():
            demo_node.destroy_node()
        mini_ros.shutdown()
        print('\n✅ miniROS shutdown complete')
        print('👋 Thank you for trying miniROS Python API!')


if __name__ == '__main__':
    main() 