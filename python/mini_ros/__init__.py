"""
miniROS-rs Python bindings

A high-performance ROS2-like middleware with Python API compatibility.
"""

try:
    from ._core import Node as _RustNode, Publisher as _RustPublisher, Subscription as _RustSubscription
    from ._core import init as _rust_init, shutdown as _rust_shutdown, spin as _rust_spin, spin_once as _rust_spin_once
    _BINDINGS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Rust bindings not available: {e}")
    print("Using pure Python stubs for testing.")
    _BINDINGS_AVAILABLE = False
from typing import Callable, Any, Optional

__version__ = "0.1.1"
__author__ = "Chenyu Cao"
__email__ = "ruziniuuuuu@gmail.com"

# Global state
_initialized = False

# Python stubs for when Rust bindings are not available
if not _BINDINGS_AVAILABLE:
    class _RustNode:
        def __init__(self, name): 
            self.name = name
            print(f"[STUB] Created Rust node: {name}")
        def get_name(self): 
            return self.name
        def destroy_node(self): 
            print(f"[STUB] Destroyed node: {self.name}")
    
    class _RustPublisher:
        def __init__(self, msg_type, topic):
            self.msg_type = msg_type
            self.topic = topic
            print(f"[STUB] Created publisher: {topic} ({msg_type})")
        def publish(self, data):
            print(f"[STUB] Publishing to {self.topic}: {data}")
        def get_subscription_count(self):
            return 0
    
    class _RustSubscription:
        def __init__(self, msg_type, topic):
            self.msg_type = msg_type
            self.topic = topic
            print(f"[STUB] Created subscription: {topic} ({msg_type})")
    
    def _rust_init():
        print("[STUB] miniROS initialized")
    
    def _rust_shutdown():
        print("[STUB] miniROS shutdown")
    
    def _rust_spin(node):
        print(f"[STUB] Spinning node: {node.get_name()}")
        import time
        while True:
            time.sleep(0.1)
    
    def _rust_spin_once(node, timeout_ms):
        print(f"[STUB] Spin once: {node.get_name()}")
        import time
        time.sleep((timeout_ms or 100) / 1000.0)


def init(*, args=None, context=None, log_level=None):
    """
    Initialize miniROS system
    
    Args:
        args: Command line arguments (unused, for ROS2 compatibility)
        context: Context (unused, for ROS2 compatibility) 
        log_level: Log level (unused, for ROS2 compatibility)
    """
    global _initialized
    if not _initialized:
        _rust_init()
        _initialized = True


def shutdown(*, context=None):
    """
    Shutdown miniROS system
    
    Args:
        context: Context (unused, for ROS2 compatibility)
    """
    global _initialized
    if _initialized:
        _rust_shutdown()
        _initialized = False


def ok(*, context=None) -> bool:
    """
    Check if miniROS is running
    
    Args:
        context: Context (unused, for ROS2 compatibility)
        
    Returns:
        bool: True if miniROS is initialized
    """
    return _initialized


class Node:
    """
    A miniROS Node - equivalent to rclpy.Node
    
    This class provides the same API as ROS2 rclpy.Node for compatibility.
    """
    
    def __init__(self, node_name: str, *, context=None, cli_args=None, namespace=None, 
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False, 
                 automatically_declare_parameters_from_overrides=False):
        """
        Create a new Node
        
        Args:
            node_name: Name of the node
            context: Context (unused, for ROS2 compatibility)
            cli_args: CLI arguments (unused, for ROS2 compatibility)
            namespace: Node namespace (unused, for ROS2 compatibility)
            use_global_arguments: Use global arguments (unused, for ROS2 compatibility)
            enable_rosout: Enable rosout (unused, for ROS2 compatibility)
            start_parameter_services: Start parameter services (unused, for ROS2 compatibility)
            parameter_overrides: Parameter overrides (unused, for ROS2 compatibility)
            allow_undeclared_parameters: Allow undeclared parameters (unused, for ROS2 compatibility)
            automatically_declare_parameters_from_overrides: Auto declare parameters (unused, for ROS2 compatibility)
        """
        if not _initialized:
            raise RuntimeError("miniROS not initialized. Call mini_ros.init() first.")
            
        self._rust_node = _RustNode(node_name)
        self._node_name = node_name
        self._subscriptions = []
        self._publishers = []
    
    def get_name(self) -> str:
        """Get the node name"""
        return self._node_name
    
    def get_namespace(self) -> str:
        """Get the node namespace (always '/' for compatibility)"""
        return "/"
    
    def get_logger(self):
        """Get logger (returns simple logger for compatibility)"""
        node_name = self._node_name
        class SimpleLogger:
            def info(self, msg): print(f"[INFO] [{node_name}]: {msg}")
            def warn(self, msg): print(f"[WARN] [{node_name}]: {msg}")
            def error(self, msg): print(f"[ERROR] [{node_name}]: {msg}")
            def debug(self, msg): print(f"[DEBUG] [{node_name}]: {msg}")
        return SimpleLogger()
    
    def create_publisher(self, msg_type, topic: str, qos_profile=10, *, callback_group=None, event_callbacks=None):
        """
        Create a publisher
        
        Args:
            msg_type: Message type (String, Float64, Int32)
            topic: Topic name
            qos_profile: QoS profile (unused, for compatibility)
            callback_group: Callback group (unused, for compatibility)
            event_callbacks: Event callbacks (unused, for compatibility)
            
        Returns:
            Publisher: Publisher instance
        """
        # Convert message type to string
        if hasattr(msg_type, '__name__'):
            type_name = msg_type.__name__
        else:
            type_name = str(msg_type)
            
        # Map common ROS2 message types
        if 'String' in type_name:
            type_name = 'String'
        elif 'Float64' in type_name:
            type_name = 'Float64'
        elif 'Int32' in type_name:
            type_name = 'Int32'
            
        # Create the Rust publisher directly
        rust_publisher = _RustPublisher(type_name, topic)
        publisher = Publisher(rust_publisher, topic, type_name)
        self._publishers.append(publisher)
        return publisher
    
    def create_subscription(self, msg_type, topic: str, callback: Callable, qos_profile=10, *, 
                          callback_group=None, event_callbacks=None, raw=False):
        """
        Create a subscription
        
        Args:
            msg_type: Message type (String, Float64, Int32)
            topic: Topic name
            callback: Callback function
            qos_profile: QoS profile (unused, for compatibility)
            callback_group: Callback group (unused, for compatibility)
            event_callbacks: Event callbacks (unused, for compatibility)
            raw: Use raw messages (unused, for compatibility)
            
        Returns:
            Subscription: Subscription instance
        """
        # Convert message type to string
        if hasattr(msg_type, '__name__'):
            type_name = msg_type.__name__
        else:
            type_name = str(msg_type)
            
        # Map common ROS2 message types
        if 'String' in type_name:
            type_name = 'String'
        elif 'Float64' in type_name:
            type_name = 'Float64'
        elif 'Int32' in type_name:
            type_name = 'Int32'
            
        # Create the Rust subscription directly
        rust_subscription = _RustSubscription(type_name, topic)
        subscription = Subscription(rust_subscription, topic, type_name, callback)
        self._subscriptions.append(subscription)
        return subscription
    
    def destroy_node(self):
        """Destroy the node and cleanup resources"""
        if hasattr(self._rust_node, 'destroy_node'):
            self._rust_node.destroy_node()


class Publisher:
    """Publisher wrapper for ROS2 compatibility"""
    
    def __init__(self, rust_publisher, topic: str, msg_type: str):
        self._rust_publisher = rust_publisher
        self._topic = topic
        self._msg_type = msg_type
    
    def publish(self, msg):
        """
        Publish a message
        
        Args:
            msg: Message to publish (can be primitive type or message object)
        """
        # Extract data from message object if needed
        if hasattr(msg, 'data'):
            data = msg.data
        else:
            data = msg
            
        # Call Rust publisher directly
        self._rust_publisher.publish(data)
    
    def get_subscription_count(self) -> int:
        """Get subscription count (returns 0 for compatibility)"""
        return self._rust_publisher.get_subscription_count()


class Subscription:
    """Subscription wrapper for ROS2 compatibility"""
    
    def __init__(self, rust_subscription, topic: str, msg_type: str, callback: Callable):
        self._rust_subscription = rust_subscription
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback
        # For now, subscriptions just store the callback
        # In a full implementation, this would register with the transport layer


def spin(node: Node, *, executor=None, context=None):
    """
    Spin a node indefinitely
    
    Args:
        node: Node to spin
        executor: Executor (unused, for compatibility)
        context: Context (unused, for compatibility)
    """
    _rust_spin(node._rust_node)


def spin_once(node: Node, *, executor=None, timeout_sec=None, context=None):
    """
    Spin a node once
    
    Args:
        node: Node to spin
        executor: Executor (unused, for compatibility)
        timeout_sec: Timeout in seconds
        context: Context (unused, for compatibility)
    """
    timeout_ms = int(timeout_sec * 1000) if timeout_sec else None
    _rust_spin_once(node._rust_node, timeout_ms)


# Message types for compatibility
class String:
    """String message type for compatibility"""
    def __init__(self, data: str = ""):
        self.data = data


class Float64:
    """Float64 message type for compatibility"""  
    def __init__(self, data: float = 0.0):
        self.data = data


class Int32:
    """Int32 message type for compatibility"""
    def __init__(self, data: int = 0):
        self.data = data


# Export main API
__all__ = [
    'init', 'shutdown', 'ok', 'spin', 'spin_once',
    'Node', 'Publisher', 'Subscription',
    'String', 'Float64', 'Int32'
] 