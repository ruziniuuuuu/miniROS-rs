"""
miniROS-rs Python bindings

A high-performance ROS2-like middleware with Python API compatibility.
"""

try:
    from ._core import Node as _RustNode, Publisher as _RustPublisher, Subscription as _RustSubscription
    from ._core import init as _rust_init, shutdown as _rust_shutdown, spin as _rust_spin, spin_once as _rust_spin_once
    
    # Import new message types from Rust bindings
    from ._core import (
        # std_msgs
        String as _RustString, Int32 as _RustInt32, Int64 as _RustInt64,
        Float32 as _RustFloat32, Float64 as _RustFloat64, Bool as _RustBool,
        Header as _RustHeader,
        
        # geometry_msgs  
        Point as _RustPoint, Vector3 as _RustVector3, Quaternion as _RustQuaternion,
        Pose as _RustPose, PoseStamped as _RustPoseStamped, Twist as _RustTwist,
        
        # nav_msgs
        Odometry as _RustOdometry,
        
        # Legacy aliases
        StringMessage as _RustStringMessage, PoseMessage as _RustPoseMessage, 
        TwistMessage as _RustTwistMessage, OdometryMessage as _RustOdometryMessage,
        
        # Visualization
        VisualizationClient as _RustVisualizationClient
    )
    _BINDINGS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Rust bindings not available: {e}")
    print("Using pure Python stubs for testing.")
    _BINDINGS_AVAILABLE = False
from typing import Callable, Any, Optional
import threading
import time

__version__ = "0.1.5"
__author__ = "Chenyu Cao"
__email__ = "ruziniuuuuu@gmail.com"

# Global state
_initialized = False

# =============================================================================
# Message Type Definitions (with Rust binding fallback)
# =============================================================================

# std_msgs package
class std_msgs:
    if _BINDINGS_AVAILABLE:
        String = _RustString
        Int32 = _RustInt32
        Int64 = _RustInt64
        Float32 = _RustFloat32
        Float64 = _RustFloat64
        Bool = _RustBool
        Header = _RustHeader
    else:
        # Python stubs for when Rust bindings are not available
        class String:
            def __init__(self, data: str = ""):
                self.data = data
        
        class Int32:
            def __init__(self, data: int = 0):
                self.data = data
        
        class Int64:
            def __init__(self, data: int = 0):
                self.data = data
                
        class Float32:
            def __init__(self, data: float = 0.0):
                self.data = data
        
        class Float64:
            def __init__(self, data: float = 0.0):
                self.data = data
        
        class Bool:
            def __init__(self, data: bool = False):
                self.data = data
        
        class Header:
            def __init__(self):
                self.stamp_sec = 0
                self.stamp_nanosec = 0
                self.frame_id = ""

# geometry_msgs package
class geometry_msgs:
    if _BINDINGS_AVAILABLE:
        Point = _RustPoint
        Vector3 = _RustVector3
        Quaternion = _RustQuaternion
        Pose = _RustPose
        PoseStamped = _RustPoseStamped
        Twist = _RustTwist
    else:
        # Python stubs
        class Point:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
        
        class Vector3:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
        
        class Quaternion:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.w = 1.0
        
        class Pose:
            def __init__(self):
                self.position = geometry_msgs.Point()
                self.orientation = geometry_msgs.Quaternion()
        
        class PoseStamped:
            def __init__(self):
                self.header = std_msgs.Header()
                self.pose = geometry_msgs.Pose()
        
        class Twist:
            def __init__(self):
                self.linear = geometry_msgs.Vector3()
                self.angular = geometry_msgs.Vector3()

# nav_msgs package
class nav_msgs:
    if _BINDINGS_AVAILABLE:
        Odometry = _RustOdometry
    else:
        # Python stub
        class Odometry:
            def __init__(self):
                self.header = std_msgs.Header()
                self.child_frame_id = ""
                self.pose = geometry_msgs.Pose()
                self.twist = geometry_msgs.Twist()

# =============================================================================
# Backward Compatibility Aliases
# =============================================================================

# Direct message type aliases for backward compatibility
if _BINDINGS_AVAILABLE:
    String = _RustString
    StringMessage = _RustStringMessage
    Float64 = _RustFloat64
    Float64Message = _RustFloat64
    Int32 = _RustInt32
    Int32Message = _RustInt32
    Bool = _RustBool
    BoolMessage = _RustBool
    Header = _RustHeader
    
    Point = _RustPoint
    Vector3 = _RustVector3
    Quaternion = _RustQuaternion
    Pose = _RustPose
    PoseMessage = _RustPoseMessage
    PoseStamped = _RustPoseStamped
    Twist = _RustTwist
    TwistMessage = _RustTwistMessage
    
    Odometry = _RustOdometry
    OdometryMessage = _RustOdometryMessage
    
    VisualizationClient = _RustVisualizationClient
else:
    # Use stub classes
    String = std_msgs.String
    StringMessage = std_msgs.String
    Float64 = std_msgs.Float64
    Float64Message = std_msgs.Float64
    Int32 = std_msgs.Int32
    Int32Message = std_msgs.Int32
    Bool = std_msgs.Bool
    BoolMessage = std_msgs.Bool
    Header = std_msgs.Header
    
    Point = geometry_msgs.Point
    Vector3 = geometry_msgs.Vector3
    Quaternion = geometry_msgs.Quaternion
    Pose = geometry_msgs.Pose
    PoseMessage = geometry_msgs.Pose
    PoseStamped = geometry_msgs.PoseStamped
    Twist = geometry_msgs.Twist
    TwistMessage = geometry_msgs.Twist
    
    Odometry = nav_msgs.Odometry
    OdometryMessage = nav_msgs.Odometry
    
    # Visualization stub
    class VisualizationClient:
        def __init__(self, application_id="miniROS", spawn_viewer=False):
            print(f"[STUB] VisualizationClient: {application_id}")
        def log_scalar(self, entity_path, value):
            print(f"[STUB] log_scalar: {entity_path} = {value}")
        def log_points(self, entity_path, points):
            print(f"[STUB] log_points: {entity_path} = {len(points)} points")

# =============================================================================
# Message Bus for Demonstration
# =============================================================================

# Simple message bus for demonstration
class MessageBus:
    def __init__(self):
        self.subscribers = {}  # topic -> list of (callback, msg_type)
        self.lock = threading.Lock()
    
    def subscribe(self, topic: str, callback: Callable, msg_type: str):
        with self.lock:
            if topic not in self.subscribers:
                self.subscribers[topic] = []
            self.subscribers[topic].append((callback, msg_type))
    
    def publish(self, topic: str, data: Any, msg_type: str):
        with self.lock:
            if topic in self.subscribers:
                for callback, sub_msg_type in self.subscribers[topic]:
                    if sub_msg_type == msg_type:
                        try:
                            callback(data)
                        except Exception as e:
                            print(f"Error in subscriber callback: {e}")

# Global message bus instance
_message_bus = MessageBus()

# =============================================================================
# Python Stubs for Missing Rust Bindings  
# =============================================================================

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

# =============================================================================
# Core API Functions
# =============================================================================

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
        """Get the name of this node"""
        return self._rust_node.get_name()

    def get_namespace(self) -> str:
        """Get the namespace of this node (always '/' for miniROS)"""
        return "/"

    def get_logger(self):
        """Get a logger for this node"""
        node_name = self._node_name
        class SimpleLogger:
            def info(self, msg): print(f"[INFO] [{node_name}]: {msg}")
            def warn(self, msg): print(f"[WARN] [{node_name}]: {msg}")
            def error(self, msg): print(f"[ERROR] [{node_name}]: {msg}")
            def debug(self, msg): print(f"[DEBUG] [{node_name}]: {msg}")
        return SimpleLogger()

    def create_publisher(self, msg_type, topic: str, qos_profile=10, *, callback_group=None, event_callbacks=None):
        """
        Create a publisher for the given message type and topic
        
        Args:
            msg_type: Message type class
            topic: Topic name to publish to
            qos_profile: QoS profile (unused, for compatibility)
            callback_group: Callback group (unused, for compatibility)
            event_callbacks: Event callbacks (unused, for compatibility)
            
        Returns:
            Publisher: A publisher instance
        """
        rust_publisher = self._rust_node.create_publisher(msg_type, topic, qos_profile)
        
        # Get the message type name for registration
        msg_type_name = getattr(msg_type, '__name__', str(msg_type))
        if hasattr(msg_type, '__module__'):
            msg_type_name = f"{msg_type.__module__}.{msg_type_name}"
            
        publisher = Publisher(rust_publisher, topic, msg_type_name)
        self._publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic: str, callback: Callable, qos_profile=10, *, 
                          callback_group=None, event_callbacks=None, raw=False):
        """
        Create a subscription for the given message type and topic
        
        Args:
            msg_type: Message type class
            topic: Topic name to subscribe to
            callback: Callback function to handle received messages
            qos_profile: QoS profile (unused, for compatibility)
            callback_group: Callback group (unused, for compatibility) 
            event_callbacks: Event callbacks (unused, for compatibility)
            raw: Raw message mode (unused, for compatibility)
            
        Returns:
            Subscription: A subscription instance
        """
        rust_subscription = self._rust_node.create_subscription(msg_type, topic, callback, qos_profile)
        
        # Get the message type name for registration
        msg_type_name = getattr(msg_type, '__name__', str(msg_type))  
        if hasattr(msg_type, '__module__'):
            msg_type_name = f"{msg_type.__module__}.{msg_type_name}"
            
        subscription = Subscription(rust_subscription, topic, msg_type_name, callback)
        self._subscriptions.append(subscription)
        
        # Register with message bus for demo
        _message_bus.subscribe(topic, callback, msg_type_name)
        
        return subscription

    def destroy_node(self):
        """Destroy this node and cleanup resources"""
        self._rust_node.destroy_node()
        self._subscriptions.clear()
        self._publishers.clear()


class Publisher:
    """A publisher for sending messages to a topic"""
    
    def __init__(self, rust_publisher, topic: str, msg_type: str):
        self._rust_publisher = rust_publisher
        self._topic = topic
        self._msg_type = msg_type

    def publish(self, msg):
        """
        Publish a message to this topic
        
        Args:
            msg: Message instance to publish
        """
        # Publish through rust binding
        self._rust_publisher.publish(msg)
        
        # Also publish through message bus for demo
        _message_bus.publish(self._topic, msg, self._msg_type)

    def get_subscription_count(self) -> int:
        """Get the number of subscribers to this topic"""
        return self._rust_publisher.get_subscription_count()


class Subscription:
    """A subscription for receiving messages from a topic"""
    
    def __init__(self, rust_subscription, topic: str, msg_type: str, callback: Callable):
        self._rust_subscription = rust_subscription
        self._topic = topic
        self._msg_type = msg_type
        self._callback = callback


def spin(node: Node, *, executor=None, context=None):
    """
    Spin a node (process callbacks) indefinitely
    
    Args:
        node: Node to spin
        executor: Executor (unused, for compatibility)
        context: Context (unused, for compatibility)
    """
    try:
        _rust_spin(node._rust_node)
    except KeyboardInterrupt:
        pass


def spin_once(node: Node, *, executor=None, timeout_sec=None, context=None):
    """
    Spin a node once (process callbacks once)
    
    Args:
        node: Node to spin
        executor: Executor (unused, for compatibility)
        timeout_sec: Timeout in seconds
        context: Context (unused, for compatibility)
    """
    timeout_ms = int(timeout_sec * 1000) if timeout_sec else None
    _rust_spin_once(node._rust_node, timeout_ms)


# Export main API
__all__ = [
    'init', 'shutdown', 'ok', 'spin', 'spin_once',
    'Node', 'Publisher', 'Subscription',
    'String', 'Float64', 'Int32',
    'StringMessage', 'Float64Message', 'Int32Message'
] 