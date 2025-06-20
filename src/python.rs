//! Python bindings for miniROS-rs
//!
//! This module provides Python API that mimics ROS2 rclpy syntax

// PyO3 macro limitations in Rust 2024 edition:
// - non_local_definitions: PyO3 macros generate impl blocks that appear non-local
// - unsafe_op_in_unsafe_fn: PyO3 argument parsing uses unsafe functions in macros
// These are known limitations of PyO3 0.20 with Rust 2024 edition
// Future PyO3 versions are expected to resolve these issues
#![allow(non_local_definitions)]
#![allow(unsafe_op_in_unsafe_fn)]

use pyo3::prelude::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Python wrapper for Node
#[pyclass]
pub struct Node {
    name: String,
    publishers: Arc<Mutex<HashMap<String, String>>>,
    subscribers: Arc<Mutex<HashMap<String, String>>>,
}

#[pymethods]
impl Node {
    #[new]
    fn new(name: String) -> PyResult<Self> {
        Ok(Node {
            name: name.clone(),
            publishers: Arc::new(Mutex::new(HashMap::new())),
            subscribers: Arc::new(Mutex::new(HashMap::new())),
        })
    }

    /// Create a publisher
    fn create_publisher(
        &self,
        _msg_type: PyObject,
        topic: String,
        _qos_depth: i32,
    ) -> PyResult<Publisher> {
        {
            let mut pubs = self.publishers.lock().unwrap();
            pubs.insert(topic.clone(), "StringMsg".to_string());
        }

        Ok(Publisher {
            topic: topic.clone(),
            node_name: self.name.clone(),
        })
    }

    /// Create a subscription
    fn create_subscription(
        &self,
        _msg_type: PyObject,
        topic: String,
        callback: PyObject,
        _qos_depth: i32,
    ) -> PyResult<Subscription> {
        {
            let mut subs = self.subscribers.lock().unwrap();
            subs.insert(topic.clone(), "StringMsg".to_string());
        }

        Ok(Subscription {
            topic: topic.clone(),
            callback: Some(callback),
            node_name: self.name.clone(),
        })
    }

    /// Get node name
    #[getter]
    fn get_name(&self) -> String {
        self.name.clone()
    }

    /// Get logger (simplified)
    fn get_logger(&self) -> Logger {
        Logger::new(self.name.clone())
    }

    /// Destroy the node (cleanup resources)
    fn destroy_node(&self) {
        // Simplified cleanup
        self.publishers.lock().unwrap().clear();
        self.subscribers.lock().unwrap().clear();
    }
}

/// Python wrapper for Publisher
#[pyclass]
pub struct Publisher {
    #[allow(dead_code)] // Field is reserved for future functionality
    topic: String,
    #[allow(dead_code)] // Field is reserved for future functionality
    node_name: String,
}

#[pymethods]
impl Publisher {
    #[new]
    fn new(_msg_type: String, topic: String) -> Self {
        Publisher {
            topic,
            node_name: "python_node".to_string(),
        }
    }

    /// Publish a message
    fn publish(&self, data: PyObject) -> PyResult<()> {
        Python::with_gil(|py| {
            // Try to extract different message types
            if let Ok(string_msg) = data.extract::<PyStringMessage>(py) {
                println!("Publishing String to {}: {}", self.topic, string_msg.data);
            }
            else if let Ok(int32_msg) = data.extract::<PyInt32Message>(py) {
                println!("Publishing Int32 to {}: {}", self.topic, int32_msg.data);
            }
            else if let Ok(float64_msg) = data.extract::<PyFloat64Message>(py) {
                println!("Publishing Float64 to {}: {}", self.topic, float64_msg.data);
            }
            else if let Ok(bool_msg) = data.extract::<PyBoolMessage>(py) {
                println!("Publishing Bool to {}: {}", self.topic, bool_msg.data);
            }
            else if let Ok(pose_msg) = data.extract::<PyPoseMessage>(py) {
                println!("Publishing Pose to {}: pos=[{}, {}, {}]", 
                    self.topic, pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
            }
            else if let Ok(twist_msg) = data.extract::<PyTwistMessage>(py) {
                println!("Publishing Twist to {}: linear=[{}, {}, {}]", 
                    self.topic, twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z);
            }
            else if let Ok(odometry_msg) = data.extract::<PyOdometryMessage>(py) {
                println!("Publishing Odometry to {}: frame_id={}", 
                    self.topic, odometry_msg.header.frame_id);
            }
            else if let Ok(string_data) = data.extract::<String>(py) {
                println!("Publishing String to {}: {}", self.topic, string_data);
            }
            else {
                println!("Publishing unknown type to {}: {:?}", self.topic, data);
            }
            Ok(())
        })
    }

    fn get_subscription_count(&self) -> i32 {
        0
    }
}

/// Python wrapper for Subscription
#[pyclass]
pub struct Subscription {
    #[allow(dead_code)] // Field is reserved for future functionality
    topic: String,
    callback: Option<PyObject>,
    #[allow(dead_code)] // Field is reserved for future functionality
    node_name: String,
}

#[pymethods]
impl Subscription {
    #[new]
    fn new(_msg_type: String, topic: String) -> Self {
        Subscription {
            topic,
            callback: None,
            node_name: "python_node".to_string(),
        }
    }

    /// Simulate receiving a message (for testing)
    fn _simulate_message(&self, data: String) -> PyResult<()> {
        if let Some(ref callback) = self.callback {
            Python::with_gil(|py| {
                let py_msg = PyStringMessage::new(data);
                if let Err(e) = callback.call1(py, (py_msg,)) {
                    eprintln!("Error calling Python callback: {}", e);
                }
            });
        }
        Ok(())
    }
}

// =============================================================================
// Python Message Types - std_msgs
// =============================================================================

/// Python wrapper for String message
#[pyclass]
#[derive(Clone)]
pub struct PyStringMessage {
    #[pyo3(get, set)]
    pub data: String,
}

#[pymethods]
impl PyStringMessage {
    #[new]
    #[pyo3(signature = (data = String::new()))]
    fn new(data: String) -> Self {
        PyStringMessage { data }
    }
}

/// Python wrapper for Int32 message
#[pyclass]
#[derive(Clone)]
pub struct PyInt32Message {
    #[pyo3(get, set)]
    pub data: i32,
}

#[pymethods]
impl PyInt32Message {
    #[new]
    #[pyo3(signature = (data = 0))]
    fn new(data: i32) -> Self {
        PyInt32Message { data }
    }
}

/// Python wrapper for Int64 message
#[pyclass]
#[derive(Clone)]
pub struct PyInt64Message {
    #[pyo3(get, set)]
    pub data: i64,
}

#[pymethods]
impl PyInt64Message {
    #[new]
    #[pyo3(signature = (data = 0))]
    fn new(data: i64) -> Self {
        PyInt64Message { data }
    }
}

/// Python wrapper for Float32 message
#[pyclass]
#[derive(Clone)]
pub struct PyFloat32Message {
    #[pyo3(get, set)]
    pub data: f32,
}

#[pymethods]
impl PyFloat32Message {
    #[new]
    #[pyo3(signature = (data = 0.0))]
    fn new(data: f32) -> Self {
        PyFloat32Message { data }
    }
}

/// Python wrapper for Float64 message
#[pyclass]
#[derive(Clone)]
pub struct PyFloat64Message {
    #[pyo3(get, set)]
    pub data: f64,
}

#[pymethods]
impl PyFloat64Message {
    #[new]
    #[pyo3(signature = (data = 0.0))]
    fn new(data: f64) -> Self {
        PyFloat64Message { data }
    }
}

/// Python wrapper for Bool message
#[pyclass]
#[derive(Clone)]
pub struct PyBoolMessage {
    #[pyo3(get, set)]
    pub data: bool,
}

#[pymethods]
impl PyBoolMessage {
    #[new]
    #[pyo3(signature = (data = false))]
    fn new(data: bool) -> Self {
        PyBoolMessage { data }
    }
}

/// Python wrapper for Header message
#[pyclass]
#[derive(Clone)]
pub struct PyHeaderMessage {
    #[pyo3(get, set)]
    pub stamp_sec: u32,
    #[pyo3(get, set)]
    pub stamp_nanosec: u32,
    #[pyo3(get, set)]
    pub frame_id: String,
}

#[pymethods]
impl PyHeaderMessage {
    #[new]
    fn new() -> Self {
        PyHeaderMessage {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
        }
    }
}

// =============================================================================
// Python Message Types - geometry_msgs
// =============================================================================

/// Python wrapper for Point message
#[pyclass]
#[derive(Clone)]
pub struct PyPointMessage {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
}

#[pymethods]
impl PyPointMessage {
    #[new]
    fn new() -> Self {
        PyPointMessage { x: 0.0, y: 0.0, z: 0.0 }
    }
}

/// Python wrapper for Vector3 message
#[pyclass]
#[derive(Clone)]
pub struct PyVector3Message {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
}

#[pymethods]
impl PyVector3Message {
    #[new]
    fn new() -> Self {
        PyVector3Message { x: 0.0, y: 0.0, z: 0.0 }
    }
}

/// Python wrapper for Quaternion message
#[pyclass]
#[derive(Clone)]
pub struct PyQuaternionMessage {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
    #[pyo3(get, set)]
    pub w: f64,
}

#[pymethods]
impl PyQuaternionMessage {
    #[new]
    fn new() -> Self {
        PyQuaternionMessage { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
    
    fn normalize(&mut self) {
        let norm = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if norm > 0.0 {
            self.x /= norm;
            self.y /= norm;
            self.z /= norm;
            self.w /= norm;
        }
    }
}

/// Python wrapper for Pose message
#[pyclass]
#[derive(Clone)]
pub struct PyPoseMessage {
    #[pyo3(get, set)]
    pub position: PyPointMessage,
    #[pyo3(get, set)]
    pub orientation: PyQuaternionMessage,
}

#[pymethods]
impl PyPoseMessage {
    #[new]
    fn new() -> Self {
        PyPoseMessage {
            position: PyPointMessage::new(),
            orientation: PyQuaternionMessage::new(),
        }
    }
    
    fn validate(&self) -> PyResult<bool> {
        // Check if quaternion is normalized
        let norm = (self.orientation.x * self.orientation.x + 
                   self.orientation.y * self.orientation.y + 
                   self.orientation.z * self.orientation.z + 
                   self.orientation.w * self.orientation.w).sqrt();
        Ok((norm - 1.0).abs() < 1e-6)
    }
}

/// Python wrapper for PoseStamped message
#[pyclass]
#[derive(Clone)]
pub struct PyPoseStampedMessage {
    #[pyo3(get, set)]
    pub header: PyHeaderMessage,
    #[pyo3(get, set)]
    pub pose: PyPoseMessage,
}

#[pymethods]
impl PyPoseStampedMessage {
    #[new]
    fn new() -> Self {
        PyPoseStampedMessage {
            header: PyHeaderMessage::new(),
            pose: PyPoseMessage::new(),
        }
    }
}

/// Python wrapper for Twist message
#[pyclass]
#[derive(Clone)]
pub struct PyTwistMessage {
    #[pyo3(get, set)]
    pub linear: PyVector3Message,
    #[pyo3(get, set)]
    pub angular: PyVector3Message,
}

#[pymethods]
impl PyTwistMessage {
    #[new]
    fn new() -> Self {
        PyTwistMessage {
            linear: PyVector3Message::new(),
            angular: PyVector3Message::new(),
        }
    }
    
    fn validate(&self) -> PyResult<bool> {
        // Check velocity limits
        let max_linear = 10.0; // m/s
        let max_angular = 2.0 * 3.14159; // rad/s
        
        let linear_speed = (self.linear.x * self.linear.x + 
                          self.linear.y * self.linear.y + 
                          self.linear.z * self.linear.z).sqrt();
        let angular_speed = (self.angular.x * self.angular.x + 
                           self.angular.y * self.angular.y + 
                           self.angular.z * self.angular.z).sqrt();
        
        Ok(linear_speed <= max_linear && angular_speed <= max_angular)
    }
    
    fn set_linear_xyz(&mut self, x: f64, y: f64, z: f64) {
        self.linear.x = x;
        self.linear.y = y;
        self.linear.z = z;
    }
    
    fn set_angular_xyz(&mut self, x: f64, y: f64, z: f64) {
        self.angular.x = x;
        self.angular.y = y;
        self.angular.z = z;
    }
}

// =============================================================================
// Python Message Types - nav_msgs
// =============================================================================

/// Python wrapper for Odometry message
#[pyclass]
#[derive(Clone)]
pub struct PyOdometryMessage {
    #[pyo3(get, set)]
    pub header: PyHeaderMessage,
    #[pyo3(get, set)]
    pub child_frame_id: String,
    #[pyo3(get, set)]
    pub pose: PyPoseMessage,
    #[pyo3(get, set)]
    pub twist: PyTwistMessage,
}

#[pymethods]
impl PyOdometryMessage {
    #[new]
    fn new() -> Self {
        PyOdometryMessage {
            header: PyHeaderMessage::new(),
            child_frame_id: String::new(),
            pose: PyPoseMessage::new(),
            twist: PyTwistMessage::new(),
        }
    }
    
    fn get_yaw(&self) -> f64 {
        // Extract yaw from quaternion
        let q = &self.pose.orientation;
        2.0 * (q.w * q.z + q.x * q.y).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    }
    
    fn set_pose_2d(&mut self, x: f64, y: f64, yaw: f64) {
        self.pose.position.x = x;
        self.pose.position.y = y;
        self.pose.position.z = 0.0;
        
        let half_yaw = yaw / 2.0;
        self.pose.orientation.x = 0.0;
        self.pose.orientation.y = 0.0;
        self.pose.orientation.z = half_yaw.sin();
        self.pose.orientation.w = half_yaw.cos();
    }
}

// =============================================================================
// Logger and Visualization
// =============================================================================

/// Logger for Python
#[pyclass]
pub struct Logger {
    name: String,
}

impl Logger {
    fn new(name: String) -> Self {
        Logger { name }
    }
}

#[pymethods]
impl Logger {
    fn info(&self, message: String) {
        println!("[INFO] [{}]: {}", self.name, message);
    }

    fn warn(&self, message: String) {
        println!("[WARN] [{}]: {}", self.name, message);
    }

    fn error(&self, message: String) {
        println!("[ERROR] [{}]: {}", self.name, message);
    }
}

/// Python wrapper for VisualizationClient
#[pyclass]
pub struct PyVisualizationClient {
    client: Arc<crate::visualization::VisualizationClient>,
}

#[pymethods]
impl PyVisualizationClient {
    #[new]
    #[pyo3(signature = (application_id = "miniROS", spawn_viewer = false))]
    fn new(application_id: &str, spawn_viewer: bool) -> PyResult<Self> {
        let config = crate::visualization::VisualizationConfig {
            application_id: application_id.to_string(),
            spawn_viewer,
        };

        let client = crate::visualization::VisualizationClient::new(config)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(Self {
            client: Arc::new(client),
        })
    }

    /// Log a scalar value for plotting
    #[pyo3(signature = (entity_path, value))]
    fn log_scalar(&self, entity_path: &str, value: f64) -> PyResult<()> {
        self.client
            .log_scalar(entity_path, value)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    /// Log 3D points for visualization
    #[pyo3(signature = (entity_path, points))]
    fn log_points(&self, entity_path: &str, points: Vec<Vec<f32>>) -> PyResult<()> {
        // Convert Vec<Vec<f32>> to Vec<[f32; 3]>
        let points_3d: Vec<[f32; 3]> = points
            .into_iter()
            .filter_map(|p| {
                if p.len() >= 3 {
                    Some([p[0], p[1], p[2]])
                } else {
                    None
                }
            })
            .collect();

        self.client
            .log_points(entity_path, points_3d)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    /// Log a 3D transform (position + rotation)
    #[pyo3(signature = (entity_path, translation, rotation_quat))]
    fn log_transform(
        &self,
        entity_path: &str,
        translation: Vec<f32>,
        rotation_quat: Vec<f32>,
    ) -> PyResult<()> {
        if translation.len() != 3 || rotation_quat.len() != 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Translation must be [x,y,z] and rotation must be [x,y,z,w] quaternion",
            ));
        }

        let trans = [translation[0], translation[1], translation[2]];
        let rot = [
            rotation_quat[0],
            rotation_quat[1],
            rotation_quat[2],
            rotation_quat[3],
        ];

        self.client
            .log_transform(entity_path, trans, rot)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    /// Log text message
    #[pyo3(signature = (entity_path, message))]
    fn log_text(&self, entity_path: &str, message: &str) -> PyResult<()> {
        self.client
            .log_text(entity_path, message)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    /// Set timeline for data logging
    #[pyo3(signature = (timeline, nanos))]
    fn set_time(&self, timeline: &str, nanos: i64) -> PyResult<()> {
        self.client
            .set_time(timeline, nanos)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
}

/// Initialize function for Python module
pub fn init_python_module(m: &PyModule) -> PyResult<()> {
    // Core classes
    m.add_class::<Node>()?;
    m.add_class::<Publisher>()?;
    m.add_class::<Subscription>()?;
    m.add_class::<Logger>()?;
    m.add_class::<PyVisualizationClient>()?;

    // std_msgs
    m.add_class::<PyStringMessage>()?;
    m.add_class::<PyInt32Message>()?;
    m.add_class::<PyInt64Message>()?;
    m.add_class::<PyFloat32Message>()?;
    m.add_class::<PyFloat64Message>()?;
    m.add_class::<PyBoolMessage>()?;
    m.add_class::<PyHeaderMessage>()?;

    // geometry_msgs
    m.add_class::<PyPointMessage>()?;
    m.add_class::<PyVector3Message>()?;
    m.add_class::<PyQuaternionMessage>()?;
    m.add_class::<PyPoseMessage>()?;
    m.add_class::<PyPoseStampedMessage>()?;
    m.add_class::<PyTwistMessage>()?;

    // nav_msgs
    m.add_class::<PyOdometryMessage>()?;

    // Add message types as module attributes with ROS2 naming convention
    // std_msgs
    m.add("String", m.py().get_type::<PyStringMessage>())?;
    m.add("Int32", m.py().get_type::<PyInt32Message>())?;
    m.add("Int64", m.py().get_type::<PyInt64Message>())?;
    m.add("Float32", m.py().get_type::<PyFloat32Message>())?;
    m.add("Float64", m.py().get_type::<PyFloat64Message>())?;
    m.add("Bool", m.py().get_type::<PyBoolMessage>())?;
    m.add("Header", m.py().get_type::<PyHeaderMessage>())?;
    
    // geometry_msgs
    m.add("Point", m.py().get_type::<PyPointMessage>())?;
    m.add("Vector3", m.py().get_type::<PyVector3Message>())?;
    m.add("Quaternion", m.py().get_type::<PyQuaternionMessage>())?;
    m.add("Pose", m.py().get_type::<PyPoseMessage>())?;
    m.add("PoseStamped", m.py().get_type::<PyPoseStampedMessage>())?;
    m.add("Twist", m.py().get_type::<PyTwistMessage>())?;
    
    // nav_msgs
    m.add("Odometry", m.py().get_type::<PyOdometryMessage>())?;

    // Legacy aliases for backward compatibility
    m.add("StringMessage", m.py().get_type::<PyStringMessage>())?;
    m.add("PoseMessage", m.py().get_type::<PyPoseMessage>())?;
    m.add("TwistMessage", m.py().get_type::<PyTwistMessage>())?;
    m.add("OdometryMessage", m.py().get_type::<PyOdometryMessage>())?;
    
    // Visualization
    m.add("VisualizationClient", m.py().get_type::<PyVisualizationClient>())?;

    // Add utility functions
    m.add_function(wrap_pyfunction!(init, m)?)?;
    m.add_function(wrap_pyfunction!(shutdown, m)?)?;
    m.add_function(wrap_pyfunction!(spin_once, m)?)?;
    m.add_function(wrap_pyfunction!(spin, m)?)?;
    m.add_function(wrap_pyfunction!(ok, m)?)?;

    Ok(())
}

/// Initialize miniROS (equivalent to rclpy.init())
#[pyfunction]
fn init() -> PyResult<()> {
    // Initialize tracing for logging
    let _ = tracing_subscriber::fmt::try_init();
    Ok(())
}

/// Shutdown miniROS (equivalent to rclpy.shutdown())
#[pyfunction]
fn shutdown() -> PyResult<()> {
    // Cleanup logic here
    Ok(())
}

/// Check if miniROS is still running
#[pyfunction]
fn ok() -> bool {
    true
}

/// Spin the node once (process callbacks once)
#[pyfunction]
fn spin_once(_node: &Node, timeout_ms: Option<u64>) -> PyResult<()> {
    let timeout = timeout_ms.unwrap_or(100);
    std::thread::sleep(std::time::Duration::from_millis(timeout));
    Ok(())
}

/// Spin the node indefinitely (equivalent to rclpy.spin())
#[pyfunction]
fn spin(_node: &Node) -> PyResult<()> {
    // Keep the node alive indefinitely
    loop {
        std::thread::sleep(std::time::Duration::from_millis(100));
        // Check for interrupts
        Python::with_gil(|py| if py.check_signals().is_err() {});
    }
}
