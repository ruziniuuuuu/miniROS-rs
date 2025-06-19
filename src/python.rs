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
            // Try to extract as StringMessage first
            if let Ok(string_msg) = data.extract::<StringMessage>(py) {
                println!(
                    "Publishing StringMessage to {}: {}",
                    self.topic, string_msg.data
                );
                Ok(())
            }
            // Try to extract as a plain string
            else if let Ok(string_data) = data.extract::<String>(py) {
                println!("Publishing String to {}: {}", self.topic, string_data);
                Ok(())
            }
            // Try other basic types
            else if let Ok(int_data) = data.extract::<i32>(py) {
                println!("Publishing Int32 to {}: {}", self.topic, int_data);
                Ok(())
            } else if let Ok(float_data) = data.extract::<f64>(py) {
                println!("Publishing Float64 to {}: {}", self.topic, float_data);
                Ok(())
            } else {
                println!("Publishing unknown type to {}: {:?}", self.topic, data);
                Ok(())
            }
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
                let py_msg = StringMessage::new(data);
                if let Err(e) = callback.call1(py, (py_msg,)) {
                    eprintln!("Error calling Python callback: {}", e);
                }
            });
        }
        Ok(())
    }
}

/// Python message types
#[pyclass]
#[derive(Clone)]
pub struct StringMessage {
    #[pyo3(get, set)]
    pub data: String,
}

#[pymethods]
impl StringMessage {
    #[new]
    fn new(data: String) -> Self {
        StringMessage { data }
    }
}

/// Python wrapper for Pose message
#[pyclass]
#[derive(Clone)]
pub struct PyPoseMessage {
    #[pyo3(get, set)]
    pub position: [f32; 3],
    #[pyo3(get, set)]
    pub orientation: [f32; 4],
    #[pyo3(get, set)]
    pub frame_id: String,
}

#[pymethods]
impl PyPoseMessage {
    #[new]
    fn new() -> Self {
        PyPoseMessage {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0], // Unit quaternion
            frame_id: "".to_string(),
        }
    }

    /// Validate the pose (especially quaternion normalization)
    fn validate(&self) -> PyResult<bool> {
        // Check position for finite values
        for &coord in &self.position {
            if !coord.is_finite() {
                return Ok(false);
            }
        }

        // Check quaternion normalization
        let norm = (self.orientation[0].powi(2)
            + self.orientation[1].powi(2)
            + self.orientation[2].powi(2)
            + self.orientation[3].powi(2))
        .sqrt();

        if (norm - 1.0).abs() > 0.1 {
            // Allow some tolerance
            return Ok(false);
        }

        Ok(true)
    }
}

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
    m.add_class::<Node>()?;
    m.add_class::<Publisher>()?;
    m.add_class::<Subscription>()?;
    m.add_class::<StringMessage>()?;
    m.add_class::<Logger>()?;
    m.add_class::<PyVisualizationClient>()?;

    // Add message types as module attributes
    m.add("StringMessage", m.py().get_type::<StringMessage>())?;
    m.add("String", m.py().get_type::<StringMessage>())?; // For backward compatibility
    m.add("PoseMessage", m.py().get_type::<PyPoseMessage>())?;
    m.add("Pose", m.py().get_type::<PyPoseMessage>())?; // For backward compatibility
    m.add(
        "VisualizationClient",
        m.py().get_type::<PyVisualizationClient>(),
    )?; // Expose as VisualizationClient

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
