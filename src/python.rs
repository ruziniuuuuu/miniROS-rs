//! Python bindings for miniROS-rs
//! 
//! This module provides Python API that mimics ROS2 rclpy syntax

use pyo3::prelude::*;
use std::sync::Arc;
use tokio::sync::Mutex;

use crate::{
    node::Node as RustNode,
    message::{StringMsg, Float64Msg, Int32Msg},
    publisher::Publisher as RustPublisher,
    subscriber::Subscriber as RustSubscriber,
};

/// Python wrapper for Node
#[pyclass]
pub struct Node {
    inner: Arc<Mutex<Option<RustNode>>>,
    name: String,
}

#[pymethods]
impl Node {
    #[new]
    fn new(name: String) -> PyResult<Self> {
        let rust_node = RustNode::new(&name)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        
        Ok(Node {
            inner: Arc::new(Mutex::new(Some(rust_node))),
            name,
        })
    }

    /// Get node name
    #[getter]
    fn get_name(&self) -> String {
        self.name.clone()
    }

    /// Destroy the node (cleanup resources)
    fn destroy_node(&self) {
        // Implementation for cleanup if needed
    }
}

/// Python wrapper for Publisher
#[pyclass]
pub struct Publisher {
    msg_type: String,
    topic: String,
}

#[pymethods]
impl Publisher {
    #[new]
    fn new(msg_type: String, topic: String) -> Self {
        Publisher { msg_type, topic }
    }

    /// Publish a message
    fn publish(&self, data: PyObject) -> PyResult<()> {
        // For now, just log the message
        println!("Publishing {} to {}: {:?}", self.msg_type, self.topic, data);
        Ok(())
    }
    
    fn get_subscription_count(&self) -> i32 {
        0
    }
}

/// Python wrapper for Subscription
#[pyclass]
pub struct Subscription {
    msg_type: String,
    topic: String,
}

#[pymethods]
impl Subscription {
    #[new]
    fn new(msg_type: String, topic: String) -> Self {
        Subscription { msg_type, topic }
    }
}

/// Initialize function for Python module
pub fn init_python_module(m: &PyModule) -> PyResult<()> {
    m.add_class::<Node>()?;
    m.add_class::<Publisher>()?;
    m.add_class::<Subscription>()?;
    
    // Add utility functions
    m.add_function(wrap_pyfunction!(init, m)?)?;
    m.add_function(wrap_pyfunction!(shutdown, m)?)?;
    m.add_function(wrap_pyfunction!(spin_once, m)?)?;
    m.add_function(wrap_pyfunction!(spin, m)?)?;
    
    Ok(())
}

/// Initialize miniROS (equivalent to rclpy.init())
#[pyfunction]
fn init() -> PyResult<()> {
    // Initialize tracing for logging
    tracing_subscriber::fmt::init();
    Ok(())
}

/// Shutdown miniROS (equivalent to rclpy.shutdown())
#[pyfunction] 
fn shutdown() -> PyResult<()> {
    // Cleanup logic here
    Ok(())
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
    }
} 