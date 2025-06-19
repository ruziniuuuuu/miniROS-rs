//! Parameter system for dynamic configuration
//!
//! This module provides parameter management capabilities similar to ROS2,
//! allowing runtime configuration of nodes and services.

use crate::error::{MiniRosError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Parameter value types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ParameterValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
    StringArray(Vec<String>),
    IntArray(Vec<i64>),
    FloatArray(Vec<f64>),
    BoolArray(Vec<bool>),
}

impl ParameterValue {
    /// Get the type name of this parameter value
    pub fn type_name(&self) -> &'static str {
        match self {
            ParameterValue::Bool(_) => "bool",
            ParameterValue::Int(_) => "int",
            ParameterValue::Float(_) => "float",
            ParameterValue::String(_) => "string",
            ParameterValue::StringArray(_) => "string_array",
            ParameterValue::IntArray(_) => "int_array",
            ParameterValue::FloatArray(_) => "float_array",
            ParameterValue::BoolArray(_) => "bool_array",
        }
    }
}

/// Parameter descriptor with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterDescriptor {
    pub name: String,
    pub description: String,
    pub read_only: bool,
    pub value: ParameterValue,
}

/// Simple parameter storage
type ParameterStore = Arc<RwLock<HashMap<String, ParameterValue>>>;

/// Parameter server for managing configuration
#[derive(Clone)]
pub struct ParameterServer {
    parameters: ParameterStore,
}

impl ParameterServer {
    /// Create a new parameter server
    pub fn new() -> Self {
        Self {
            parameters: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Set a parameter value
    pub fn set_parameter(&self, name: &str, value: ParameterValue) -> Result<()> {
        let mut params = self
            .parameters
            .write()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        params.insert(name.to_string(), value);
        tracing::debug!("Set parameter: {} = {:?}", name, params.get(name));
        Ok(())
    }

    /// Get a parameter value
    pub fn get_parameter(&self, name: &str) -> Result<Option<ParameterValue>> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.get(name).cloned())
    }

    /// List all parameter names
    pub fn list_parameters(&self) -> Result<Vec<String>> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.keys().cloned().collect())
    }

    /// Check if parameter exists
    pub fn has_parameter(&self, name: &str) -> Result<bool> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.contains_key(name))
    }

    /// Delete a parameter
    pub fn delete_parameter(&self, name: &str) -> Result<bool> {
        let mut params = self
            .parameters
            .write()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.remove(name).is_some())
    }

    /// Get all parameters as a map
    pub fn get_all_parameters(&self) -> Result<HashMap<String, ParameterValue>> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.clone())
    }
}

impl Default for ParameterServer {
    fn default() -> Self {
        Self::new()
    }
}

/// Parameter client for accessing configuration
#[derive(Clone)]
pub struct ParameterClient {
    parameters: ParameterStore,
}

impl ParameterClient {
    /// Create a new parameter client
    pub fn new() -> Self {
        Self {
            parameters: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create a parameter client connected to a server
    pub fn from_server(server: &ParameterServer) -> Self {
        Self {
            parameters: server.parameters.clone(),
        }
    }

    /// Get a parameter value
    pub fn get_parameter(&self, name: &str) -> Result<Option<ParameterValue>> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.get(name).cloned())
    }

    /// List all parameters
    pub fn list_parameters(&self) -> Result<HashMap<String, ParameterValue>> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.clone())
    }

    /// Check if parameter exists
    pub fn has_parameter(&self, name: &str) -> Result<bool> {
        let params = self
            .parameters
            .read()
            .map_err(|_| MiniRosError::Custom("Failed to acquire parameter lock".to_string()))?;

        Ok(params.contains_key(name))
    }
}

impl Default for ParameterClient {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parameter_server_basic() {
        let server = ParameterServer::new();

        // Set parameters
        assert!(
            server
                .set_parameter("test.bool", ParameterValue::Bool(true))
                .is_ok()
        );
        assert!(
            server
                .set_parameter("test.int", ParameterValue::Int(42))
                .is_ok()
        );
        assert!(
            server
                .set_parameter("test.float", ParameterValue::Float(3.14))
                .is_ok()
        );
        assert!(
            server
                .set_parameter("test.string", ParameterValue::String("hello".to_string()))
                .is_ok()
        );

        // Get parameters
        assert_eq!(
            server.get_parameter("test.bool").unwrap(),
            Some(ParameterValue::Bool(true))
        );
        assert_eq!(
            server.get_parameter("test.int").unwrap(),
            Some(ParameterValue::Int(42))
        );
        assert_eq!(
            server.get_parameter("test.float").unwrap(),
            Some(ParameterValue::Float(3.14))
        );
        assert_eq!(
            server.get_parameter("test.string").unwrap(),
            Some(ParameterValue::String("hello".to_string()))
        );

        // Non-existent parameter
        assert_eq!(server.get_parameter("non.existent").unwrap(), None);
    }

    #[test]
    fn test_parameter_client() {
        let server = ParameterServer::new();
        server
            .set_parameter("test.value", ParameterValue::Int(123))
            .unwrap();

        let client = ParameterClient::from_server(&server);
        assert_eq!(
            client.get_parameter("test.value").unwrap(),
            Some(ParameterValue::Int(123))
        );
    }

    #[test]
    fn test_parameter_operations() {
        let server = ParameterServer::new();

        // Test parameter existence
        assert!(!server.has_parameter("test.param").unwrap());

        server
            .set_parameter("test.param", ParameterValue::String("value".to_string()))
            .unwrap();
        assert!(server.has_parameter("test.param").unwrap());

        // Test parameter deletion
        assert!(server.delete_parameter("test.param").unwrap());
        assert!(!server.has_parameter("test.param").unwrap());

        // Test deleting non-existent parameter
        assert!(!server.delete_parameter("non.existent").unwrap());
    }

    #[test]
    fn test_parameter_arrays() {
        let server = ParameterServer::new();

        let string_array = vec!["a".to_string(), "b".to_string(), "c".to_string()];
        let int_array = vec![1, 2, 3];
        let float_array = vec![1.1, 2.2, 3.3];
        let bool_array = vec![true, false, true];

        server
            .set_parameter(
                "arrays.strings",
                ParameterValue::StringArray(string_array.clone()),
            )
            .unwrap();
        server
            .set_parameter("arrays.ints", ParameterValue::IntArray(int_array.clone()))
            .unwrap();
        server
            .set_parameter(
                "arrays.floats",
                ParameterValue::FloatArray(float_array.clone()),
            )
            .unwrap();
        server
            .set_parameter(
                "arrays.bools",
                ParameterValue::BoolArray(bool_array.clone()),
            )
            .unwrap();

        assert_eq!(
            server.get_parameter("arrays.strings").unwrap(),
            Some(ParameterValue::StringArray(string_array))
        );
        assert_eq!(
            server.get_parameter("arrays.ints").unwrap(),
            Some(ParameterValue::IntArray(int_array))
        );
        assert_eq!(
            server.get_parameter("arrays.floats").unwrap(),
            Some(ParameterValue::FloatArray(float_array))
        );
        assert_eq!(
            server.get_parameter("arrays.bools").unwrap(),
            Some(ParameterValue::BoolArray(bool_array))
        );
    }
}
