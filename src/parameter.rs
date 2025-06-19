//! Parameter system for miniROS
//! 
//! Provides a simple parameter server functionality for node configuration.
//! Parameters can be set, get, and monitored for changes.

use std::collections::HashMap;
use std::sync::Arc;
use serde::{Serialize, Deserialize};
use tokio::sync::RwLock;

use crate::error::{Result, MiniRosError};
use crate::{Node, Publisher, Subscriber};
use crate::message::StringMsg;

/// Parameter value types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ParameterValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntArray(Vec<i64>),
    FloatArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParameterValue {
    /// Get the type name as a string
    pub fn type_name(&self) -> &'static str {
        match self {
            ParameterValue::Bool(_) => "bool",
            ParameterValue::Int(_) => "int",
            ParameterValue::Float(_) => "float",
            ParameterValue::String(_) => "string",
            ParameterValue::BoolArray(_) => "bool_array",
            ParameterValue::IntArray(_) => "int_array",
            ParameterValue::FloatArray(_) => "float_array",
            ParameterValue::StringArray(_) => "string_array",
        }
    }

    /// Convert to bool (if possible)
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            ParameterValue::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// Convert to integer (if possible)
    pub fn as_int(&self) -> Option<i64> {
        match self {
            ParameterValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    /// Convert to float (if possible)
    pub fn as_float(&self) -> Option<f64> {
        match self {
            ParameterValue::Float(v) => Some(*v),
            ParameterValue::Int(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Convert to string (if possible)
    pub fn as_string(&self) -> Option<&str> {
        match self {
            ParameterValue::String(v) => Some(v),
            _ => None,
        }
    }
}

/// Parameter descriptor with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterDescriptor {
    pub name: String,
    pub description: String,
    pub read_only: bool,
    pub floating_point_range: Option<(f64, f64)>,
    pub integer_range: Option<(i64, i64)>,
}

impl Default for ParameterDescriptor {
    fn default() -> Self {
        Self {
            name: String::new(),
            description: String::new(),
            read_only: false,
            floating_point_range: None,
            integer_range: None,
        }
    }
}

/// Parameter get request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetParameterRequest {
    pub name: String,
}

/// Parameter get response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetParameterResponse {
    pub value: Option<ParameterValue>,
    pub success: bool,
    pub message: String,
}

/// Parameter set request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetParameterRequest {
    pub name: String,
    pub value: ParameterValue,
}

/// Parameter set response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetParameterResponse {
    pub success: bool,
    pub message: String,
}

/// List parameters request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListParametersRequest {
    pub prefix: String,
}

/// List parameters response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListParametersResponse {
    pub names: Vec<String>,
    pub success: bool,
}

/// Parameter event types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ParameterEvent {
    Set { name: String, value: ParameterValue },
    Delete { name: String },
}

/// Parameter request types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ParameterRequest {
    Get { name: String },
    Set { name: String, value: ParameterValue },
    List,
    Describe { name: String },
}

/// Parameter response types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ParameterResponse {
    Value(Option<ParameterValue>),
    Success(bool),
    List(Vec<String>),
    Description(Option<ParameterDescriptor>),
    Error(String),
}

/// Parameter server that manages runtime configuration
pub struct ParameterServer {
    node_name: String,
    parameters: Arc<RwLock<HashMap<String, ParameterValue>>>,
    descriptors: Arc<RwLock<HashMap<String, ParameterDescriptor>>>,
    request_subscriber: Subscriber<StringMsg>,
    response_publisher: Publisher<StringMsg>,
    event_publisher: Publisher<StringMsg>,
}

impl ParameterServer {
    /// Create a new parameter server
    pub async fn new(node: &mut Node) -> Result<Self> {
        let node_name = node.name().to_string();
        
        // Create topics for parameter communication
        let request_topic = format!("/{}/parameters/request", node_name);
        let response_topic = format!("/{}/parameters/response", node_name);
        let event_topic = format!("/{}/parameters/events", node_name);

        // Create subscribers and publishers
        let request_subscriber = node.create_subscriber(&request_topic).await?;
        let response_publisher = node.create_publisher(&response_topic).await?;
        let event_publisher = node.create_publisher(&event_topic).await?;

        let parameters = Arc::new(RwLock::new(HashMap::<String, ParameterValue>::new()));
        let descriptors = Arc::new(RwLock::new(HashMap::<String, ParameterDescriptor>::new()));

        let server = ParameterServer {
            node_name,
            parameters,
            descriptors,
            request_subscriber,
            response_publisher,
            event_publisher,
        };

        // Set up request handling
        server.setup_request_handling().await?;

        Ok(server)
    }

    /// Set up request handling loop
    async fn setup_request_handling(&self) -> Result<()> {
        let parameters = Arc::clone(&self.parameters);
        let descriptors = Arc::clone(&self.descriptors);
        let response_publisher = self.response_publisher.clone();
        let event_publisher = self.event_publisher.clone();

        self.request_subscriber.on_message(move |msg: StringMsg| {
            let parameters = Arc::clone(&parameters);
            let descriptors = Arc::clone(&descriptors);
            let response_publisher = response_publisher.clone();
            let event_publisher = event_publisher.clone();

            tokio::spawn(async move {
                if let Ok(request) = serde_json::from_str::<ParameterRequest>(&msg.data) {
                    let response = Self::handle_request(request, &parameters, &descriptors, &event_publisher).await;
                    
                    if let Ok(response_msg) = serde_json::to_string(&response) {
                        let string_msg = StringMsg { data: response_msg };
                        let _ = response_publisher.publish(&string_msg).await;
                    }
                }
            });
        })?;

        Ok(())
    }

    /// Handle parameter requests
    async fn handle_request(
        request: ParameterRequest,
        parameters: &Arc<RwLock<HashMap<String, ParameterValue>>>,
        descriptors: &Arc<RwLock<HashMap<String, ParameterDescriptor>>>,
        event_publisher: &Publisher<StringMsg>,
    ) -> ParameterResponse {
        match request {
            ParameterRequest::Get { name } => {
                let params = parameters.read().await;
                ParameterResponse::Value(params.get(&name).cloned())
            }
            ParameterRequest::Set { name, value } => {
                // Check if parameter is read-only
                {
                    let descs = descriptors.read().await;
                    if let Some(descriptor) = descs.get(&name) {
                        if descriptor.read_only {
                            return ParameterResponse::Error("Parameter is read-only".to_string());
                        }
                    }
                }

                // Set the parameter
                {
                    let mut params = parameters.write().await;
                    params.insert(name.clone(), value.clone());
                }

                // Publish event
                let event = ParameterEvent::Set { name, value };
                if let Ok(event_msg) = serde_json::to_string(&event) {
                    let string_msg = StringMsg { data: event_msg };
                    let _ = event_publisher.publish(&string_msg).await;
                }

                ParameterResponse::Success(true)
            }
            ParameterRequest::List => {
                let params = parameters.read().await;
                let names: Vec<String> = params.keys().cloned().collect();
                ParameterResponse::List(names)
            }
            ParameterRequest::Describe { name } => {
                let descs = descriptors.read().await;
                ParameterResponse::Description(descs.get(&name).cloned())
            }
        }
    }

    /// Declare a parameter with descriptor
    pub async fn declare_parameter(
        &self,
        name: &str,
        default_value: ParameterValue,
        descriptor: ParameterDescriptor,
    ) -> Result<()> {
        {
            let mut params = self.parameters.write().await;
            params.insert(name.to_string(), default_value);
        }

        {
            let mut descs = self.descriptors.write().await;
            descs.insert(name.to_string(), descriptor);
        }

        Ok(())
    }

    /// Set a parameter value
    pub async fn set_parameter(&self, name: &str, value: ParameterValue) -> Result<()> {
        // Check if read-only
        {
            let descs = self.descriptors.read().await;
            if let Some(descriptor) = descs.get(name) {
                if descriptor.read_only {
                    return Err(MiniRosError::Other("Parameter is read-only".to_string()));
                }
            }
        }

        // Set the parameter
        {
            let mut params = self.parameters.write().await;
            params.insert(name.to_string(), value.clone());
        }

        // Publish event
        let event = ParameterEvent::Set {
            name: name.to_string(),
            value,
        };
        let event_msg = serde_json::to_string(&event)?;
        let string_msg = StringMsg { data: event_msg };
        self.event_publisher.publish(&string_msg).await?;

        Ok(())
    }

    /// Get a parameter value
    pub async fn get_parameter(&self, name: &str) -> Option<ParameterValue> {
        let params = self.parameters.read().await;
        params.get(name).cloned()
    }

    /// List all parameter names
    pub async fn list_parameters(&self) -> Vec<String> {
        let params = self.parameters.read().await;
        params.keys().cloned().collect()
    }
}

/// Parameter client for accessing remote parameters
pub struct ParameterClient {
    target_node: String,
    request_publisher: Publisher<StringMsg>,
    response_subscriber: Subscriber<StringMsg>,
    event_subscriber: Subscriber<StringMsg>,
}

impl ParameterClient {
    /// Create a new parameter client
    pub async fn new(node: &mut Node, target_node: &str) -> Result<Self> {
        // Create topics for parameter communication
        let request_topic = format!("/{}/parameters/request", target_node);
        let response_topic = format!("/{}/parameters/response", target_node);
        let event_topic = format!("/{}/parameters/events", target_node);

        // Create publisher and subscribers
        let request_publisher = node.create_publisher(&request_topic).await?;
        let response_subscriber = node.create_subscriber(&response_topic).await?;
        let event_subscriber = node.create_subscriber(&event_topic).await?;

        Ok(ParameterClient {
            target_node: target_node.to_string(),
            request_publisher,
            response_subscriber,
            event_subscriber,
        })
    }

    /// Get a parameter value
    pub async fn get_parameter(&self, name: &str) -> Result<Option<ParameterValue>> {
        let request = ParameterRequest::Get {
            name: name.to_string(),
        };
        let request_msg = serde_json::to_string(&request)?;
        let string_msg = StringMsg { data: request_msg };
        self.request_publisher.publish(&string_msg).await?;

        // In a real implementation, we'd wait for the response
        // For simplicity, we'll return None here
        Ok(None)
    }

    /// Set a parameter value
    pub async fn set_parameter(&self, name: &str, value: ParameterValue) -> Result<bool> {
        let request = ParameterRequest::Set {
            name: name.to_string(),
            value,
        };
        let request_msg = serde_json::to_string(&request)?;
        let string_msg = StringMsg { data: request_msg };
        self.request_publisher.publish(&string_msg).await?;

        // In a real implementation, we'd wait for the response
        // For simplicity, we'll return true here
        Ok(true)
    }

    /// List all parameters
    pub async fn list_parameters(&self) -> Result<Vec<String>> {
        let request = ParameterRequest::List;
        let request_msg = serde_json::to_string(&request)?;
        let string_msg = StringMsg { data: request_msg };
        self.request_publisher.publish(&string_msg).await?;

        // In a real implementation, we'd wait for the response
        // For simplicity, we'll return empty list here
        Ok(vec![])
    }

    /// Set callback for parameter events
    pub fn on_parameter_event<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(ParameterEvent) + Send + Sync + 'static,
    {
        self.event_subscriber.on_message(move |msg: StringMsg| {
            if let Ok(event) = serde_json::from_str::<ParameterEvent>(&msg.data) {
                callback(event);
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_parameter_value_types() {
        let bool_param = ParameterValue::Bool(true);
        assert_eq!(bool_param.type_name(), "bool");

        let int_param = ParameterValue::Int(42);
        assert_eq!(int_param.type_name(), "int");

        let string_param = ParameterValue::String("test".to_string());
        assert_eq!(string_param.type_name(), "string");
    }
} 