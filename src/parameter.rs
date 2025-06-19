//! Parameter system for miniROS
//! 
//! Provides a simple parameter server functionality for node configuration.
//! Parameters can be set, get, and monitored for changes.

use crate::core::Context;
use crate::error::{Result, MiniRosError};
use crate::service::{Service, ServiceClient};

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

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
    /// Get the parameter type as a string
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
    pub type_name: String,
    pub description: String,
    pub read_only: bool,
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

/// Parameter server that manages configuration parameters
pub struct ParameterServer {
    context: Context,
    parameters: Arc<RwLock<HashMap<String, ParameterValue>>>,
    descriptors: Arc<RwLock<HashMap<String, ParameterDescriptor>>>,
    get_service: Service<GetParameterRequest, GetParameterResponse>,
    set_service: Service<SetParameterRequest, SetParameterResponse>,
    list_service: Service<ListParametersRequest, ListParametersResponse>,
}

impl ParameterServer {
    /// Create a new parameter server
    pub async fn new(context: Context) -> Result<Self> {
        let parameters = Arc::new(RwLock::new(HashMap::new()));
        let descriptors = Arc::new(RwLock::new(HashMap::new()));

        // Create service node
        let mut service_node = crate::node::Node::with_context("parameter_server", context.clone())?;
        service_node.init().await?;

        // Get parameter service
        let get_service = {
            let parameters_get = parameters.clone();
            service_node.create_service(
                "/parameters/get",
                move |req: GetParameterRequest| -> Result<GetParameterResponse> {
                    let parameters_ref = parameters_get.clone();
                    let response = tokio::task::block_in_place(|| {
                        tokio::runtime::Handle::current().block_on(async {
                            let params = parameters_ref.read().await;
                            if let Some(value) = params.get(&req.name) {
                                GetParameterResponse {
                                    value: Some(value.clone()),
                                    success: true,
                                    message: "Parameter found".to_string(),
                                }
                            } else {
                                GetParameterResponse {
                                    value: None,
                                    success: false,
                                    message: format!("Parameter '{}' not found", req.name),
                                }
                            }
                        })
                    });
                    Ok(response)
                }
            ).await?
        };

        // Set parameter service
        let set_service = {
            let parameters_set = parameters.clone();
            let descriptors_set = descriptors.clone();
            service_node.create_service(
                "/parameters/set",
                move |req: SetParameterRequest| -> Result<SetParameterResponse> {
                    let parameters_ref = parameters_set.clone();
                    let descriptors_ref = descriptors_set.clone();
                    let response = tokio::task::block_in_place(|| {
                        tokio::runtime::Handle::current().block_on(async {
                            // Check if parameter is read-only
                            let descriptors_guard = descriptors_ref.read().await;
                            if let Some(descriptor) = descriptors_guard.get(&req.name) {
                                if descriptor.read_only {
                                    return SetParameterResponse {
                                        success: false,
                                        message: format!("Parameter '{}' is read-only", req.name),
                                    };
                                }
                            }
                            drop(descriptors_guard);

                            // Set the parameter
                            let mut params = parameters_ref.write().await;
                            params.insert(req.name.clone(), req.value.clone());
                            
                            tracing::info!("Parameter '{}' set to {:?}", req.name, req.value);
                            
                            SetParameterResponse {
                                success: true,
                                message: format!("Parameter '{}' set successfully", req.name),
                            }
                        })
                    });
                    Ok(response)
                }
            ).await?
        };

        // List parameters service
        let list_service = {
            let parameters_list = parameters.clone();
            service_node.create_service(
                "/parameters/list",
                move |req: ListParametersRequest| -> Result<ListParametersResponse> {
                    let parameters_ref = parameters_list.clone();
                    let response = tokio::task::block_in_place(|| {
                        tokio::runtime::Handle::current().block_on(async {
                            let params = parameters_ref.read().await;
                            let names: Vec<String> = params
                                .keys()
                                .filter(|name| name.starts_with(&req.prefix))
                                .cloned()
                                .collect();
                            
                            ListParametersResponse {
                                names,
                                success: true,
                            }
                        })
                    });
                    Ok(response)
                }
            ).await?
        };

        Ok(ParameterServer {
            context,
            parameters,
            descriptors,
            get_service,
            set_service,
            list_service,
        })
    }

    /// Declare a parameter with descriptor
    pub async fn declare_parameter(
        &self,
        name: &str,
        default_value: ParameterValue,
        description: &str,
        read_only: bool,
    ) -> Result<()> {
        let descriptor = ParameterDescriptor {
            name: name.to_string(),
            type_name: default_value.type_name().to_string(),
            description: description.to_string(),
            read_only,
        };

        // Set default value if not already set
        {
            let mut params = self.parameters.write().await;
            params.entry(name.to_string()).or_insert(default_value);
        }

        // Store descriptor
        {
            let mut descriptors = self.descriptors.write().await;
            descriptors.insert(name.to_string(), descriptor);
        }

        tracing::info!("Declared parameter '{}': {}", name, description);
        Ok(())
    }

    /// Get a parameter value directly (server-side)
    pub async fn get_parameter(&self, name: &str) -> Option<ParameterValue> {
        let params = self.parameters.read().await;
        params.get(name).cloned()
    }

    /// Set a parameter value directly (server-side)
    pub async fn set_parameter(&self, name: &str, value: ParameterValue) -> Result<()> {
        // Check if read-only
        {
            let descriptors = self.descriptors.read().await;
            if let Some(descriptor) = descriptors.get(name) {
                if descriptor.read_only {
                    return Err(MiniRosError::ConfigError(
                        format!("Parameter '{}' is read-only", name)
                    ));
                }
            }
        }

        // Set parameter
        {
            let mut params = self.parameters.write().await;
            params.insert(name.to_string(), value);
        }

        Ok(())
    }

    /// Get all parameter names
    pub async fn list_parameters(&self, prefix: &str) -> Vec<String> {
        let params = self.parameters.read().await;
        params
            .keys()
            .filter(|name| name.starts_with(prefix))
            .cloned()
            .collect()
    }
}

/// Parameter client for accessing parameters from other nodes
pub struct ParameterClient {
    context: Context,
    get_client: ServiceClient<GetParameterRequest, GetParameterResponse>,
    set_client: ServiceClient<SetParameterRequest, SetParameterResponse>,
    list_client: ServiceClient<ListParametersRequest, ListParametersResponse>,
}

impl ParameterClient {
    /// Create a new parameter client
    pub async fn new(context: Context) -> Result<Self> {
        let mut client_node = crate::node::Node::with_context("parameter_client", context.clone())?;
        client_node.init().await?;

        let get_client = client_node.create_service_client::<GetParameterRequest, GetParameterResponse>("/parameters/get").await?;
        let set_client = client_node.create_service_client::<SetParameterRequest, SetParameterResponse>("/parameters/set").await?;
        let list_client = client_node.create_service_client::<ListParametersRequest, ListParametersResponse>("/parameters/list").await?;

        Ok(ParameterClient {
            context,
            get_client,
            set_client,
            list_client,
        })
    }

    /// Get a parameter value
    pub async fn get_parameter(&self, name: &str) -> Result<Option<ParameterValue>> {
        let request = GetParameterRequest {
            name: name.to_string(),
        };

        let response = self.get_client.call(request).await?;
        
        if response.success {
            Ok(response.value)
        } else {
            Err(MiniRosError::ConfigError(response.message))
        }
    }

    /// Set a parameter value
    pub async fn set_parameter(&self, name: &str, value: ParameterValue) -> Result<()> {
        let request = SetParameterRequest {
            name: name.to_string(),
            value,
        };

        let response = self.set_client.call(request).await?;
        
        if response.success {
            Ok(())
        } else {
            Err(MiniRosError::ConfigError(response.message))
        }
    }

    /// List parameters with optional prefix filter
    pub async fn list_parameters(&self, prefix: &str) -> Result<Vec<String>> {
        let request = ListParametersRequest {
            prefix: prefix.to_string(),
        };

        let response = self.list_client.call(request).await?;
        
        if response.success {
            Ok(response.names)
        } else {
            Err(MiniRosError::ConfigError("Failed to list parameters".to_string()))
        }
    }

    /// Convenience method to get a bool parameter
    pub async fn get_bool(&self, name: &str) -> Result<Option<bool>> {
        if let Some(value) = self.get_parameter(name).await? {
            Ok(value.as_bool())
        } else {
            Ok(None)
        }
    }

    /// Convenience method to get an int parameter
    pub async fn get_int(&self, name: &str) -> Result<Option<i64>> {
        if let Some(value) = self.get_parameter(name).await? {
            Ok(value.as_int())
        } else {
            Ok(None)
        }
    }

    /// Convenience method to get a float parameter
    pub async fn get_float(&self, name: &str) -> Result<Option<f64>> {
        if let Some(value) = self.get_parameter(name).await? {
            Ok(value.as_float())
        } else {
            Ok(None)
        }
    }

    /// Convenience method to get a string parameter
    pub async fn get_string(&self, name: &str) -> Result<Option<String>> {
        if let Some(value) = self.get_parameter(name).await? {
            Ok(value.as_string().map(|s| s.to_string()))
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_parameter_server() {
        let context = crate::core::Context::with_domain_id(60).unwrap();
        context.init().await.unwrap();

        let server = ParameterServer::new(context.clone()).await.unwrap();
        
        // Declare a parameter
        server.declare_parameter(
            "test_param",
            ParameterValue::String("default".to_string()),
            "Test parameter",
            false,
        ).await.unwrap();

        // Get parameter
        let value = server.get_parameter("test_param").await;
        assert!(value.is_some());
        assert_eq!(value.unwrap().as_string(), Some("default"));

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_parameter_client() {
        let context = crate::core::Context::with_domain_id(61).unwrap();
        context.init().await.unwrap();

        let _server = ParameterServer::new(context.clone()).await.unwrap();
        let client = ParameterClient::new(context.clone()).await.unwrap();

        // This would normally require the server to be running
        // In a real scenario, we'd test with a running parameter server

        context.shutdown().await.unwrap();
    }
} 