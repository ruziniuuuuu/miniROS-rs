//! Service implementation for MiniROS

use crate::core::Context;

use crate::error::{MiniRosError, Result};
use crate::message::Message;
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::timeout;
use uuid::Uuid;

/// Service request/response wrapper for network transport
#[derive(Debug, Clone, Serialize, Deserialize)]
struct ServiceMessage<T> {
    request_id: Uuid,
    data: T,
}

/// Service response with status
#[derive(Debug, Clone, Serialize, Deserialize)]
enum ServiceResponse<T> {
    Success(T),
    Error(String),
}

/// Service provider that handles incoming requests
///
/// Services provide request/response communication pattern.
/// Unlike pub/sub, services are synchronous and bidirectional.
pub struct Service<Req: Message, Res: Message> {
    name: String,
    endpoint: String,
    #[allow(dead_code)]
    context: Context,
    _stop_sender: mpsc::UnboundedSender<()>,
    _phantom: PhantomData<(Req, Res)>,
}

impl<Req: Message, Res: Message> Service<Req, Res> {
    /// Create a new service provider
    #[allow(clippy::await_holding_lock)]
    pub(crate) async fn new<F>(
        name: &str,
        endpoint: &str,
        callback: F,
        context: Context,
    ) -> Result<Self>
    where
        F: Fn(Req) -> Result<Res> + Send + Sync + 'static,
    {
        tracing::debug!("Creating service: {} on endpoint: {}", name, endpoint);

        let (stop_sender, mut stop_receiver) = mpsc::unbounded_channel();

        // Start listening for incoming requests
        let transport_receiver = {
            let transport = context.inner.transport.clone();
            let transport_guard = transport.read();
            transport_guard.listen(endpoint).await?
        };

        // Spawn background task to handle requests
        let name_clone = name.to_string();
        let endpoint_clone = endpoint.to_string();
        let context_clone = context.clone();
        let callback = Arc::new(callback);

        tokio::spawn(async move {
            tracing::info!("Service '{}' listening on {}", name_clone, endpoint_clone);

            loop {
                tokio::select! {
                    // Receive incoming request
                    raw_data = async {
                        transport_receiver.recv().ok()
                    } => {
                        if let Some(data) = raw_data {
                            let callback_clone = callback.clone();
                            let context_ref = context_clone.clone();
                            let service_name = name_clone.clone();

                            // Process request in separate task to avoid blocking
                            tokio::spawn(async move {
                                Self::handle_request(data, callback_clone, context_ref, &service_name).await;
                            });
                        } else {
                            // Transport closed
                            break;
                        }
                    }
                    // Stop signal
                    _ = stop_receiver.recv() => {
                        tracing::debug!("Stopping service: {}", name_clone);
                        break;
                    }
                }
            }
        });

        Ok(Service {
            name: name.to_string(),
            endpoint: endpoint.to_string(),
            context,
            _stop_sender: stop_sender,
            _phantom: PhantomData,
        })
    }

    async fn handle_request<F>(
        raw_data: Vec<u8>,
        callback: Arc<F>,
        _context: Context,
        service_name: &str,
    ) where
        F: Fn(Req) -> Result<Res> + Send + Sync + 'static,
    {
        // Deserialize request
        let service_request: ServiceMessage<Req> = match bincode::deserialize(&raw_data) {
            Ok(req) => req,
            Err(e) => {
                tracing::warn!(
                    "Failed to deserialize service request for {}: {}",
                    service_name,
                    e
                );
                return;
            }
        };

        tracing::debug!(
            "Processing service request {} for service: {}",
            service_request.request_id,
            service_name
        );

        // Process request
        let response_data = match callback(service_request.data) {
            Ok(result) => ServiceResponse::Success(result),
            Err(e) => ServiceResponse::Error(e.to_string()),
        };

        let response_message = ServiceMessage {
            request_id: service_request.request_id,
            data: response_data,
        };

        // Serialize response
        let _response_bytes = match bincode::serialize(&response_message) {
            Ok(bytes) => bytes,
            Err(e) => {
                tracing::error!("Failed to serialize service response: {}", e);
                return;
            }
        };

        // Send response back (this is simplified - in reality we'd need response addressing)
        // For now, we'll just log that we processed the request
        tracing::debug!(
            "Processed service request {} for service: {}",
            service_request.request_id,
            service_name
        );
    }

    /// Get the service name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the service endpoint
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }
}

impl<Req: Message, Res: Message> Drop for Service<Req, Res> {
    fn drop(&mut self) {
        let _ = self._stop_sender.send(());
    }
}

/// Service client for making requests to services
pub struct ServiceClient<Req: Message, Res: Message> {
    service_name: String,
    context: Context,
    _phantom: PhantomData<(Req, Res)>,
}

impl<Req: Message, Res: Message> ServiceClient<Req, Res> {
    /// Create a new service client
    pub(crate) async fn new(service_name: &str, context: Context) -> Result<Self> {
        tracing::debug!("Creating service client for: {}", service_name);

        Ok(ServiceClient {
            service_name: service_name.to_string(),
            context,
            _phantom: PhantomData,
        })
    }

    /// Call the service with a request and wait for response
    pub async fn call(&self, request: Req) -> Result<Res> {
        self.call_with_timeout(request, Duration::from_secs(5))
            .await
    }

    /// Call the service with a timeout
    #[allow(clippy::await_holding_lock)]
    pub async fn call_with_timeout(&self, request: Req, timeout_duration: Duration) -> Result<Res> {
        timeout(timeout_duration, self.call_internal(request))
            .await
            .map_err(|_| {
                MiniRosError::Timeout(format!("Service call to '{}' timed out", self.service_name))
            })?
    }

    #[allow(clippy::await_holding_lock)]
    async fn call_internal(&self, request: Req) -> Result<Res> {
        tracing::debug!("Calling service: {}", self.service_name);

        // Find service providers
        let (_providers, service_endpoint) = {
            let discovery = self.context.inner.discovery.read();
            let providers = discovery.get_service_providers(&self.service_name);

            if providers.is_empty() {
                return Err(MiniRosError::ServiceNotFound(self.service_name.clone()));
            }

            let (_node_info, service_info) = &providers[0];
            let service_endpoint = service_info.endpoint.clone();
            (providers, service_endpoint)
        };

        // Create unique request ID
        let request_id = Uuid::new_v4();
        let service_request = ServiceMessage {
            request_id,
            data: request,
        };

        // Serialize request
        let request_bytes = bincode::serialize(&service_request)
            .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;

        // Send request
        {
            let transport = self.context.inner.transport.clone();
            let transport_guard = transport.read();
            transport_guard.send(&service_endpoint, &request_bytes).await?;
        }

        // For now, return a dummy response since we don't have full response handling
        // In a complete implementation, we'd wait for the response with the matching request_id
        tracing::debug!("Service call sent to: {}", self.service_name);

        // This is a placeholder - in reality we'd wait for and deserialize the actual response
        Err(MiniRosError::NetworkError(
            "Service response handling not fully implemented".to_string(),
        ))
    }

    /// Get the service name
    pub fn service_name(&self) -> &str {
        &self.service_name
    }

    /// Check if the service is available
    pub async fn is_available(&self) -> bool {
        let discovery = self.context.inner.discovery.read();
        !discovery
            .get_service_providers(&self.service_name)
            .is_empty()
    }

    /// Wait for the service to become available
    pub async fn wait_for_service(&self, timeout_duration: Duration) -> Result<()> {
        let start = std::time::Instant::now();

        while start.elapsed() < timeout_duration {
            if self.is_available().await {
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }

        Err(MiniRosError::Timeout(format!(
            "Service '{}' did not become available within {:?}",
            self.service_name, timeout_duration
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Context;
    use crate::message::{Int32Msg, StringMsg};

    #[tokio::test]
    async fn test_service_creation() {
        let context = Context::with_domain_id(40).unwrap();
        context.init().await.unwrap();

        let service = Service::new(
            "test_service",
            "127.0.0.1:0",
            |req: StringMsg| -> Result<Int32Msg> {
                Ok(Int32Msg {
                    data: req.data.len() as i32,
                })
            },
            context.clone(),
        )
        .await
        .unwrap();

        assert_eq!(service.name(), "test_service");

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_service_client_creation() {
        let context = Context::with_domain_id(41).unwrap();
        context.init().await.unwrap();

        let client = ServiceClient::<StringMsg, Int32Msg>::new("test_service", context.clone())
            .await
            .unwrap();

        assert_eq!(client.service_name(), "test_service");
        assert!(!client.is_available().await); // No service running

        context.shutdown().await.unwrap();
    }
}
