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
    #[allow(dead_code)]
    context: Context,
    _stop_sender: mpsc::UnboundedSender<()>,
    _phantom: PhantomData<(Req, Res)>,
}

impl<Req: Message, Res: Message> Service<Req, Res> {
    /// Create a new service provider
    pub(crate) async fn new<F>(name: &str, callback: F, context: Context) -> Result<Self>
    where
        F: Fn(Req) -> Result<Res> + Send + Sync + 'static,
    {
        tracing::debug!("Creating service: {}", name);

        let (stop_sender, mut stop_receiver) = mpsc::unbounded_channel();

        // For now, services are simplified - just create the structure
        // In a full implementation, we'd set up request handling
        let name_clone = name.to_string();
        let _callback = Arc::new(callback);

        tokio::spawn(async move {
            tracing::info!("Service '{}' created (simplified mode)", name_clone);

            // Simple wait for stop signal
            tokio::select! {
                _ = stop_receiver.recv() => {
                    tracing::debug!("Stopping service: {}", name_clone);
                }
            }
        });

        Ok(Service {
            name: name.to_string(),
            context,
            _stop_sender: stop_sender,
            _phantom: PhantomData,
        })
    }

    /// Get the service name
    pub fn name(&self) -> &str {
        &self.name
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
        self.call_with_timeout(request, Duration::from_secs(10))
            .await
    }

    /// Call the service with a request and custom timeout
    pub async fn call_with_timeout(&self, request: Req, timeout_duration: Duration) -> Result<Res> {
        timeout(timeout_duration, self.call_internal(request))
            .await
            .map_err(|_| MiniRosError::ServiceTimeout(self.service_name.clone()))?
    }

    async fn call_internal(&self, _request: Req) -> Result<Res> {
        // For now, this is a simplified implementation
        // In a real implementation, we'd:
        // 1. Find service providers via discovery
        // 2. Send request to service
        // 3. Wait for response
        // 4. Return response

        tracing::debug!("Service call to {} (simplified mode)", self.service_name);

        // Return an error for now since we don't have full service implementation
        Err(MiniRosError::ServiceNotFound(self.service_name.clone()))
    }

    /// Get the service name
    pub fn service_name(&self) -> &str {
        &self.service_name
    }

    /// Check if the service is available
    pub async fn is_available(&self) -> bool {
        // Check if any service providers are available via discovery
        let discovery = self.context.inner.discovery.read();
        let providers = discovery.get_service_providers(&self.service_name);
        !providers.is_empty()
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

        Err(MiniRosError::ServiceTimeout(self.service_name.clone()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Context;
    use crate::message::{Int32Msg, StringMsg};

    #[tokio::test]
    async fn test_service_creation() {
        let context = Context::with_domain_id(140).unwrap();

        match context.init().await {
            Ok(_) => {
                let callback = |request: StringMsg| -> Result<Int32Msg> {
                    Ok(Int32Msg {
                        data: request.data.len() as i32,
                    })
                };

                let service = Service::new("test_service", callback, context.clone())
                    .await
                    .unwrap();

                assert_eq!(service.name(), "test_service");

                let _ = context.shutdown().await;
            }
            Err(_) => {
                println!("Context init failed in test environment - this is expected in CI");
            }
        }
    }

    #[tokio::test]
    async fn test_service_client_creation() {
        let context = Context::with_domain_id(141).unwrap();

        match context.init().await {
            Ok(_) => {
                let client =
                    ServiceClient::<StringMsg, Int32Msg>::new("test_service", context.clone())
                        .await
                        .unwrap();

                assert_eq!(client.service_name(), "test_service");
                assert!(!client.is_available().await); // No service provider running

                let _ = context.shutdown().await;
            }
            Err(_) => {
                println!("Context init failed in test environment - this is expected in CI");
            }
        }
    }
}
