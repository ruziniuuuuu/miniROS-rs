//! Node implementation for MiniROS

use crate::core::Context;
use crate::discovery::{NodeInfo, ServiceInfo, TopicInfo};
use crate::error::{MiniRosError, Result};
use crate::message::Message;
use crate::publisher::Publisher;
use crate::service::{Service, ServiceClient};
use crate::subscriber::Subscriber;

use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;

/// MiniROS Node - the basic computational unit
///
/// A node encapsulates functionality and can publish, subscribe to topics,
/// provide services, and communicate with other nodes.
pub struct Node {
    pub name: String,
    pub id: Uuid,
    context: Context,
    publishers: Vec<TopicInfo>,
    subscribers: Vec<TopicInfo>,
    services: Vec<ServiceInfo>,
}

impl Node {
    /// Create a new node with the given name and default context
    pub fn new(name: &str) -> Result<Self> {
        let context = Context::new()?;
        Self::with_context(name, context)
    }

    /// Create a new node with the given name and context
    pub fn with_context(name: &str, context: Context) -> Result<Self> {
        let id = Uuid::new_v4();

        Ok(Node {
            name: name.to_string(),
            id,
            context,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            services: Vec::new(),
        })
    }

    /// Initialize the node and register with discovery service
    pub async fn init(&mut self) -> Result<()> {
        tracing::info!("Initializing node: {}", self.name);

        // Initialize context if not already done
        self.context.init().await?;

        // Register with discovery service
        self.announce().await?;

        tracing::info!("Node '{}' initialized with ID: {}", self.name, self.id);
        Ok(())
    }

    /// Shutdown the node gracefully
    pub async fn shutdown(&self) -> Result<()> {
        tracing::info!("Shutting down node: {}", self.name);

        // Context shutdown will handle cleanup
        self.context.shutdown().await?;

        tracing::info!("Node '{}' shutdown complete", self.name);
        Ok(())
    }

    /// Create a publisher for the given topic
    pub async fn create_publisher<T: Message>(&mut self, topic: &str) -> Result<Publisher<T>> {
        let type_name = std::any::type_name::<T>().to_string();

        let topic_info = TopicInfo {
            name: topic.to_string(),
            type_name,
            endpoint: "local".to_string(), // Use "local" to indicate in-memory communication
        };

        self.publishers.push(topic_info);

        // Re-announce with updated publisher info
        self.announce().await?;

        Publisher::new(topic, self.context.clone()).await
    }

    /// Create a subscriber for the given topic
    pub async fn create_subscriber<T: Message>(&mut self, topic: &str) -> Result<Subscriber<T>> {
        let type_name = std::any::type_name::<T>().to_string();

        let topic_info = TopicInfo {
            name: topic.to_string(),
            type_name,
            endpoint: "local".to_string(), // Use "local" to indicate in-memory communication
        };

        self.subscribers.push(topic_info);

        // Re-announce with updated subscriber info
        self.announce().await?;

        Subscriber::new(topic, self.context.clone()).await
    }

    /// Create a service provider for the given service
    pub async fn create_service<Req: Message, Res: Message>(
        &mut self,
        service_name: &str,
        callback: impl Fn(Req) -> Result<Res> + Send + Sync + 'static,
    ) -> Result<Service<Req, Res>> {
        let type_name = format!(
            "{}/{}",
            std::any::type_name::<Req>(),
            std::any::type_name::<Res>()
        );

        let service_info = ServiceInfo {
            name: service_name.to_string(),
            type_name,
            endpoint: "local".to_string(), // Use "local" for simplicity
        };

        self.services.push(service_info);

        // Re-announce with updated service info
        self.announce().await?;

        Service::new(service_name, callback, self.context.clone()).await
    }

    /// Create a service client for the given service
    pub async fn create_service_client<Req: Message, Res: Message>(
        &self,
        service_name: &str,
    ) -> Result<ServiceClient<Req, Res>> {
        ServiceClient::new(service_name, self.context.clone()).await
    }

    /// Get the node's unique ID
    pub fn id(&self) -> Uuid {
        self.id
    }

    /// Get the node's name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the context
    pub fn context(&self) -> &Context {
        &self.context
    }

    /// Spin the node - keep it running and processing messages
    /// This is a convenience method for simple nodes
    pub async fn spin(&self) -> Result<()> {
        tracing::info!("Node '{}' spinning...", self.name);

        // Simple spin - just wait for shutdown signal
        // In a real implementation, this might process callbacks, etc.
        tokio::signal::ctrl_c()
            .await
            .map_err(MiniRosError::IoError)?;

        tracing::info!("Received shutdown signal");
        Ok(())
    }

    /// Announce node presence to discovery service
    #[allow(clippy::await_holding_lock)]
    async fn announce(&self) -> Result<()> {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        let node_info = NodeInfo {
            id: self.id,
            name: self.name.clone(),
            domain_id: self.context.domain_id(),
            endpoint: "local".to_string(), // Simplified endpoint
            last_seen: now,
            publishers: self.publishers.clone(),
            subscribers: self.subscribers.clone(),
            services: self.services.clone(),
        };

        let discovery = self.context.inner.discovery.clone();
        let discovery_guard = discovery.read();
        discovery_guard.announce_node(node_info).await?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Context;
    use crate::message::StringMsg;

    #[tokio::test]
    async fn test_node_creation() {
        let node = Node::new("test_node").unwrap();
        assert_eq!(node.name(), "test_node");
    }

    #[tokio::test]
    async fn test_node_with_context() {
        let context = Context::with_domain_id(10).unwrap();
        let node = Node::with_context("test_node", context).unwrap();
        assert_eq!(node.name(), "test_node");
    }

    #[tokio::test]
    async fn test_publisher_creation() {
        let context = Context::with_domain_id(11).unwrap();
        context.init().await.unwrap();

        let mut node = Node::with_context("test_node", context.clone()).unwrap();
        node.init().await.unwrap();

        let publisher = node
            .create_publisher::<StringMsg>("/test_topic")
            .await
            .unwrap();

        assert_eq!(publisher.topic(), "/test_topic");

        // Test that the publisher info was added to the node
        assert_eq!(node.publishers.len(), 1);
        assert_eq!(node.publishers[0].name, "/test_topic");

        node.shutdown().await.unwrap();
        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_subscriber_creation() {
        let context = Context::with_domain_id(12).unwrap();
        context.init().await.unwrap();

        let mut node = Node::with_context("test_node", context.clone()).unwrap();
        node.init().await.unwrap();

        let subscriber = node
            .create_subscriber::<StringMsg>("/test_topic")
            .await
            .unwrap();

        assert_eq!(subscriber.topic(), "/test_topic");

        // Test that the subscriber info was added to the node
        assert_eq!(node.subscribers.len(), 1);
        assert_eq!(node.subscribers[0].name, "/test_topic");

        node.shutdown().await.unwrap();
        context.shutdown().await.unwrap();
    }
}
