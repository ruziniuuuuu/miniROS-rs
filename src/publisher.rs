//! Publisher implementation for MiniROS

use crate::core::Context;
use crate::error::{MiniRosError, Result};
use crate::message::Message;

use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

/// Publisher for sending messages to a topic
///
/// Publishers send messages to subscribers interested in the same topic.
/// The communication is decoupled - publishers don't need to know about subscribers.
pub struct Publisher<T: Message> {
    topic: String,
    endpoint: String,
    context: Context,
    sequence: Arc<AtomicU64>,
    _phantom: PhantomData<T>,
}

impl<T: Message> Publisher<T> {
    /// Create a new publisher for the given topic
    #[allow(clippy::await_holding_lock)]
    pub(crate) async fn new(topic: &str, endpoint: &str, context: Context) -> Result<Self> {
        tracing::debug!(
            "Creating publisher for topic: {} on endpoint: {}",
            topic,
            endpoint
        );

        // Start listening on the endpoint for subscriber connections
        {
            let transport = context.inner.transport.clone();
            let transport_guard = transport.read();
            let _receiver = transport_guard.listen(endpoint).await?;
        }

        Ok(Publisher {
            topic: topic.to_string(),
            endpoint: endpoint.to_string(),
            context,
            sequence: Arc::new(AtomicU64::new(0)),
            _phantom: PhantomData,
        })
    }

    /// Publish a message to all subscribers
    #[allow(clippy::await_holding_lock)]
    pub async fn publish(&self, message: &T) -> Result<()> {
        // Increment sequence number
        let seq = self.sequence.fetch_add(1, Ordering::Relaxed);

        tracing::debug!("Publishing message {} to topic: {}", seq, self.topic);

        // Serialize the message
        let data = bincode::serialize(message)
            .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;

        // Find all subscribers for this topic
        let subscribers = {
            let discovery = self.context.inner.discovery.read();
            discovery.get_subscribers(&self.topic)
        };

        if subscribers.is_empty() {
            tracing::debug!("No subscribers found for topic: {}", self.topic);
            return Ok(());
        }

        // Send to all subscribers concurrently
        let mut results = Vec::new();
        for (_node_info, topic_info) in &subscribers {
            let endpoint = topic_info.endpoint.clone();
            let data_clone = data.clone();

            // Send to each subscriber
            let transport = self.context.inner.transport.clone();
            let transport_guard = transport.read();
            let result = transport_guard.send(&endpoint, &data_clone).await;
            results.push(result);
        }

        // Check for any errors
        let mut error_count = 0;
        for result in results {
            if let Err(e) = result {
                tracing::warn!("Failed to send message to subscriber: {}", e);
                error_count += 1;
            }
        }

        if error_count > 0 {
            tracing::warn!("Failed to send to {} subscribers", error_count);
        } else {
            let subscriber_count = {
                let discovery = self.context.inner.discovery.read();
                discovery.get_subscribers(&self.topic).len()
            };
            tracing::debug!(
                "Successfully published message to {} subscribers",
                subscriber_count
            );
        }

        Ok(())
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get the publisher endpoint
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Get the current sequence number
    pub fn sequence(&self) -> u64 {
        self.sequence.load(Ordering::Relaxed)
    }
}

impl<T: Message> Clone for Publisher<T> {
    fn clone(&self) -> Self {
        Publisher {
            topic: self.topic.clone(),
            endpoint: self.endpoint.clone(),
            context: self.context.clone(),
            sequence: self.sequence.clone(),
            _phantom: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Context;
    use crate::message::StringMsg;

    #[tokio::test]
    async fn test_publisher_creation() {
        let context = Context::with_domain_id(20).unwrap();
        context.init().await.unwrap();

        let publisher = Publisher::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone())
            .await
            .unwrap();

        assert_eq!(publisher.topic(), "test_topic");
        assert_eq!(publisher.sequence(), 0);

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_publisher_publish() {
        let context = Context::with_domain_id(21).unwrap();
        context.init().await.unwrap();

        let publisher = Publisher::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone())
            .await
            .unwrap();

        let message = StringMsg {
            data: "Hello, World!".to_string(),
        };

        // Should succeed even with no subscribers
        publisher.publish(&message).await.unwrap();

        assert_eq!(publisher.sequence(), 1);

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_publisher_sequence() {
        let context = Context::with_domain_id(22).unwrap();
        context.init().await.unwrap();

        let publisher = Publisher::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone())
            .await
            .unwrap();

        let message = StringMsg {
            data: "test".to_string(),
        };

        for i in 1..=5 {
            publisher.publish(&message).await.unwrap();
            assert_eq!(publisher.sequence(), i);
        }

        context.shutdown().await.unwrap();
    }
}
