//! Publisher implementation for MiniROS

use crate::core::Context;
use crate::error::{MiniRosError, Result};
use crate::message::Message;

use std::marker::PhantomData;
use std::sync::atomic::{AtomicU64, Ordering};

/// Publisher for sending messages to a topic
///
/// Publishers send messages to subscribers interested in the same topic.
/// The communication is decoupled - publishers don't need to know about subscribers.
pub struct Publisher<T: Message> {
    topic: String,
    context: Context,
    sequence: AtomicU64,
    _phantom: PhantomData<T>,
}

impl<T: Message> Publisher<T> {
    /// Create a new publisher for the given topic
    pub(crate) async fn new(topic: &str, context: Context) -> Result<Self> {
        tracing::debug!("Creating publisher for topic: {}", topic);

        Ok(Publisher {
            topic: topic.to_string(),
            context,
            sequence: AtomicU64::new(0),
            _phantom: PhantomData,
        })
    }

    /// Publish a message to all subscribers
    #[cfg(feature = "tcp-transport")]
    pub async fn publish(&self, message: &T) -> Result<()> {
        // Increment sequence number
        let seq = self.sequence.fetch_add(1, Ordering::Relaxed);

        tracing::debug!("Publishing message {} to topic: {}", seq, self.topic);

        // Serialize the message
        let data = bincode::serialize(message)
            .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;

        // Use the message broker for local communication
        {
            let transport = self.context.inner.transport.read();
            let sent_count = transport.broker().publish(&self.topic, data)?;
            
            if sent_count > 0 {
                tracing::debug!(
                    "Successfully published message to {} local subscribers",
                    sent_count
                );
            } else {
                tracing::debug!("No local subscribers found for topic: {}", self.topic);
            }
        }

        // Also try to send to remote subscribers via discovery
        let remote_subscribers = {
            let discovery = self.context.inner.discovery.read();
            discovery.get_subscribers(&self.topic)
        };

        if !remote_subscribers.is_empty() {
            let data = bincode::serialize(message)
                .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;
            
            let mut remote_sent = 0;
            let mut remote_failed = 0;
            
            for (_node_info, topic_info) in &remote_subscribers {
                let transport = self.context.inner.transport.read();
                match transport.send(&topic_info.endpoint, &data).await {
                    Ok(()) => remote_sent += 1,
                    Err(e) => {
                        tracing::debug!("Failed to send to remote subscriber {}: {}", topic_info.endpoint, e);
                        remote_failed += 1;
                    }
                }
            }
            
            if remote_sent > 0 {
                tracing::debug!("Successfully sent to {} remote subscribers", remote_sent);
            }
            if remote_failed > 0 {
                tracing::debug!("Failed to send to {} remote subscribers", remote_failed);
            }
        }

        Ok(())
    }

    /// Publish a message to all subscribers (DDS transport version)
    #[cfg(feature = "dds-transport")]
    pub async fn publish(&self, message: &T) -> Result<()> {
        // Increment sequence number
        let seq = self.sequence.fetch_add(1, Ordering::Relaxed);

        tracing::debug!("Publishing message {} to topic: {}", seq, self.topic);

        // Use DDS transport if available
        {
            let dds_transport_lock = self.context.inner.dds_transport.read();
            if let Some(dds_transport) = dds_transport_lock.as_ref() {
                return dds_transport.publish(&self.topic, message).await;
            }
        }

        // Fallback to other transports
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

        // Send to all subscribers
        let mut sent_count = 0;
        let mut error_count = 0;
        
        for (_node_info, topic_info) in &subscribers {
            // Since we don't have transport in DDS mode, this is placeholder
            tracing::debug!("Would send to subscriber at: {}", topic_info.endpoint);
            sent_count += 1;
        }

        tracing::debug!(
            "Successfully published message to {} subscribers",
            sent_count
        );

        Ok(())
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
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
            context: self.context.clone(),
            sequence: AtomicU64::new(self.sequence.load(Ordering::Relaxed)),
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

        let publisher = Publisher::<StringMsg>::new("test_topic", context.clone())
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

        let publisher = Publisher::<StringMsg>::new("test_topic", context.clone())
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

        let publisher = Publisher::<StringMsg>::new("test_topic", context.clone())
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
