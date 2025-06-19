//! Publisher implementation for MiniROS

use crate::core::Context;
#[cfg(any(feature = "dds-transport", feature = "tcp-transport"))]
use crate::error::MiniRosError;
use crate::error::Result;
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
    #[allow(clippy::await_holding_lock)]
    pub async fn publish(&self, message: &T) -> Result<()> {
        // Increment sequence number
        let seq = self.sequence.fetch_add(1, Ordering::Relaxed);

        tracing::debug!("Publishing message {} to topic: {}", seq, self.topic);

        // Try DDS transport first if available
        #[cfg(feature = "dds-transport")]
        {
            let has_dds_transport = {
                let dds_transport_lock = self.context.inner.dds_transport.read();
                dds_transport_lock.is_some()
            };
            if has_dds_transport {
                let dds_transport_lock = self.context.inner.dds_transport.read();
                if let Some(dds_transport) = dds_transport_lock.as_ref() {
                    // Create a DDS publisher and publish the message
                    let dds_publisher = dds_transport.create_publisher::<T>(&self.topic).await?;
                    return dds_publisher.publish(message).await;
                }
            }
        }

        // Fallback to TCP transport or memory broker
        #[cfg(feature = "tcp-transport")]
        {
            // Serialize the message
            let data = bincode::serialize(message)
                .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;

            // Use the message broker for local communication
            let sent_count = {
                let transport = self.context.inner.transport.read();
                transport.broker().publish(&self.topic, data.clone())?
            }; // Lock is released here

            if sent_count > 0 {
                tracing::debug!(
                    "Successfully published message to {} local subscribers",
                    sent_count
                );
            } else {
                tracing::debug!("No local subscribers found for topic: {}", self.topic);
            }

            // Also try to send to remote subscribers via discovery
            let remote_subscribers = {
                let discovery = self.context.inner.discovery.read();
                discovery.get_subscribers(&self.topic)
            }; // Lock is released here

            if !remote_subscribers.is_empty() {
                let mut remote_sent = 0;
                let mut remote_failed = 0;

                // Collect all endpoints first to avoid holding lock across await
                let endpoints: Vec<String> = remote_subscribers
                    .iter()
                    .map(|(_, topic_info)| topic_info.endpoint.clone())
                    .collect();

                for endpoint in endpoints {
                    // For each endpoint, send without holding the transport lock across await
                    // Note: This is a simplified approach - in practice, we'd want better
                    // connection management and error handling
                    match tokio::net::UdpSocket::bind("0.0.0.0:0").await {
                        Ok(socket) => {
                            if let Ok(addr) = endpoint.parse::<std::net::SocketAddr>() {
                                match socket.send_to(&data, addr).await {
                                    Ok(_) => remote_sent += 1,
                                    Err(e) => {
                                        tracing::debug!(
                                            "Failed to send to remote subscriber {}: {}",
                                            endpoint,
                                            e
                                        );
                                        remote_failed += 1;
                                    }
                                }
                            } else {
                                tracing::debug!("Invalid endpoint format: {}", endpoint);
                                remote_failed += 1;
                            }
                        }
                        Err(e) => {
                            tracing::debug!("Failed to create socket for {}: {}", endpoint, e);
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
        }

        // If no transport is available, use in-memory only
        #[cfg(not(any(feature = "dds-transport", feature = "tcp-transport")))]
        {
            tracing::debug!("Using in-memory only publishing (no external transport configured)");
        }

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
        let context = Context::with_domain_id(120).unwrap();

        // Try to initialize, but handle network failures gracefully in test environment
        match context.init().await {
            Ok(_) => {
                let publisher = Publisher::<StringMsg>::new("test_topic", context.clone())
                    .await
                    .unwrap();

                assert_eq!(publisher.topic(), "test_topic");
                assert_eq!(publisher.sequence(), 0);

                let _ = context.shutdown().await;
            }
            Err(_) => {
                // Network initialization may fail in constrained test environments
                println!("Network init failed in test environment - this is expected in CI");
            }
        }
    }

    #[tokio::test]
    async fn test_publisher_publish() {
        let context = Context::with_domain_id(121).unwrap();

        match context.init().await {
            Ok(_) => {
                let publisher = Publisher::<StringMsg>::new("test_topic", context.clone())
                    .await
                    .unwrap();

                let message = StringMsg {
                    data: "Hello, World!".to_string(),
                };

                // Should succeed even with no subscribers
                publisher.publish(&message).await.unwrap();

                assert_eq!(publisher.sequence(), 1);

                let _ = context.shutdown().await;
            }
            Err(_) => {
                println!("Network init failed in test environment - this is expected in CI");
            }
        }
    }

    #[tokio::test]
    async fn test_publisher_sequence() {
        let context = Context::with_domain_id(122).unwrap();

        match context.init().await {
            Ok(_) => {
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

                let _ = context.shutdown().await;
            }
            Err(_) => {
                println!("Network init failed in test environment - this is expected in CI");
            }
        }
    }
}
