//! Subscriber implementation for MiniROS

use crate::core::Context;
use crate::error::{MiniRosError, Result};
use crate::message::Message;

use crossbeam_channel::Receiver;
use std::marker::PhantomData;
use std::sync::Arc;

/// Type alias for message callback to reduce complexity
type MessageCallback<T> = Arc<parking_lot::Mutex<Option<Box<dyn Fn(T) + Send + Sync + 'static>>>>;
use parking_lot::Mutex;

/// Subscriber for receiving messages from a topic
///
/// Subscribers receive messages published to their topic of interest.
/// They can register callbacks to handle incoming messages.
pub struct Subscriber<T: Message> {
    topic: String,
    context: Context,
    message_receiver: Option<Receiver<Vec<u8>>>,
    callback: MessageCallback<T>,
    _phantom: PhantomData<T>,
}

impl<T: Message> Subscriber<T> {
    /// Create a new subscriber for the given topic
    pub(crate) async fn new(topic: &str, context: Context) -> Result<Self> {
        tracing::debug!("Creating subscriber for topic: {}", topic);

        // Subscribe to the message broker for local communication
        let message_receiver = {
            let transport = context.inner.transport.read();
            Some(transport.broker().subscribe(topic))
        };

        let subscriber = Subscriber {
            topic: topic.to_string(),
            context,
            message_receiver,
            callback: Arc::new(Mutex::new(None)),
            _phantom: PhantomData,
        };

        // Start message processing task
        subscriber.start_message_processing();

        Ok(subscriber)
    }

    /// Set a callback function to handle incoming messages
    pub fn on_message<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(T) + Send + Sync + 'static,
    {
        tracing::debug!("Setting message callback for topic: {}", self.topic);
        *self.callback.lock() = Some(Box::new(callback));
        Ok(())
    }

    /// Try to receive a message without blocking
    pub fn try_recv(&self) -> Result<Option<T>> {
        if let Some(ref receiver) = self.message_receiver {
            match receiver.try_recv() {
                Ok(data) => {
                    let message: T = bincode::deserialize(&data)
                        .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;
                    Ok(Some(message))
                }
                Err(crossbeam_channel::TryRecvError::Empty) => Ok(None),
                Err(crossbeam_channel::TryRecvError::Disconnected) => Err(
                    MiniRosError::NetworkError("Receiver disconnected".to_string()),
                ),
            }
        } else {
            Ok(None)
        }
    }

    /// Start the background message processing task
    fn start_message_processing(&self) {
        if let Some(ref receiver) = self.message_receiver {
            let receiver = receiver.clone();
            let callback = self.callback.clone();
            let topic = self.topic.clone();

            tokio::spawn(async move {
                while let Ok(data) = receiver.recv() {
                    // Deserialize the message
                    match bincode::deserialize::<T>(&data) {
                        Ok(message) => {
                            // Call the callback if set
                            if let Some(ref callback_fn) = *callback.lock() {
                                callback_fn(message);
                            }
                        }
                        Err(e) => {
                            tracing::warn!(
                                "Failed to deserialize message for topic {}: {}",
                                topic,
                                e
                            );
                        }
                    }
                }
            });
        }
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

impl<T: Message> Clone for Subscriber<T> {
    fn clone(&self) -> Self {
        // Create a new subscriber with the same configuration
        let new_receiver = {
            let transport = self.context.inner.transport.read();
            Some(transport.broker().subscribe(&self.topic))
        };

        let subscriber = Subscriber {
            topic: self.topic.clone(),
            context: self.context.clone(),
            message_receiver: new_receiver,
            callback: Arc::new(Mutex::new(None)),
            _phantom: PhantomData,
        };

        // Start message processing for the cloned subscriber
        subscriber.start_message_processing();

        subscriber
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Context;
    use crate::message::StringMsg;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;
    use tokio::time::sleep;

    #[tokio::test]
    async fn test_subscriber_creation() {
        let context = Context::with_domain_id(30).unwrap();
        context.init().await.unwrap();

        let subscriber = Subscriber::<StringMsg>::new("test_topic", context.clone())
            .await
            .unwrap();

        assert_eq!(subscriber.topic(), "test_topic");

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_subscriber_callback() {
        let context = Context::with_domain_id(31).unwrap();
        context.init().await.unwrap();

        let subscriber = Subscriber::<StringMsg>::new("test_topic", context.clone())
            .await
            .unwrap();

        let received = Arc::new(AtomicBool::new(false));
        let received_clone = received.clone();

        subscriber
            .on_message(move |_msg: StringMsg| {
                received_clone.store(true, Ordering::Relaxed);
            })
            .unwrap();

        // Simulate receiving a message by publishing to the broker
        {
            let transport = context.inner.transport.read();
            let message = StringMsg {
                data: "test".to_string(),
            };
            let data = bincode::serialize(&message).unwrap();
            transport.broker().publish("test_topic", data).unwrap();
        }

        // Wait a bit for the message to be processed
        sleep(Duration::from_millis(10)).await;

        assert!(received.load(Ordering::Relaxed));

        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_subscriber_try_recv() {
        let context = Context::with_domain_id(32).unwrap();
        context.init().await.unwrap();

        let subscriber = Subscriber::<StringMsg>::new("test_topic", context.clone())
            .await
            .unwrap();

        // Should return None when no messages are available
        assert!(subscriber.try_recv().unwrap().is_none());

        // Publish a message to the broker
        {
            let transport = context.inner.transport.read();
            let message = StringMsg {
                data: "test message".to_string(),
            };
            let data = bincode::serialize(&message).unwrap();
            transport.broker().publish("test_topic", data).unwrap();
        }

        // Should receive the message
        let received = subscriber.try_recv().unwrap();
        assert!(received.is_some());
        assert_eq!(received.unwrap().data, "test message");

        context.shutdown().await.unwrap();
    }
}
