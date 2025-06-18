//! Subscriber implementation for MiniROS

use crate::core::Context;
use crate::error::Result;
use crate::message::Message;
use crossbeam_channel::{Receiver, unbounded};
use std::marker::PhantomData;
use std::sync::Arc;
use tokio::sync::mpsc;

/// Subscriber for receiving messages from a topic
/// 
/// Subscribers receive messages published to topics they are interested in.
/// Messages are received asynchronously and can be processed via callbacks or polling.
pub struct Subscriber<T: Message> {
    topic: String,
    endpoint: String,
    #[allow(dead_code)]
    context: Context,
    receiver: Receiver<T>,
    _stop_sender: mpsc::UnboundedSender<()>,
    _phantom: PhantomData<T>,
}

impl<T: Message> Subscriber<T> {
    /// Create a new subscriber for the given topic
    pub(crate) async fn new(topic: &str, endpoint: &str, context: Context) -> Result<Self> {
        tracing::debug!("Creating subscriber for topic: {} on endpoint: {}", topic, endpoint);

        // Set up message receiver channel
        let (msg_sender, msg_receiver) = unbounded();
        let (stop_sender, mut stop_receiver) = mpsc::unbounded_channel();

        // Start listening on the transport
        let transport_receiver = {
            let transport = context.inner.transport.read();
            transport.listen(endpoint).await?
        };

        // Spawn background task to process incoming messages
        let topic_clone = topic.to_string();
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    // Receive raw data from transport
                    raw_data_result = async {
                        transport_receiver.recv()
                    } => {
                        if let Ok(data) = raw_data_result {
                            // Deserialize the message
                            match bincode::deserialize::<T>(&data) {
                                Ok(message) => {
                                    tracing::debug!("Received message on topic: {}", topic_clone);
                                    if msg_sender.send(message).is_err() {
                                        tracing::debug!("Message receiver dropped for topic: {}", topic_clone);
                                        break;
                                    }
                                }
                                Err(e) => {
                                    tracing::warn!("Failed to deserialize message on topic {}: {}", topic_clone, e);
                                }
                            }
                        } else {
                            // Transport closed
                            break;
                        }
                    }
                    // Stop signal
                    _ = stop_receiver.recv() => {
                        tracing::debug!("Stopping subscriber for topic: {}", topic_clone);
                        break;
                    }
                }
            }
        });

        Ok(Subscriber {
            topic: topic.to_string(),
            endpoint: endpoint.to_string(),
            context,
            receiver: msg_receiver,
            _stop_sender: stop_sender,
            _phantom: PhantomData,
        })
    }

    /// Receive the next message (blocking)
    /// 
    /// Returns `None` if the subscriber is closed or no more messages are available.
    pub fn recv(&self) -> Option<T> {
        self.receiver.recv().ok()
    }

    /// Try to receive a message without blocking
    /// 
    /// Returns `None` if no message is currently available.
    pub fn try_recv(&self) -> Option<T> {
        self.receiver.try_recv().ok()
    }

    /// Get an iterator over messages
    /// 
    /// This iterator will block on each call to `next()` until a message is available.
    pub fn iter(&self) -> impl Iterator<Item = T> + '_ {
        self.receiver.iter()
    }

    /// Set up a callback function to process messages asynchronously
    /// 
    /// The callback will be called for each received message.
    /// This method spawns a background task and returns immediately.
    pub fn on_message<F>(&self, mut callback: F) -> Result<()>
    where
        F: FnMut(T) + Send + 'static,
    {
        let receiver = self.receiver.clone();
        let topic = self.topic.clone();

        tokio::spawn(async move {
            for message in receiver.iter() {
                tracing::debug!("Processing message callback for topic: {}", topic);
                callback(message);
            }
        });

        Ok(())
    }

    /// Set up an async callback function to process messages
    /// 
    /// Similar to `on_message` but for async callbacks.
    pub fn on_message_async<F, Fut>(&self, callback: F) -> Result<()>
    where
        F: Fn(T) -> Fut + Send + Sync + 'static,
        Fut: std::future::Future<Output = ()> + Send + 'static,
        T: 'static,
    {
        let receiver = self.receiver.clone();
        let topic = self.topic.clone();
        let callback = Arc::new(callback);

        tokio::spawn(async move {
            for message in receiver.iter() {
                tracing::debug!("Processing async message callback for topic: {}", topic);
                let callback_clone = callback.clone();
                tokio::spawn(async move {
                    callback_clone(message).await;
                });
            }
        });

        Ok(())
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Get the subscriber endpoint
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Check if there are any messages waiting
    pub fn has_messages(&self) -> bool {
        !self.receiver.is_empty()
    }

    /// Get the number of messages currently in the queue
    pub fn message_count(&self) -> usize {
        self.receiver.len()
    }
}

impl<T: Message> Drop for Subscriber<T> {
    fn drop(&mut self) {
        // Send stop signal when subscriber is dropped
        let _ = self._stop_sender.send(());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::StringMsg;
    use crate::core::Context;


    #[tokio::test]
    async fn test_subscriber_creation() {
        let context = Context::with_domain_id(30).unwrap();
        context.init().await.unwrap();
        
        let subscriber = Subscriber::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone()).await.unwrap();
        
        assert_eq!(subscriber.topic(), "test_topic");
        assert!(!subscriber.has_messages());
        
        drop(subscriber);
        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_subscriber_try_recv() {
        let context = Context::with_domain_id(31).unwrap();
        context.init().await.unwrap();
        
        let subscriber = Subscriber::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone()).await.unwrap();
        
        // Should return None when no messages
        assert!(subscriber.try_recv().is_none());
        
        drop(subscriber);
        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn test_subscriber_callback() {
        let context = Context::with_domain_id(32).unwrap();
        context.init().await.unwrap();
        
        let subscriber = Subscriber::<StringMsg>::new("test_topic", "127.0.0.1:0", context.clone()).await.unwrap();
        
        let received_messages = Arc::new(std::sync::Mutex::new(Vec::new()));
        let received_clone = received_messages.clone();
        
        subscriber.on_message(move |msg: StringMsg| {
            received_clone.lock().unwrap().push(msg.data);
        }).unwrap();
        
        // Test passes if callback setup doesn't panic
        // No actual messages expected in this unit test
        
        // Immediately drop subscriber to stop background tasks
        drop(subscriber);
        
        context.shutdown().await.unwrap();
    }
}