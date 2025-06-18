//! Zenoh-style transport implementation for miniROS (simplified version)
//! 
//! Provides high-performance pub/sub communication with zenoh-compatible API.
//! This is a simplified implementation that will be upgraded to full Zenoh when
//! version compatibility issues are resolved.

use crate::error::{Result, MiniRosError};
use crate::message::Message;
use crossbeam_channel::{Receiver, Sender, unbounded};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::net::UdpSocket;

/// Zenoh-style transport configuration
#[derive(Clone, Debug)]
pub struct ZenohConfig {
    pub multicast_address: String,
    pub port_range: (u16, u16),
}

impl Default for ZenohConfig {
    fn default() -> Self {
        Self {
            multicast_address: "224.0.0.224".to_string(),
            port_range: (7447, 7500),
        }
    }
}

/// Simplified Zenoh-style transport implementation
pub struct ZenohTransport {
    socket: Arc<UdpSocket>,
    publishers: Arc<RwLock<HashMap<String, ZenohPublisher<Vec<u8>>>>>,
    subscribers: Arc<RwLock<HashMap<String, (Sender<Vec<u8>>, String)>>>,
    config: ZenohConfig,
}

impl ZenohTransport {
    /// Create new Zenoh-style transport
    pub async fn new(config: ZenohConfig) -> Result<Self> {
        // Create UDP socket for multicast communication
        let socket = UdpSocket::bind("0.0.0.0:0").await
            .map_err(|e| MiniRosError::NetworkError(format!("Failed to bind socket: {}", e)))?;

        // Join multicast group
        socket.join_multicast_v4(
            config.multicast_address.parse().unwrap(),
            "0.0.0.0".parse().unwrap()
        ).map_err(|e| MiniRosError::NetworkError(format!("Failed to join multicast: {}", e)))?;

        Ok(Self {
            socket: Arc::new(socket),
            publishers: Arc::new(RwLock::new(HashMap::new())),
            subscribers: Arc::new(RwLock::new(HashMap::new())),
            config,
        })
    }

    /// Create publisher for topic
    pub async fn create_publisher<T: Message>(&self, topic: &str) -> Result<ZenohPublisher<T>> {
        let publisher = ZenohPublisher {
            socket: self.socket.clone(),
            topic: topic.to_string(),
            multicast_addr: format!("{}:{}", self.config.multicast_address, 7447),
            _phantom: std::marker::PhantomData,
        };

        Ok(publisher)
    }

    /// Create subscriber for topic
    pub async fn create_subscriber<T: Message>(&self, topic: &str) -> Result<ZenohSubscriber<T>> {
        let (sender, receiver) = unbounded();
        
        // Store subscriber info
        {
            let mut subscribers = self.subscribers.write().await;
            subscribers.insert(topic.to_string(), (sender.clone(), topic.to_string()));
        }

        // Spawn receiver task
        let socket = self.socket.clone();
        let topic_filter = topic.to_string();
        let sender_clone = sender.clone();
        
        tokio::spawn(async move {
            let mut buf = [0u8; 65536];
            loop {
                match socket.recv_from(&mut buf).await {
                    Ok((len, _addr)) => {
                        // Simple topic filtering based on message prefix
                        if let Ok(msg_str) = std::str::from_utf8(&buf[..len]) {
                            if msg_str.starts_with(&format!("{}: ", topic_filter)) {
                                // Extract payload after topic prefix
                                if let Some(payload_start) = msg_str.find(": ") {
                                    let payload = &buf[payload_start + 2..len];
                                    if let Ok(message) = bincode::deserialize::<T>(payload) {
                                        let serialized = bincode::serialize(&message).unwrap_or_default();
                                        let _ = sender_clone.send(serialized);
                                    }
                                }
                            }
                        }
                    }
                    Err(_) => break,
                }
            }
        });

        Ok(ZenohSubscriber {
            receiver,
            topic: topic.to_string(),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Shutdown transport
    pub async fn shutdown(&self) -> Result<()> {
        // Clear all publishers and subscribers
        self.publishers.write().await.clear();
        self.subscribers.write().await.clear();
        Ok(())
    }
}

/// Zenoh-style publisher wrapper
pub struct ZenohPublisher<T: Message> {
    socket: Arc<UdpSocket>,
    topic: String,
    multicast_addr: String,
    _phantom: std::marker::PhantomData<T>,
}

impl<T: Message> ZenohPublisher<T> {
    /// Publish message
    pub async fn publish(&self, message: &T) -> Result<()> {
        let data = bincode::serialize(message)
            .map_err(|e| MiniRosError::SerializationError(format!("Failed to serialize: {}", e)))?;

        // Create message with topic prefix
        let topic_msg = format!("{}: ", self.topic);
        let mut full_msg = topic_msg.into_bytes();
        full_msg.extend_from_slice(&data);

        self.socket
            .send_to(&full_msg, &self.multicast_addr)
            .await
            .map_err(|e| MiniRosError::NetworkError(format!("Failed to publish: {}", e)))?;

        tracing::debug!("Published message on topic: {}", self.topic);
        Ok(())
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// Zenoh-style subscriber wrapper
pub struct ZenohSubscriber<T: Message> {
    receiver: Receiver<Vec<u8>>,
    topic: String,
    _phantom: std::marker::PhantomData<T>,
}

impl<T: Message> ZenohSubscriber<T> {
    /// Receive next message (blocking)
    pub fn recv(&self) -> Option<T> {
        if let Ok(data) = self.receiver.recv() {
            bincode::deserialize(&data).ok()
        } else {
            None
        }
    }

    /// Try to receive message without blocking
    pub fn try_recv(&self) -> Option<T> {
        if let Ok(data) = self.receiver.try_recv() {
            bincode::deserialize(&data).ok()
        } else {
            None
        }
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Set up callback for messages
    pub fn on_message<F>(&self, mut callback: F) -> Result<()>
    where
        F: FnMut(T) + Send + 'static,
    {
        let receiver = self.receiver.clone();
        let topic = self.topic.clone();

        tokio::spawn(async move {
            for data in receiver.iter() {
                if let Ok(message) = bincode::deserialize::<T>(&data) {
                    tracing::debug!("Processing message callback for topic: {}", topic);
                    callback(message);
                }
            }
        });

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::StringMsg;

    #[tokio::test]
    async fn test_zenoh_transport_creation() {
        let config = ZenohConfig::default();
        let result = ZenohTransport::new(config).await;
        assert!(result.is_ok(), "Zenoh-style transport should be created successfully");
    }

    #[tokio::test]
    async fn test_publisher_creation() {
        let config = ZenohConfig::default();
        let transport = ZenohTransport::new(config).await.unwrap();
        let publisher = transport.create_publisher::<StringMsg>("test_topic").await;
        assert!(publisher.is_ok(), "Publisher should be created successfully");
    }
} 