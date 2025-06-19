//! DDS-based transport layer for ROS2 compatibility
//!
//! This module provides a simplified DDS (Data Distribution Service) implementation
//! that maintains compatibility with ROS2's communication patterns while keeping
//! the implementation minimal and focused on core functionality.

use crate::error::{MiniRosError, Result};
use crate::message::Message;

use crossbeam_channel::{Receiver, Sender, unbounded};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::{RwLock, mpsc};

/// DDS Quality of Service (QoS) policies
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QosPolicy {
    pub reliability: ReliabilityKind,
    pub durability: DurabilityKind,
    pub history: HistoryKind,
    pub depth: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ReliabilityKind {
    BestEffort,
    Reliable,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DurabilityKind {
    Volatile,
    TransientLocal,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HistoryKind {
    KeepLast,
    KeepAll,
}

impl Default for QosPolicy {
    fn default() -> Self {
        Self {
            reliability: ReliabilityKind::Reliable,
            durability: DurabilityKind::Volatile,
            history: HistoryKind::KeepLast,
            depth: 10,
        }
    }
}

/// DDS Domain Participant - manages communication within a domain
pub struct DomainParticipant {
    domain_id: u32,
    socket: Arc<UdpSocket>,
    publishers: Arc<RwLock<HashMap<String, DdsPublisher>>>,
    subscribers: Arc<RwLock<HashMap<String, DdsSubscriber>>>,
    discovery_port: u16,
}

impl DomainParticipant {
    /// Create new domain participant with domain isolation
    ///
    /// Domain ID provides network isolation between different robot systems.
    /// Valid range: 0-232 (ROS2 compatible)
    pub async fn new(domain_id: u32) -> Result<Self> {
        // Validate domain ID range (ROS2 compatible)
        if domain_id > 232 {
            return Err(MiniRosError::ConfigError(format!(
                "Domain ID {} exceeds maximum of 232",
                domain_id
            )));
        }

        // Use domain-specific port to ensure isolation
        let discovery_port = 7400 + domain_id as u16;

        tracing::info!(
            "Creating DDS participant for domain {} on port {}",
            domain_id,
            discovery_port
        );

        let socket = UdpSocket::bind(format!("0.0.0.0:{}", discovery_port))
            .await
            .map_err(|e| {
                MiniRosError::NetworkError(format!(
                    "Failed to bind DDS socket for domain {}: {}",
                    domain_id, e
                ))
            })?;

        // Enable multicast for discovery with domain-specific address
        let multicast_addr = format!(
            "239.255.{}.{}",
            (domain_id / 256) as u8,
            (domain_id % 256) as u8
        );

        socket
            .join_multicast_v4(multicast_addr.parse().unwrap(), "0.0.0.0".parse().unwrap())
            .map_err(|e| {
                MiniRosError::NetworkError(format!(
                    "Failed to join multicast {} for domain {}: {}",
                    multicast_addr, domain_id, e
                ))
            })?;

        tracing::info!("DDS participant joined multicast group: {}", multicast_addr);

        Ok(Self {
            domain_id,
            socket: Arc::new(socket),
            publishers: Arc::new(RwLock::new(HashMap::new())),
            subscribers: Arc::new(RwLock::new(HashMap::new())),
            discovery_port,
        })
    }

    /// Create DDS publisher for topic
    pub async fn create_publisher<T: Message>(
        &self,
        topic: &str,
        qos: QosPolicy,
    ) -> Result<DdsPublisher> {
        let publisher =
            DdsPublisher::new(self.socket.clone(), topic.to_string(), qos, self.domain_id).await?;

        self.publishers
            .write()
            .await
            .insert(topic.to_string(), publisher.clone());
        Ok(publisher)
    }

    /// Create DDS subscriber for topic
    pub async fn create_subscriber<T: Message>(
        &self,
        topic: &str,
        qos: QosPolicy,
    ) -> Result<DdsSubscriber> {
        let subscriber =
            DdsSubscriber::new(self.socket.clone(), topic.to_string(), qos, self.domain_id).await?;

        self.subscribers
            .write()
            .await
            .insert(topic.to_string(), subscriber.clone());
        Ok(subscriber)
    }

    /// Get domain ID
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }
}

/// DDS Publisher - publishes data to a topic
#[derive(Clone)]
pub struct DdsPublisher {
    socket: Arc<UdpSocket>,
    topic: String,
    qos: QosPolicy,
    domain_id: u32,
    sequence_number: Arc<RwLock<u64>>,
}

impl DdsPublisher {
    async fn new(
        socket: Arc<UdpSocket>,
        topic: String,
        qos: QosPolicy,
        domain_id: u32,
    ) -> Result<Self> {
        Ok(Self {
            socket,
            topic,
            qos,
            domain_id,
            sequence_number: Arc::new(RwLock::new(0)),
        })
    }

    /// Publish message to topic
    pub async fn publish<T: Message>(&self, message: &T) -> Result<()> {
        let mut seq_num = self.sequence_number.write().await;
        *seq_num += 1;

        let dds_message = DdsMessage {
            topic: self.topic.clone(),
            sequence_number: *seq_num,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            data: bincode::serialize(message)
                .map_err(|e| MiniRosError::SerializationError(e.to_string()))?,
        };

        let serialized = bincode::serialize(&dds_message)
            .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;

        // Multicast to all subscribers in the same domain
        let multicast_addr = format!(
            "239.255.{}.{}:{}",
            (self.domain_id / 256) as u8,
            (self.domain_id % 256) as u8,
            7400 + self.domain_id
        );

        self.socket
            .send_to(&serialized, &multicast_addr)
            .await
            .map_err(|e| {
                MiniRosError::NetworkError(format!(
                    "Failed to publish to domain {}: {}",
                    self.domain_id, e
                ))
            })?;

        tracing::debug!("Published DDS message on topic: {}", self.topic);
        Ok(())
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// DDS Subscriber - receives data from a topic
#[derive(Clone)]
pub struct DdsSubscriber {
    receiver: Receiver<Vec<u8>>,
    topic: String,
    qos: QosPolicy,
    _domain_id: u32,
}

impl DdsSubscriber {
    async fn new(
        socket: Arc<UdpSocket>,
        topic: String,
        qos: QosPolicy,
        domain_id: u32,
    ) -> Result<Self> {
        let (sender, receiver) = unbounded();
        let topic_filter = topic.clone();

        // Spawn receiver task
        tokio::spawn(async move {
            let mut buffer = [0u8; 65536];
            loop {
                match socket.recv_from(&mut buffer).await {
                    Ok((len, _addr)) => {
                        if let Ok(dds_msg) = bincode::deserialize::<DdsMessage>(&buffer[..len]) {
                            if dds_msg.topic == topic_filter {
                                if sender.send(dds_msg.data).is_err() {
                                    break; // Receiver dropped
                                }
                            }
                        }
                    }
                    Err(_) => break,
                }
            }
        });

        Ok(Self {
            receiver,
            topic,
            qos,
            _domain_id: domain_id,
        })
    }

    /// Receive next message (blocking)
    pub fn recv<T: Message>(&self) -> Option<T> {
        if let Ok(data) = self.receiver.recv() {
            bincode::deserialize(&data).ok()
        } else {
            None
        }
    }

    /// Try to receive message without blocking
    pub fn try_recv<T: Message>(&self) -> Option<T> {
        if let Ok(data) = self.receiver.try_recv() {
            bincode::deserialize(&data).ok()
        } else {
            None
        }
    }

    /// Set up callback for incoming messages
    pub fn on_message<T: Message, F>(&self, mut callback: F) -> Result<()>
    where
        F: FnMut(T) + Send + 'static,
        T: 'static,
    {
        let receiver = self.receiver.clone();
        let topic = self.topic.clone();

        tokio::spawn(async move {
            while let Ok(data) = receiver.recv() {
                if let Ok(message) = bincode::deserialize::<T>(&data) {
                    callback(message);
                } else {
                    tracing::warn!("Failed to deserialize message on topic: {}", topic);
                }
            }
        });

        Ok(())
    }

    /// Get topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// DDS message wrapper with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
struct DdsMessage {
    topic: String,
    sequence_number: u64,
    timestamp: u64,
    data: Vec<u8>,
}

/// DDS Transport Manager - simplified DDS implementation
pub struct DdsTransport {
    participant: DomainParticipant,
}

impl DdsTransport {
    /// Create new DDS transport with domain ID
    pub async fn new(domain_id: u32) -> Result<Self> {
        let participant = DomainParticipant::new(domain_id).await?;
        Ok(Self { participant })
    }

    /// Create publisher with default QoS
    pub async fn create_publisher<T: Message>(&self, topic: &str) -> Result<DdsPublisher> {
        self.participant
            .create_publisher::<T>(topic, QosPolicy::default())
            .await
    }

    /// Create subscriber with default QoS
    pub async fn create_subscriber<T: Message>(&self, topic: &str) -> Result<DdsSubscriber> {
        self.participant
            .create_subscriber::<T>(topic, QosPolicy::default())
            .await
    }

    /// Create publisher with custom QoS
    pub async fn create_publisher_with_qos<T: Message>(
        &self,
        topic: &str,
        qos: QosPolicy,
    ) -> Result<DdsPublisher> {
        self.participant.create_publisher::<T>(topic, qos).await
    }

    /// Create subscriber with custom QoS
    pub async fn create_subscriber_with_qos<T: Message>(
        &self,
        topic: &str,
        qos: QosPolicy,
    ) -> Result<DdsSubscriber> {
        self.participant.create_subscriber::<T>(topic, qos).await
    }

    /// Get domain ID
    pub fn domain_id(&self) -> u32 {
        self.participant.domain_id()
    }

    /// Shutdown DDS transport
    pub async fn shutdown(&self) -> Result<()> {
        // Clear publishers and subscribers
        self.participant.publishers.write().await.clear();
        self.participant.subscribers.write().await.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::StringMsg;

    #[tokio::test]
    async fn test_dds_transport_creation() {
        let transport = DdsTransport::new(0).await.unwrap();
        assert_eq!(transport.domain_id(), 0);
    }

    #[tokio::test]
    async fn test_dds_publisher_subscriber() {
        let transport = DdsTransport::new(1).await.unwrap();

        let publisher = transport
            .create_publisher::<StringMsg>("test_topic")
            .await
            .unwrap();
        let subscriber = transport
            .create_subscriber::<StringMsg>("test_topic")
            .await
            .unwrap();

        assert_eq!(publisher.topic(), "test_topic");
        assert_eq!(subscriber.topic(), "test_topic");
    }
}
