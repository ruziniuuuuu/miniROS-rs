//! High-performance transport layer for MiniROS

use crate::error::{MiniRosError, Result};

use crossbeam_channel::{Receiver, Sender, unbounded};
use dashmap::DashMap;

use std::net::SocketAddr;
use std::sync::Arc;
use tokio::net::{TcpListener, TcpStream, UdpSocket};
use tokio::sync::mpsc;

/// Message broker for in-memory communication
pub struct MessageBroker {
    subscribers: Arc<DashMap<String, Vec<Sender<Vec<u8>>>>>,
}

impl Default for MessageBroker {
    fn default() -> Self {
        Self::new()
    }
}

impl MessageBroker {
    pub fn new() -> Self {
        MessageBroker {
            subscribers: Arc::new(DashMap::new()),
        }
    }

    /// Publish a message to all subscribers of a topic
    pub fn publish(&self, topic: &str, data: Vec<u8>) -> Result<usize> {
        if let Some(subscribers) = self.subscribers.get(topic) {
            let mut sent_count = 0;
            // Remove dead subscribers while sending
            let mut active_subscribers = Vec::new();
            
            for sender in subscribers.iter() {
                if sender.send(data.clone()).is_ok() {
                    active_subscribers.push(sender.clone());
                    sent_count += 1;
                }
            }
            
            // Update the subscriber list with only active ones
            self.subscribers.insert(topic.to_string(), active_subscribers);
            Ok(sent_count)
        } else {
            Ok(0)
        }
    }

    /// Subscribe to a topic
    pub fn subscribe(&self, topic: &str) -> Receiver<Vec<u8>> {
        let (tx, rx) = unbounded();
        
        self.subscribers
            .entry(topic.to_string())
            .or_default()
            .push(tx);
        
        rx
    }

    /// Get subscriber count for a topic
    pub fn subscriber_count(&self, topic: &str) -> usize {
        self.subscribers
            .get(topic)
            .map(|subs| subs.len())
            .unwrap_or(0)
    }
}

/// Transport layer abstraction for different communication protocols
#[async_trait::async_trait]
pub trait Transport: Send + Sync {
    /// Send a message to the specified endpoint
    async fn send(&self, endpoint: &str, data: &[u8]) -> Result<()>;

    /// Start listening for incoming messages
    async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>>;

    /// Stop the transport
    async fn stop(&self) -> Result<()>;
}

/// UDP-based transport for high-performance local communication
pub struct UdpTransport {
    receivers: Arc<DashMap<String, Sender<Vec<u8>>>>,
}

impl Default for UdpTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl UdpTransport {
    pub fn new() -> Self {
        UdpTransport {
            receivers: Arc::new(DashMap::new()),
        }
    }
}

#[async_trait::async_trait]
impl Transport for UdpTransport {
    async fn send(&self, endpoint: &str, data: &[u8]) -> Result<()> {
        let addr: SocketAddr = endpoint
            .parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;

        let socket = UdpSocket::bind("0.0.0.0:0")
            .await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;

        socket
            .send_to(data, addr)
            .await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;

        Ok(())
    }

    async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        let addr: SocketAddr = endpoint
            .parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;

        let socket = Arc::new(
            UdpSocket::bind(addr)
                .await
                .map_err(|e| MiniRosError::NetworkError(e.to_string()))?,
        );

        let (tx, rx) = unbounded();
        self.receivers.insert(endpoint.to_string(), tx.clone());

        // Spawn background task to receive messages
        let receivers_clone = self.receivers.clone();
        let endpoint_clone = endpoint.to_string();

        tokio::spawn(async move {
            let mut buffer = [0u8; 65536];
            while let Ok(len) = socket.recv(&mut buffer).await {
                let data = buffer[..len].to_vec();
                if let Some(sender) = receivers_clone.get(&endpoint_clone) {
                    if sender.send(data).is_err() {
                        break; // Receiver dropped
                    }
                }
            }

            // Clean up when done
            receivers_clone.remove(&endpoint_clone);
        });

        Ok(rx)
    }

    async fn stop(&self) -> Result<()> {
        self.receivers.clear();
        Ok(())
    }
}

/// TCP-based transport for reliable communication
pub struct TcpTransport {
    listeners: Arc<DashMap<String, mpsc::UnboundedSender<()>>>,
}

impl Default for TcpTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl TcpTransport {
    pub fn new() -> Self {
        TcpTransport {
            listeners: Arc::new(DashMap::new()),
        }
    }
}

#[async_trait::async_trait]
impl Transport for TcpTransport {
    async fn send(&self, endpoint: &str, data: &[u8]) -> Result<()> {
        let addr: SocketAddr = endpoint
            .parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;

        let mut stream = TcpStream::connect(addr)
            .await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;

        use tokio::io::AsyncWriteExt;
        stream
            .write_all(data)
            .await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;

        Ok(())
    }

    async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        let addr: SocketAddr = endpoint
            .parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;

        let listener = TcpListener::bind(addr)
            .await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;

        let (tx, rx) = unbounded();
        let (stop_tx, mut stop_rx) = mpsc::unbounded_channel();

        self.listeners.insert(endpoint.to_string(), stop_tx);

        // Spawn background task to accept connections
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = listener.accept() => {
                        match result {
                            Ok((mut stream, _)) => {
                                let tx_clone = tx.clone();
                                tokio::spawn(async move {
                                    let mut buffer = [0u8; 65536];
                                    use tokio::io::AsyncReadExt;
                                    loop {
                                        match stream.read(&mut buffer).await {
                                            Ok(0) => break, // Connection closed
                                            Ok(len) => {
                                                let data = buffer[..len].to_vec();
                                                if tx_clone.send(data).is_err() {
                                                    break; // Receiver dropped
                                                }
                                            }
                                            Err(_) => break,
                                        }
                                    }
                                });
                            }
                            Err(_) => break,
                        }
                    }
                    _ = stop_rx.recv() => {
                        break;
                    }
                }
            }
        });

        Ok(rx)
    }

    async fn stop(&self) -> Result<()> {
        for item in self.listeners.iter() {
            let _ = item.value().send(());
        }
        self.listeners.clear();
        Ok(())
    }
}

/// Transport manager that coordinates different transport types
pub struct TransportManager {
    udp_transport: UdpTransport,
    tcp_transport: TcpTransport,
    message_broker: Arc<MessageBroker>,
}

impl TransportManager {
    pub fn new() -> Result<Self> {
        Ok(TransportManager {
            udp_transport: UdpTransport::new(),
            tcp_transport: TcpTransport::new(),
            message_broker: Arc::new(MessageBroker::new()),
        })
    }

    pub async fn start(&mut self) -> Result<()> {
        tracing::debug!("Transport manager started");
        Ok(())
    }

    pub async fn stop(&mut self) -> Result<()> {
        self.udp_transport.stop().await?;
        self.tcp_transport.stop().await?;
        tracing::debug!("Transport manager stopped");
        Ok(())
    }

    /// Get access to the message broker for in-memory communication
    pub fn broker(&self) -> &Arc<MessageBroker> {
        &self.message_broker
    }

    pub fn udp(&self) -> &UdpTransport {
        &self.udp_transport
    }

    pub fn tcp(&self) -> &TcpTransport {
        &self.tcp_transport
    }

    /// Send data using the appropriate transport
    pub async fn send(&self, endpoint: &str, data: &[u8]) -> Result<()> {
        if endpoint.starts_with("tcp://") {
            let endpoint = endpoint.strip_prefix("tcp://").unwrap();
            self.tcp_transport.send(endpoint, data).await
        } else if endpoint.starts_with("udp://") {
            let endpoint = endpoint.strip_prefix("udp://").unwrap();
            self.udp_transport.send(endpoint, data).await
        } else {
            // Default to TCP
            self.tcp_transport.send(endpoint, data).await
        }
    }

    /// Listen on an endpoint using the appropriate transport
    pub async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        if endpoint.starts_with("tcp://") {
            let endpoint = endpoint.strip_prefix("tcp://").unwrap();
            self.tcp_transport.listen(endpoint).await
        } else if endpoint.starts_with("udp://") {
            let endpoint = endpoint.strip_prefix("udp://").unwrap();
            self.udp_transport.listen(endpoint).await
        } else {
            // Default to TCP
            self.tcp_transport.listen(endpoint).await
        }
    }
}
