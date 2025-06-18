//! High-performance transport layer for MiniROS

use crate::error::{Result, MiniRosError};

use crossbeam_channel::{unbounded, Receiver, Sender};
use dashmap::DashMap;

use std::sync::Arc;
use tokio::net::{UdpSocket, TcpListener, TcpStream};
use tokio::sync::mpsc;
use std::net::SocketAddr;

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
        let addr: SocketAddr = endpoint.parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;
        
        let socket = UdpSocket::bind("0.0.0.0:0").await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;
        
        socket.send_to(data, addr).await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;
        
        Ok(())
    }

    async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        let addr: SocketAddr = endpoint.parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;
        
        let socket = Arc::new(UdpSocket::bind(addr).await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?);

        let (tx, rx) = unbounded();
        self.receivers.insert(endpoint.to_string(), tx.clone());

        // Spawn background task to receive messages
        let receivers_clone = self.receivers.clone();
        let endpoint_clone = endpoint.to_string();
        
        tokio::spawn(async move {
            let mut buffer = [0u8; 65536];
            loop {
                match socket.recv(&mut buffer).await {
                    Ok(len) => {
                        let data = buffer[..len].to_vec();
                        if let Some(sender) = receivers_clone.get(&endpoint_clone) {
                            if sender.send(data).is_err() {
                                break; // Receiver dropped
                            }
                        }
                    }
                    Err(_) => break,
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
        let addr: SocketAddr = endpoint.parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;
        
        let mut stream = TcpStream::connect(addr).await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;
        
        use tokio::io::AsyncWriteExt;
        stream.write_all(data).await
            .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;
        
        Ok(())
    }

    async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        let addr: SocketAddr = endpoint.parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid endpoint: {}", e)))?;
        
        let listener = TcpListener::bind(addr).await
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

/// Transport manager that handles multiple transport protocols
pub struct TransportManager {
    udp_transport: UdpTransport,
    tcp_transport: TcpTransport,
}

impl TransportManager {
    pub fn new() -> Result<Self> {
        Ok(TransportManager {
            udp_transport: UdpTransport::new(),
            tcp_transport: TcpTransport::new(),
        })
    }

    pub async fn start(&mut self) -> Result<()> {
        tracing::debug!("Starting transport manager");
        Ok(())
    }

    pub async fn stop(&mut self) -> Result<()> {
        tracing::debug!("Stopping transport manager");
        self.udp_transport.stop().await?;
        self.tcp_transport.stop().await?;
        Ok(())
    }

    /// Get UDP transport instance
    pub fn udp(&self) -> &UdpTransport {
        &self.udp_transport
    }

    /// Get TCP transport instance
    pub fn tcp(&self) -> &TcpTransport {
        &self.tcp_transport
    }

    /// Send message using the appropriate transport based on endpoint protocol
    pub async fn send(&self, endpoint: &str, data: &[u8]) -> Result<()> {
        if endpoint.starts_with("udp://") {
            let addr = endpoint.strip_prefix("udp://").unwrap();
            self.udp_transport.send(addr, data).await
        } else if endpoint.starts_with("tcp://") {
            let addr = endpoint.strip_prefix("tcp://").unwrap();
            self.tcp_transport.send(addr, data).await
        } else {
            // Default to UDP for backward compatibility
            self.udp_transport.send(endpoint, data).await
        }
    }

    /// Listen on endpoint using the appropriate transport
    pub async fn listen(&self, endpoint: &str) -> Result<Receiver<Vec<u8>>> {
        if endpoint.starts_with("udp://") {
            let addr = endpoint.strip_prefix("udp://").unwrap();
            self.udp_transport.listen(addr).await
        } else if endpoint.starts_with("tcp://") {
            let addr = endpoint.strip_prefix("tcp://").unwrap();
            self.tcp_transport.listen(addr).await
        } else {
            // Default to UDP
            self.udp_transport.listen(endpoint).await
        }
    }
} 