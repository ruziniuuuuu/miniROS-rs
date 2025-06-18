//! Node and service discovery for MiniROS

use crate::error::{Result, MiniRosError};
use dashmap::DashMap;
use serde::{Deserialize, Serialize};

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use tokio::time::interval;
use uuid::Uuid;

/// Information about a discovered node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeInfo {
    pub id: Uuid,
    pub name: String,
    pub domain_id: u32,
    pub endpoint: String,
    pub last_seen: u64, // Unix timestamp in seconds
    pub publishers: Vec<TopicInfo>,
    pub subscribers: Vec<TopicInfo>,
    pub services: Vec<ServiceInfo>,
}

/// Information about a topic
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub type_name: String,
    pub endpoint: String,
}

/// Information about a service
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceInfo {
    pub name: String,
    pub type_name: String,
    pub endpoint: String,
}

/// Discovery message types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DiscoveryMessage {
    /// Announce node presence
    NodeAnnouncement(NodeInfo),
    /// Request node information
    NodeQuery { domain_id: u32 },
    /// Response to node query
    NodeResponse(Vec<NodeInfo>),
    /// Heartbeat to maintain presence
    Heartbeat { node_id: Uuid, timestamp: u64 },
}

/// Discovery service for finding nodes and topics
pub struct DiscoveryService {
    domain_id: u32,
    socket: Option<Arc<UdpSocket>>,
    nodes: Arc<DashMap<Uuid, NodeInfo>>,
    multicast_addr: SocketAddr,
    local_endpoint: String,
    _stop_sender: Option<mpsc::UnboundedSender<()>>,
}

impl DiscoveryService {
    const MULTICAST_ADDR: &'static str = "239.255.0.1:7400";
    const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(30);
    const NODE_TIMEOUT: Duration = Duration::from_secs(90);

    pub fn new(domain_id: u32) -> Result<Self> {
        let multicast_addr = Self::MULTICAST_ADDR.parse()
            .map_err(|e| MiniRosError::NetworkError(format!("Invalid multicast address: {}", e)))?;

        Ok(DiscoveryService {
            domain_id,
            socket: None,
            nodes: Arc::new(DashMap::new()),
            multicast_addr,
            local_endpoint: format!("0.0.0.0:{}", 7400 + domain_id),
            _stop_sender: None,
        })
    }

    /// Start the discovery service
    pub async fn start(&mut self) -> Result<()> {
        tracing::debug!("Starting discovery service for domain {}", self.domain_id);

        // Bind to local address for receiving discovery messages
        let socket = Arc::new(UdpSocket::bind(&self.local_endpoint).await
            .map_err(|e| MiniRosError::NetworkError(format!("Failed to bind discovery socket: {}", e)))?);

        // Join multicast group
        #[cfg(unix)]
        {
            use std::net::Ipv4Addr;
            socket.join_multicast_v4(
                "239.255.0.1".parse::<Ipv4Addr>().unwrap(),
                "0.0.0.0".parse::<Ipv4Addr>().unwrap(),
            ).map_err(|e| MiniRosError::NetworkError(format!("Failed to join multicast: {}", e)))?;
        }

        self.socket = Some(socket.clone());

        let (stop_tx, mut stop_rx) = mpsc::unbounded_channel();
        self._stop_sender = Some(stop_tx);

        // Start background tasks
        let socket_clone = socket.clone();
        let nodes_clone = self.nodes.clone();

        // Message receiver task
        tokio::spawn(async move {
            let mut buffer = [0u8; 65536];
            loop {
                tokio::select! {
                    result = socket_clone.recv_from(&mut buffer) => {
                        match result {
                            Ok((len, _addr)) => {
                                if let Ok(msg) = bincode::deserialize::<DiscoveryMessage>(&buffer[..len]) {
                                    Self::handle_discovery_message(msg, &nodes_clone).await;
                                }
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

        // Heartbeat and cleanup task
        let nodes_cleanup = self.nodes.clone();
        let socket_heartbeat = socket.clone();
        let multicast_addr = self.multicast_addr;
        
        tokio::spawn(async move {
            let mut heartbeat_timer = interval(Self::HEARTBEAT_INTERVAL);
            let mut cleanup_timer = interval(Duration::from_secs(60));

            loop {
                tokio::select! {
                    _ = heartbeat_timer.tick() => {
                        // Send periodic node query to discover new nodes
                        let query = DiscoveryMessage::NodeQuery { 
                            domain_id: 0 // Query all domains for now
                        };
                        if let Ok(data) = bincode::serialize(&query) {
                            let _ = socket_heartbeat.send_to(&data, multicast_addr).await;
                        }
                    }
                    _ = cleanup_timer.tick() => {
                        // Remove stale nodes
                        let now = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_secs();
                        
                        nodes_cleanup.retain(|_, node| {
                            now - node.last_seen < Self::NODE_TIMEOUT.as_secs()
                        });
                    }
                }
            }
        });

        tracing::info!("Discovery service started on {}", self.local_endpoint);
        Ok(())
    }

    /// Stop the discovery service
    pub async fn stop(&mut self) -> Result<()> {
        tracing::debug!("Stopping discovery service");
        
        if let Some(stop_sender) = self._stop_sender.take() {
            let _ = stop_sender.send(());
        }
        
        self.socket = None;
        self.nodes.clear();
        
        Ok(())
    }

    /// Announce a node to the network
    pub async fn announce_node(&self, node_info: NodeInfo) -> Result<()> {
        if let Some(socket) = &self.socket {
            let msg = DiscoveryMessage::NodeAnnouncement(node_info.clone());
            let data = bincode::serialize(&msg)
                .map_err(|e| MiniRosError::SerializationError(e.to_string()))?;
            
            socket.send_to(&data, self.multicast_addr).await
                .map_err(|e| MiniRosError::NetworkError(e.to_string()))?;
            
            // Also store locally
            self.nodes.insert(node_info.id, node_info);
        }
        Ok(())
    }

    /// Get all discovered nodes
    pub fn get_nodes(&self) -> Vec<NodeInfo> {
        self.nodes.iter().map(|entry| entry.value().clone()).collect()
    }

    /// Find nodes by name
    pub fn find_nodes_by_name(&self, name: &str) -> Vec<NodeInfo> {
        self.nodes
            .iter()
            .filter(|entry| entry.value().name == name)
            .map(|entry| entry.value().clone())
            .collect()
    }

    /// Get all publishers for a topic
    pub fn get_publishers(&self, topic: &str) -> Vec<(NodeInfo, TopicInfo)> {
        let mut publishers = Vec::new();
        for node_entry in self.nodes.iter() {
            let node = node_entry.value();
            for pub_info in &node.publishers {
                if pub_info.name == topic {
                    publishers.push((node.clone(), pub_info.clone()));
                }
            }
        }
        publishers
    }

    /// Get all subscribers for a topic  
    pub fn get_subscribers(&self, topic: &str) -> Vec<(NodeInfo, TopicInfo)> {
        let mut subscribers = Vec::new();
        for node_entry in self.nodes.iter() {
            let node = node_entry.value();
            for sub_info in &node.subscribers {
                if sub_info.name == topic {
                    subscribers.push((node.clone(), sub_info.clone()));
                }
            }
        }
        subscribers
    }

    /// Get all service providers for a service
    pub fn get_service_providers(&self, service: &str) -> Vec<(NodeInfo, ServiceInfo)> {
        let mut providers = Vec::new();
        for node_entry in self.nodes.iter() {
            let node = node_entry.value();
            for service_info in &node.services {
                if service_info.name == service {
                    providers.push((node.clone(), service_info.clone()));
                }
            }
        }
        providers
    }

    async fn handle_discovery_message(msg: DiscoveryMessage, nodes: &DashMap<Uuid, NodeInfo>) {
        match msg {
            DiscoveryMessage::NodeAnnouncement(node_info) => {
                tracing::debug!("Discovered node: {}", node_info.name);
                nodes.insert(node_info.id, node_info);
            }
            DiscoveryMessage::NodeQuery { domain_id: _domain_id } => {
                // Could respond with our known nodes, but keeping it simple for now
            }
            DiscoveryMessage::NodeResponse(node_list) => {
                for node_info in node_list {
                    nodes.insert(node_info.id, node_info);
                }
            }
            DiscoveryMessage::Heartbeat { node_id, timestamp } => {
                if let Some(mut node) = nodes.get_mut(&node_id) {
                    node.last_seen = timestamp;
                }
            }
        }
    }
} 