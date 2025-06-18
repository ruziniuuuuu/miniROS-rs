//! Core MiniROS context and configuration

use crate::error::Result;
use crate::discovery::DiscoveryService;
use crate::transport::TransportManager;
use std::sync::Arc;
use parking_lot::RwLock;
use uuid::Uuid;

/// Global MiniROS context containing shared state
#[derive(Clone)]
pub struct Context {
    pub(crate) inner: Arc<ContextInner>,
}

pub(crate) struct ContextInner {
    pub domain_id: u32,
    pub instance_id: Uuid,
    pub discovery: Arc<RwLock<DiscoveryService>>,
    pub transport: Arc<RwLock<TransportManager>>,
}

impl Context {
    /// Create a new MiniROS context with default configuration
    pub fn new() -> Result<Self> {
        Self::with_domain_id(0)
    }

    /// Create a new MiniROS context with specified domain ID
    pub fn with_domain_id(domain_id: u32) -> Result<Self> {
        let instance_id = Uuid::new_v4();
        let discovery = Arc::new(RwLock::new(DiscoveryService::new(domain_id)?));
        let transport = Arc::new(RwLock::new(TransportManager::new()?));

        let inner = ContextInner {
            domain_id,
            instance_id,
            discovery,
            transport,
        };

        Ok(Context {
            inner: Arc::new(inner),
        })
    }

    /// Get the domain ID for this context
    pub fn domain_id(&self) -> u32 {
        self.inner.domain_id
    }

    /// Get the unique instance ID for this context
    pub fn instance_id(&self) -> Uuid {
        self.inner.instance_id
    }

    /// Initialize the context and start background services
    pub async fn init(&self) -> Result<()> {
        tracing::info!("Initializing MiniROS context with domain_id: {}", self.domain_id());
        
        // Start discovery service
        {
            let mut discovery = self.inner.discovery.write();
            discovery.start().await?;
        }

        // Initialize transport manager
        {
            let mut transport = self.inner.transport.write();
            transport.start().await?;
        }

        tracing::info!("MiniROS context initialized successfully");
        Ok(())
    }

    /// Shutdown the context and clean up resources
    pub async fn shutdown(&self) -> Result<()> {
        tracing::info!("Shutting down MiniROS context");

        // Stop discovery service
        {
            let mut discovery = self.inner.discovery.write();
            discovery.stop().await?;
        }

        // Stop transport manager
        {
            let mut transport = self.inner.transport.write();
            transport.stop().await?;
        }

        tracing::info!("MiniROS context shutdown complete");
        Ok(())
    }
}

impl Default for Context {
    fn default() -> Self {
        Self::new().expect("Failed to create default context")
    }
} 