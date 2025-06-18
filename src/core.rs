//! Core MiniROS context and configuration

use crate::error::Result;
use crate::discovery::DiscoveryService;

// Import transport based on feature flags
#[cfg(feature = "tcp-transport")]
use crate::transport::TransportManager;
#[cfg(feature = "dds-transport")]
use crate::dds_transport::DdsTransport;

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
    #[cfg(feature = "tcp-transport")]
    pub transport: Arc<RwLock<TransportManager>>,
    #[cfg(feature = "dds-transport")]
    pub dds_transport: Arc<RwLock<Option<DdsTransport>>>,
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

        let inner = ContextInner {
            domain_id,
            instance_id,
            discovery,
            #[cfg(feature = "tcp-transport")]
            transport: Arc::new(RwLock::new(TransportManager::new()?)),
            #[cfg(feature = "dds-transport")]
            dds_transport: Arc::new(RwLock::new(None)),
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

        // Initialize transport based on feature flags
        #[cfg(feature = "tcp-transport")]
        {
            let mut transport = self.inner.transport.write();
            transport.start().await?;
        }

        #[cfg(feature = "dds-transport")]
        {
            let dds_transport = DdsTransport::new(self.domain_id()).await?;
            let mut transport_lock = self.inner.dds_transport.write();
            *transport_lock = Some(dds_transport);
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

        // Stop transport based on feature flags
        #[cfg(feature = "tcp-transport")]
        {
            let mut transport = self.inner.transport.write();
            transport.stop().await?;
        }

        #[cfg(feature = "dds-transport")]
        {
            let mut transport_lock = self.inner.dds_transport.write();
            if let Some(dds_transport) = transport_lock.as_ref() {
                dds_transport.shutdown().await?;
            }
            *transport_lock = None;
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