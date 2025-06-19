//! Core MiniROS context and configuration

use crate::discovery::DiscoveryService;
use crate::error::Result;

// Import transport based on feature flags
#[cfg(feature = "dds-transport")]
use crate::dds_transport::DdsTransport;
#[cfg(feature = "tcp-transport")]
use crate::transport::TransportManager;

use parking_lot::RwLock;
use std::sync::Arc;
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
    #[allow(clippy::await_holding_lock)]
    pub async fn init(&self) -> Result<()> {
        tracing::info!(
            "Initializing MiniROS context with domain_id: {}",
            self.domain_id()
        );

        // Start discovery service
        {
            let discovery = self.inner.discovery.clone();
            let mut discovery_guard = discovery.write();
            discovery_guard.start().await?;
        }

        // Initialize transport based on feature flags
        #[cfg(feature = "tcp-transport")]
        {
            let transport = self.inner.transport.clone();
            let mut transport_guard = transport.write();
            transport_guard.start().await?;
        }

        #[cfg(feature = "dds-transport")]
        {
            let dds_transport = DdsTransport::new(self.domain_id()).await?;
            *self.inner.dds_transport.write() = Some(dds_transport);
        }

        tracing::info!("MiniROS context initialized successfully");
        Ok(())
    }

    /// Shutdown the context and clean up resources
    #[allow(clippy::await_holding_lock)]
    pub async fn shutdown(&self) -> Result<()> {
        tracing::info!("Shutting down MiniROS context");

        // Stop discovery service
        {
            let discovery = self.inner.discovery.clone();
            let mut discovery_guard = discovery.write();
            discovery_guard.stop().await?;
        }

        // Stop transport based on feature flags
        #[cfg(feature = "tcp-transport")]
        {
            let transport = self.inner.transport.clone();
            let mut transport_guard = transport.write();
            transport_guard.stop().await?;
        }

        #[cfg(feature = "dds-transport")]
        {
            let transport_lock = self.inner.dds_transport.read();
            if let Some(dds_transport) = transport_lock.as_ref() {
                dds_transport.shutdown().await?;
            }
            drop(transport_lock);
            *self.inner.dds_transport.write() = None;
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
