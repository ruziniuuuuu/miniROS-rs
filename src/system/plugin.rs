//! Plugin System - Phase 3 Feature
//!
//! Provides an extensible architecture for custom transports, message handlers,
//! and other features while maintaining the "mini" philosophy of simplicity.

use crate::core::{MiniRosError, Result};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{debug, info, warn};

/// Plugin interface that all plugins must implement
#[async_trait]
pub trait Plugin: Send + Sync {
    /// Get plugin name
    fn name(&self) -> &str;
    
    /// Get plugin version
    fn version(&self) -> &str;
    
    /// Get plugin description
    fn description(&self) -> &str;
    
    /// Initialize the plugin
    async fn initialize(&mut self, context: &PluginContext) -> Result<()>;
    
    /// Start the plugin (called after initialization)
    async fn start(&mut self) -> Result<()>;
    
    /// Stop the plugin gracefully
    async fn stop(&mut self) -> Result<()>;
    
    /// Get plugin configuration schema
    fn config_schema(&self) -> PluginConfigSchema;
    
    /// Handle configuration updates
    async fn configure(&mut self, config: PluginConfig) -> Result<()>;
    
    /// Get plugin status
    fn status(&self) -> PluginStatus;
    
    /// Get plugin as Any for downcasting
    fn as_any(&self) -> &dyn Any;
    
    /// Get mutable plugin as Any for downcasting
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

/// Plugin context provides access to system resources
#[derive(Debug, Clone)]
pub struct PluginContext {
    /// System configuration
    pub system_config: HashMap<String, String>,
    /// Available services
    pub services: Vec<String>,
    /// Available topics
    pub topics: Vec<String>,
    /// Plugin registry for inter-plugin communication
    pub plugin_registry: Arc<RwLock<HashMap<String, Box<dyn Any + Send + Sync>>>>,
}

impl PluginContext {
    /// Create new plugin context
    pub fn new() -> Self {
        Self {
            system_config: HashMap::new(),
            services: Vec::new(),
            topics: Vec::new(),
            plugin_registry: Arc::new(RwLock::new(HashMap::new())),
        }
    }
    
    /// Get system configuration value
    pub fn get_config(&self, key: &str) -> Option<&String> {
        self.system_config.get(key)
    }
    
    /// Set system configuration value
    pub fn set_config(&mut self, key: String, value: String) {
        self.system_config.insert(key, value);
    }
}

/// Plugin configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginConfig {
    /// Plugin-specific settings
    pub settings: HashMap<String, serde_json::Value>,
    /// Plugin dependencies
    pub dependencies: Vec<String>,
    /// Plugin priority (higher = loaded first)
    pub priority: u32,
    /// Whether plugin is enabled
    pub enabled: bool,
}

impl Default for PluginConfig {
    fn default() -> Self {
        Self {
            settings: HashMap::new(),
            dependencies: Vec::new(),
            priority: 100,
            enabled: true,
        }
    }
}

/// Plugin configuration schema
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginConfigSchema {
    /// Schema fields
    pub fields: Vec<ConfigField>,
    /// Required fields
    pub required: Vec<String>,
}

/// Configuration field definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigField {
    /// Field name
    pub name: String,
    /// Field type
    pub field_type: ConfigFieldType,
    /// Field description
    pub description: String,
    /// Default value
    pub default: Option<serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConfigFieldType {
    String,
    Integer,
    Float,
    Boolean,
    Array,
    Object,
}

/// Plugin status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PluginStatus {
    Uninitialized,
    Initializing,
    Ready,
    Running,
    Stopping,
    Stopped,
    Error(String),
}

/// Plugin manager for loading and managing plugins
pub struct PluginManager {
    plugins: RwLock<HashMap<String, Box<dyn Plugin>>>,
    context: PluginContext,
    load_order: RwLock<Vec<String>>,
}

impl PluginManager {
    /// Create new plugin manager
    pub fn new() -> Self {
        Self {
            plugins: RwLock::new(HashMap::new()),
            context: PluginContext::new(),
            load_order: RwLock::new(Vec::new()),
        }
    }
    
    /// Register a plugin
    pub async fn register_plugin(&self, plugin: Box<dyn Plugin>) -> Result<()> {
        let name = plugin.name().to_string();
        
        info!("🔌 Registering plugin: {}", name);
        
        // Add to plugins map
        {
            let mut plugins = self.plugins.write().await;
            if plugins.contains_key(&name) {
                return Err(MiniRosError::Custom(format!("Plugin '{}' already registered", name)));
            }
            plugins.insert(name.clone(), plugin);
        }
        
        // Add to load order (sorted by priority)
        {
            let mut load_order = self.load_order.write().await;
            load_order.push(name.clone());
            
            // Sort by priority (higher priority first)
            let _plugins = self.plugins.read().await;
            load_order.sort_by(|_a, _b| {
                // For now, assume default priority since config might not be set yet
                std::cmp::Ordering::Equal
            });
        }
        
        info!("✅ Plugin '{}' registered successfully", name);
        Ok(())
    }
    
    /// Initialize all plugins
    pub async fn initialize_all(&mut self) -> Result<()> {
        info!("🚀 Initializing all plugins...");
        
        let load_order = self.load_order.read().await.clone();
        
        for plugin_name in load_order {
            self.initialize_plugin(&plugin_name).await?;
        }
        
        info!("✅ All plugins initialized");
        Ok(())
    }
    
    /// Initialize specific plugin
    pub async fn initialize_plugin(&mut self, name: &str) -> Result<()> {
        debug!("🔧 Initializing plugin: {}", name);
        
        let mut plugins = self.plugins.write().await;
        if let Some(plugin) = plugins.get_mut(name) {
            plugin.initialize(&self.context).await?;
            info!("✅ Plugin '{}' initialized", name);
        } else {
            return Err(MiniRosError::Custom(format!("Plugin '{}' not found", name)));
        }
        
        Ok(())
    }
    
    /// Start all plugins
    pub async fn start_all(&self) -> Result<()> {
        info!("▶️ Starting all plugins...");
        
        let load_order = self.load_order.read().await.clone();
        
        for plugin_name in load_order {
            self.start_plugin(&plugin_name).await?;
        }
        
        info!("✅ All plugins started");
        Ok(())
    }
    
    /// Start specific plugin
    pub async fn start_plugin(&self, name: &str) -> Result<()> {
        debug!("▶️ Starting plugin: {}", name);
        
        let mut plugins = self.plugins.write().await;
        if let Some(plugin) = plugins.get_mut(name) {
            plugin.start().await?;
            info!("✅ Plugin '{}' started", name);
        } else {
            return Err(MiniRosError::Custom(format!("Plugin '{}' not found", name)));
        }
        
        Ok(())
    }
    
    /// Stop all plugins
    pub async fn stop_all(&self) -> Result<()> {
        info!("⏹️ Stopping all plugins...");
        
        // Stop in reverse order
        let load_order = self.load_order.read().await.clone();
        
        for plugin_name in load_order.iter().rev() {
            if let Err(e) = self.stop_plugin(plugin_name).await {
                warn!("Failed to stop plugin '{}': {}", plugin_name, e);
            }
        }
        
        info!("✅ All plugins stopped");
        Ok(())
    }
    
    /// Stop specific plugin
    pub async fn stop_plugin(&self, name: &str) -> Result<()> {
        debug!("⏹️ Stopping plugin: {}", name);
        
        let mut plugins = self.plugins.write().await;
        if let Some(plugin) = plugins.get_mut(name) {
            plugin.stop().await?;
            info!("✅ Plugin '{}' stopped", name);
        } else {
            return Err(MiniRosError::Custom(format!("Plugin '{}' not found", name)));
        }
        
        Ok(())
    }
    
    /// Get plugin by name (returns plugin info instead of reference due to lifetime issues)
    pub async fn get_plugin_info(&self, name: &str) -> Option<(String, String, String)> {
        let plugins = self.plugins.read().await;
        plugins.get(name).map(|p| (p.name().to_string(), p.version().to_string(), p.description().to_string()))
    }
    
    /// Get plugin status
    pub async fn get_plugin_status(&self, name: &str) -> Option<PluginStatus> {
        let plugins = self.plugins.read().await;
        plugins.get(name).map(|p| p.status())
    }
    
    /// List all plugins
    pub async fn list_plugins(&self) -> Vec<PluginInfo> {
        let plugins = self.plugins.read().await;
        let mut plugin_list = Vec::new();
        
        for (name, plugin) in plugins.iter() {
            plugin_list.push(PluginInfo {
                name: name.clone(),
                version: plugin.version().to_string(),
                description: plugin.description().to_string(),
                status: plugin.status(),
            });
        }
        
        plugin_list
    }
    
    /// Configure plugin
    pub async fn configure_plugin(&self, name: &str, config: PluginConfig) -> Result<()> {
        let mut plugins = self.plugins.write().await;
        if let Some(plugin) = plugins.get_mut(name) {
            plugin.configure(config).await?;
            info!("🔧 Plugin '{}' configured", name);
        } else {
            return Err(MiniRosError::Custom(format!("Plugin '{}' not found", name)));
        }
        
        Ok(())
    }
}

impl Default for PluginManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Plugin information for listing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginInfo {
    pub name: String,
    pub version: String,
    pub description: String,
    pub status: PluginStatus,
}

/// Example transport plugin
pub struct CustomTransportPlugin {
    name: String,
    version: String,
    status: PluginStatus,
    config: Option<PluginConfig>,
}

impl CustomTransportPlugin {
    pub fn new() -> Self {
        Self {
            name: "custom_transport".to_string(),
            version: "1.0.0".to_string(),
            status: PluginStatus::Uninitialized,
            config: None,
        }
    }
}

#[async_trait]
impl Plugin for CustomTransportPlugin {
    fn name(&self) -> &str {
        &self.name
    }
    
    fn version(&self) -> &str {
        &self.version
    }
    
    fn description(&self) -> &str {
        "Example custom transport plugin for miniROS"
    }
    
    async fn initialize(&mut self, _context: &PluginContext) -> Result<()> {
        self.status = PluginStatus::Initializing;
        info!("🚀 Initializing custom transport plugin");
        
        // Initialize transport-specific resources
        // TODO: Setup custom transport layer
        
        self.status = PluginStatus::Ready;
        Ok(())
    }
    
    async fn start(&mut self) -> Result<()> {
        self.status = PluginStatus::Running;
        info!("▶️ Starting custom transport plugin");
        
        // Start transport services
        // TODO: Start listening for connections
        
        Ok(())
    }
    
    async fn stop(&mut self) -> Result<()> {
        self.status = PluginStatus::Stopping;
        info!("⏹️ Stopping custom transport plugin");
        
        // Cleanup transport resources
        // TODO: Close connections and cleanup
        
        self.status = PluginStatus::Stopped;
        Ok(())
    }
    
    fn config_schema(&self) -> PluginConfigSchema {
        PluginConfigSchema {
            fields: vec![
                ConfigField {
                    name: "port".to_string(),
                    field_type: ConfigFieldType::Integer,
                    description: "Transport port number".to_string(),
                    default: Some(serde_json::Value::Number(serde_json::Number::from(8080))),
                },
                ConfigField {
                    name: "protocol".to_string(),
                    field_type: ConfigFieldType::String,
                    description: "Transport protocol (tcp/udp)".to_string(),
                    default: Some(serde_json::Value::String("tcp".to_string())),
                },
            ],
            required: vec!["port".to_string()],
        }
    }
    
    async fn configure(&mut self, config: PluginConfig) -> Result<()> {
        info!("🔧 Configuring custom transport plugin");
        self.config = Some(config);
        Ok(())
    }
    
    fn status(&self) -> PluginStatus {
        self.status.clone()
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// Example monitoring plugin
pub struct MonitoringPlugin {
    name: String,
    version: String,
    status: PluginStatus,
    metrics: HashMap<String, f64>,
}

impl MonitoringPlugin {
    pub fn new() -> Self {
        Self {
            name: "monitoring".to_string(),
            version: "1.0.0".to_string(),
            status: PluginStatus::Uninitialized,
            metrics: HashMap::new(),
        }
    }
    
    /// Record a metric
    pub fn record_metric(&mut self, name: String, value: f64) {
        self.metrics.insert(name, value);
    }
    
    /// Get metrics
    pub fn get_metrics(&self) -> &HashMap<String, f64> {
        &self.metrics
    }
}

#[async_trait]
impl Plugin for MonitoringPlugin {
    fn name(&self) -> &str {
        &self.name
    }
    
    fn version(&self) -> &str {
        &self.version
    }
    
    fn description(&self) -> &str {
        "System monitoring and metrics collection plugin"
    }
    
    async fn initialize(&mut self, _context: &PluginContext) -> Result<()> {
        self.status = PluginStatus::Initializing;
        info!("📊 Initializing monitoring plugin");
        
        // Initialize metrics collection
        self.metrics.insert("startup_time".to_string(), 
                           std::time::SystemTime::now()
                               .duration_since(std::time::UNIX_EPOCH)
                               .unwrap()
                               .as_secs_f64());
        
        self.status = PluginStatus::Ready;
        Ok(())
    }
    
    async fn start(&mut self) -> Result<()> {
        self.status = PluginStatus::Running;
        info!("▶️ Starting monitoring plugin");
        
        // Start metrics collection
        self.record_metric("messages_processed".to_string(), 0.0);
        self.record_metric("memory_usage_mb".to_string(), 0.0);
        
        Ok(())
    }
    
    async fn stop(&mut self) -> Result<()> {
        self.status = PluginStatus::Stopping;
        info!("⏹️ Stopping monitoring plugin");
        
        // Final metrics snapshot
        info!("📊 Final metrics: {:?}", self.metrics);
        
        self.status = PluginStatus::Stopped;
        Ok(())
    }
    
    fn config_schema(&self) -> PluginConfigSchema {
        PluginConfigSchema {
            fields: vec![
                ConfigField {
                    name: "collection_interval_ms".to_string(),
                    field_type: ConfigFieldType::Integer,
                    description: "Metrics collection interval in milliseconds".to_string(),
                    default: Some(serde_json::Value::Number(serde_json::Number::from(1000))),
                },
                ConfigField {
                    name: "export_format".to_string(),
                    field_type: ConfigFieldType::String,
                    description: "Metrics export format (json/prometheus)".to_string(),
                    default: Some(serde_json::Value::String("json".to_string())),
                },
            ],
            required: vec![],
        }
    }
    
    async fn configure(&mut self, _config: PluginConfig) -> Result<()> {
        info!("🔧 Configuring monitoring plugin");
        Ok(())
    }
    
    fn status(&self) -> PluginStatus {
        self.status.clone()
    }
    
    fn as_any(&self) -> &dyn Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_plugin_manager() {
        let mut manager = PluginManager::new();
        
        // Register plugins
        let transport_plugin = Box::new(CustomTransportPlugin::new());
        let monitoring_plugin = Box::new(MonitoringPlugin::new());
        
        manager.register_plugin(transport_plugin).await.unwrap();
        manager.register_plugin(monitoring_plugin).await.unwrap();
        
        // Initialize and start
        manager.initialize_all().await.unwrap();
        manager.start_all().await.unwrap();
        
        // Check status
        let plugins = manager.list_plugins().await;
        assert_eq!(plugins.len(), 2);
        
        // Stop all
        manager.stop_all().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_plugin_configuration() {
        let mut plugin = CustomTransportPlugin::new();
        
        let mut config = PluginConfig::default();
        config.settings.insert("port".to_string(), serde_json::Value::Number(serde_json::Number::from(9090)));
        
        plugin.configure(config).await.unwrap();
        assert!(plugin.config.is_some());
    }
} 