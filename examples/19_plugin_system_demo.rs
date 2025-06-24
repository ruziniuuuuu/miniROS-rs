//! Plugin System Demo - Phase 3 Feature
//!
//! This example demonstrates the plugin system that provides extensible 
//! architecture for custom transports, message handlers, and other features
//! while maintaining the "mini" philosophy of simplicity.
//!
//! Features demonstrated:
//! - Plugin registration and management
//! - Custom transport plugin
//! - Monitoring and metrics plugin
//! - Plugin configuration and lifecycle
//! - Inter-plugin communication
//!
//! Run with: cargo run --example 19_plugin_system_demo

use mini_ros::prelude::*;
use mini_ros::{
    CustomTransportPlugin, MonitoringPlugin, Plugin, PluginConfig, PluginManager, PluginStatus,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::{info, warn};

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    info!("🔌 miniROS Plugin System Demo - Phase 3 Feature");
    info!("=" = 50);
    info!("Demonstrating extensible architecture with mini philosophy!");

    // Demo 1: Basic plugin registration and lifecycle
    demo_basic_plugin_lifecycle().await?;

    // Demo 2: Plugin configuration
    demo_plugin_configuration().await?;

    // Demo 3: Custom plugin development
    demo_custom_plugin_development().await?;

    // Demo 4: Plugin monitoring and management
    demo_plugin_monitoring().await?;

    info!("🎉 Plugin system demo completed successfully!");
    info!("💡 This demonstrates Phase 3 ecosystem features with mini philosophy:");
    info!("   • Extensible architecture: Easy plugin development");
    info!("   • Minimum complexity: Simple plugin interface");
    info!("   • Maximum performance: Efficient plugin management");
    info!("   • Production ready: Monitoring and lifecycle management");

    Ok(())
}

/// Demonstrate basic plugin lifecycle
async fn demo_basic_plugin_lifecycle() -> Result<()> {
    info!("🚀 === Demo 1: Basic Plugin Lifecycle ===");

    // Create plugin manager
    let mut manager = PluginManager::new();

    // Register plugins
    info!("📦 Registering plugins...");
    let transport_plugin = Box::new(CustomTransportPlugin::new());
    let monitoring_plugin = Box::new(MonitoringPlugin::new());

    manager.register_plugin(transport_plugin).await?;
    manager.register_plugin(monitoring_plugin).await?;

    // List registered plugins
    let plugins = manager.list_plugins().await;
    info!("📋 Registered plugins: {}", plugins.len());
    for plugin in &plugins {
        info!(
            "   • {}: v{} - {}",
            plugin.name, plugin.version, plugin.description
        );
    }

    // Initialize all plugins
    info!("🔧 Initializing plugins...");
    manager.initialize_all().await?;

    // Check plugin status after initialization
    for plugin in &plugins {
        if let Some(status) = manager.get_plugin_status(&plugin.name).await {
            info!("   {} status: {:?}", plugin.name, status);
        }
    }

    // Start all plugins
    info!("▶️ Starting plugins...");
    manager.start_all().await?;

    // Simulate some runtime
    sleep(Duration::from_millis(500)).await;

    // Stop all plugins
    info!("⏹️ Stopping plugins...");
    manager.stop_all().await?;

    // Final status check
    for plugin in &plugins {
        if let Some(status) = manager.get_plugin_status(&plugin.name).await {
            info!("   {} final status: {:?}", plugin.name, status);
        }
    }

    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demonstrate plugin configuration
async fn demo_plugin_configuration() -> Result<()> {
    info!("⚙️ === Demo 2: Plugin Configuration ===");

    let mut manager = PluginManager::new();

    // Register transport plugin
    let transport_plugin = Box::new(CustomTransportPlugin::new());
    manager.register_plugin(transport_plugin).await?;

    // Show basic plugin info  
    if let Some((name, version, description)) = manager.get_plugin_info("custom_transport").await {
        info!("🔧 Transport plugin info:");
        info!("   • Name: {}", name);
        info!("   • Version: {}", version);  
        info!("   • Description: {}", description);
        
        // Create a temporary plugin to show schema
        let temp_plugin = CustomTransportPlugin::new();
        let schema = temp_plugin.config_schema();
        info!("   • Configuration schema:");
        for field in &schema.fields {
            info!(
                "     - {}: {:?} - {}",
                field.name, field.field_type, field.description
            );
            if let Some(default) = &field.default {
                info!("       Default: {}", default);
            }
        }
        info!("   • Required fields: {:?}", schema.required);
    }

    // Configure the plugin
    let mut config = PluginConfig::default();
    config.priority = 200; // Higher priority
    config
        .settings
        .insert("port".to_string(), serde_json::Value::Number(9090.into()));
    config.settings.insert(
        "protocol".to_string(),
        serde_json::Value::String("udp".to_string()),
    );

    info!("🔧 Configuring transport plugin with custom settings...");
    manager
        .configure_plugin("custom_transport", config)
        .await?;

    // Initialize and start with custom configuration
    manager.initialize_all().await?;
    manager.start_all().await?;

    sleep(Duration::from_millis(300)).await;

    manager.stop_all().await?;

    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demonstrate custom plugin development
async fn demo_custom_plugin_development() -> Result<()> {
    info!("🛠️ === Demo 3: Custom Plugin Development ===");

    // Create a custom data processing plugin
    let data_processor = Box::new(DataProcessingPlugin::new());

    let mut manager = PluginManager::new();
    manager.register_plugin(data_processor).await?;

    // Also register monitoring for demonstration
    let monitoring = Box::new(MonitoringPlugin::new());
    manager.register_plugin(monitoring).await?;

    // Initialize and start
    manager.initialize_all().await?;
    manager.start_all().await?;

    // Simulate data processing
    info!("🔄 Simulating data processing with custom plugin...");

    // In a real scenario, you would get plugin references and call methods
    let plugins = manager.list_plugins().await;
    for plugin in &plugins {
        info!("   Active plugin: {} ({})", plugin.name, plugin.version);
    }

    // Simulate processing for a bit
    for i in 1..=5 {
        info!("   Processing batch {} of data...", i);
        sleep(Duration::from_millis(200)).await;
    }

    manager.stop_all().await?;

    sleep(Duration::from_secs(1)).await;
    Ok(())
}

/// Demonstrate plugin monitoring and management
async fn demo_plugin_monitoring() -> Result<()> {
    info!("📊 === Demo 4: Plugin Monitoring ===");

    let mut manager = PluginManager::new();

    // Register multiple plugins for monitoring
    let plugins_to_register = vec![
        ("transport", Box::new(CustomTransportPlugin::new()) as Box<dyn Plugin>),
        ("monitoring", Box::new(MonitoringPlugin::new()) as Box<dyn Plugin>),
        ("data_processing", Box::new(DataProcessingPlugin::new()) as Box<dyn Plugin>),
    ];

    for (name, plugin) in plugins_to_register {
        info!("📦 Registering {} plugin...", name);
        manager.register_plugin(plugin).await?;
    }

    // Initialize and start all
    manager.initialize_all().await?;
    manager.start_all().await?;

    // Monitor plugin status over time
    info!("🔍 Monitoring plugin health...");

    for cycle in 1..=5 {
        info!("   Monitoring cycle {}/5:", cycle);

        let plugins = manager.list_plugins().await;
        for plugin in plugins {
            match plugin.status {
                PluginStatus::Running => {
                    info!("     ✅ {} - Running normally", plugin.name);
                }
                PluginStatus::Error(ref err) => {
                    warn!("     ❌ {} - Error: {}", plugin.name, err);
                }
                _ => {
                    info!("     ⏸️ {} - Status: {:?}", plugin.name, plugin.status);
                }
            }
        }

        sleep(Duration::from_millis(400)).await;
    }

    // Demonstrate individual plugin control
    info!("🎛️ Demonstrating individual plugin control...");

    // Stop one plugin
    info!("⏹️ Stopping transport plugin individually...");
    manager.stop_plugin("custom_transport").await?;

    // Check status
    if let Some(status) = manager.get_plugin_status("custom_transport").await {
        info!("   Transport plugin status after stop: {:?}", status);
    }

    // Stop remaining plugins
    info!("⏹️ Stopping remaining plugins...");
    manager.stop_all().await?;

    // Performance summary
    info!("⚡ Plugin system performance characteristics:");
    info!("   • Fast registration: O(1) plugin lookup");
    info!("   • Minimal overhead: Trait-based abstraction");
    info!("   • Safe lifecycle: Automatic cleanup on errors");
    info!("   • Flexible configuration: JSON-based settings");

    Ok(())
}

/// Example custom data processing plugin
use async_trait::async_trait;
use mini_ros::{PluginConfigSchema, PluginContext};
use std::any::Any;

struct DataProcessingPlugin {
    name: String,
    version: String,
    status: PluginStatus,
    processed_items: u64,
}

impl DataProcessingPlugin {
    fn new() -> Self {
        Self {
            name: "data_processing".to_string(),
            version: "1.0.0".to_string(),
            status: PluginStatus::Uninitialized,
            processed_items: 0,
        }
    }

    fn process_data(&mut self, _data: &str) {
        self.processed_items += 1;
    }

    fn get_processed_count(&self) -> u64 {
        self.processed_items
    }
}

#[async_trait]
impl Plugin for DataProcessingPlugin {
    fn name(&self) -> &str {
        &self.name
    }

    fn version(&self) -> &str {
        &self.version
    }

    fn description(&self) -> &str {
        "High-performance data processing plugin"
    }

    async fn initialize(&mut self, _context: &PluginContext) -> Result<()> {
        self.status = PluginStatus::Initializing;
        info!("🚀 Initializing data processing plugin");

        // Initialize processing buffers, algorithms, etc.
        self.processed_items = 0;

        self.status = PluginStatus::Ready;
        Ok(())
    }

    async fn start(&mut self) -> Result<()> {
        self.status = PluginStatus::Running;
        info!("▶️ Starting data processing plugin");

        // Start processing threads, set up data pipelines, etc.
        Ok(())
    }

    async fn stop(&mut self) -> Result<()> {
        self.status = PluginStatus::Stopping;
        info!("⏹️ Stopping data processing plugin");

        info!("📊 Final processing stats: {} items processed", self.processed_items);

        self.status = PluginStatus::Stopped;
        Ok(())
    }

    fn config_schema(&self) -> PluginConfigSchema {
        use mini_ros::{ConfigField, ConfigFieldType};

        PluginConfigSchema {
            fields: vec![
                ConfigField {
                    name: "batch_size".to_string(),
                    field_type: ConfigFieldType::Integer,
                    description: "Processing batch size".to_string(),
                    default: Some(serde_json::Value::Number(100.into())),
                },
                ConfigField {
                    name: "algorithm".to_string(),
                    field_type: ConfigFieldType::String,
                    description: "Processing algorithm (fast/accurate)".to_string(),
                    default: Some(serde_json::Value::String("fast".to_string())),
                },
            ],
            required: vec![],
        }
    }

    async fn configure(&mut self, _config: PluginConfig) -> Result<()> {
        info!("🔧 Configuring data processing plugin");
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