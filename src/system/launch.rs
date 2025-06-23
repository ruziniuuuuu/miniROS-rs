//! # Launch System
//!
//! Provides orchestration capabilities for launching multiple nodes simultaneously.
//! Inspired by ROS2 launch but simplified for miniROS.

use crate::core::error::{MiniRosError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::process::{Child, Command, Stdio};
use std::time::Duration;
use tokio::time::sleep;
use tracing::{error, info, warn};

/// Configuration for launching a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeLaunchConfig {
    /// Node name
    pub name: String,
    /// Package name (optional, defaults to current package)
    pub package: Option<String>,
    /// Executable name or example name
    pub executable: String,
    /// Command line arguments
    pub args: Vec<String>,
    /// Environment variables
    pub env: HashMap<String, String>,
    /// Working directory
    pub cwd: Option<String>,
    /// Whether to respawn on failure
    pub respawn: bool,
    /// Delay before starting (seconds)
    pub delay: f64,
    /// Whether to run in Python mode
    pub python: bool,
}

impl Default for NodeLaunchConfig {
    fn default() -> Self {
        Self {
            name: String::new(),
            package: None,
            executable: String::new(),
            args: Vec::new(),
            env: HashMap::new(),
            cwd: None,
            respawn: false,
            delay: 0.0,
            python: false,
        }
    }
}

/// Launch configuration for multiple nodes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchConfig {
    /// Launch file name/description
    pub name: String,
    /// List of nodes to launch
    pub nodes: Vec<NodeLaunchConfig>,
    /// Global environment variables
    pub global_env: HashMap<String, String>,
    /// Launch timeout (seconds)
    pub timeout: Option<f64>,
}

impl LaunchConfig {
    /// Create new launch configuration
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            nodes: Vec::new(),
            global_env: HashMap::new(),
            timeout: None,
        }
    }

    /// Add a node to launch
    pub fn add_node(&mut self, node: NodeLaunchConfig) -> &mut Self {
        self.nodes.push(node);
        self
    }

    /// Add environment variable
    pub fn add_env(&mut self, key: &str, value: &str) -> &mut Self {
        self.global_env.insert(key.to_string(), value.to_string());
        self
    }

    /// Set launch timeout
    pub fn set_timeout(&mut self, timeout_sec: f64) -> &mut Self {
        self.timeout = Some(timeout_sec);
        self
    }

    /// Load from YAML file
    pub fn from_yaml(path: &str) -> Result<Self> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| MiniRosError::ConfigError(format!("Failed to read launch file: {}", e)))?;

        serde_yaml::from_str(&content)
            .map_err(|e| MiniRosError::ConfigError(format!("Failed to parse launch file: {}", e)))
    }

    /// Save to YAML file
    pub fn to_yaml(&self, path: &str) -> Result<()> {
        let content = serde_yaml::to_string(self).map_err(|e| {
            MiniRosError::ConfigError(format!("Failed to serialize launch config: {}", e))
        })?;

        std::fs::write(path, content).map_err(|e| {
            MiniRosError::ConfigError(format!("Failed to write launch file: {}", e))
        })?;

        Ok(())
    }
}

/// Launch description builder pattern
pub struct LaunchDescription {
    config: LaunchConfig,
}

impl LaunchDescription {
    /// Create new launch description
    pub fn new(name: &str) -> Self {
        Self {
            config: LaunchConfig::new(name),
        }
    }

    /// Add Rust node
    pub fn add_rust_node(mut self, name: &str, executable: &str) -> Self {
        let node = NodeLaunchConfig {
            name: name.to_string(),
            executable: executable.to_string(),
            python: false,
            ..Default::default()
        };
        self.config.add_node(node);
        self
    }

    /// Add Python node
    pub fn add_python_node(mut self, name: &str, script: &str) -> Self {
        let node = NodeLaunchConfig {
            name: name.to_string(),
            executable: script.to_string(),
            python: true,
            ..Default::default()
        };
        self.config.add_node(node);
        self
    }

    /// Add node with custom configuration
    pub fn add_custom_node(mut self, node: NodeLaunchConfig) -> Self {
        self.config.add_node(node);
        self
    }

    /// Set global environment
    pub fn set_env(mut self, key: &str, value: &str) -> Self {
        self.config.add_env(key, value);
        self
    }

    /// Build the launch configuration
    pub fn build(self) -> LaunchConfig {
        self.config
    }
}

/// Process handle for launched node
#[derive(Debug)]
pub struct LaunchedNode {
    pub name: String,
    pub process: Option<Child>,
    pub config: NodeLaunchConfig,
    pub start_time: std::time::Instant,
}

impl LaunchedNode {
    /// Check if process is still running
    pub fn is_running(&mut self) -> bool {
        if let Some(ref mut process) = self.process {
            match process.try_wait() {
                Ok(Some(_)) => false, // Process has exited
                Ok(None) => true,     // Process is still running
                Err(_) => false,      // Error checking status
            }
        } else {
            false
        }
    }

    /// Stop the process
    pub fn stop(&mut self) -> Result<()> {
        if let Some(ref mut process) = self.process {
            process.kill().map_err(|e| {
                MiniRosError::Custom(format!("Failed to kill process {}: {}", self.name, e))
            })?;
        }
        Ok(())
    }
}

/// Launch manager for orchestrating multiple nodes
pub struct LaunchManager {
    launched_nodes: Vec<LaunchedNode>,
}

impl Default for LaunchManager {
    fn default() -> Self {
        Self::new()
    }
}

impl LaunchManager {
    /// Create new launch manager
    pub fn new() -> Self {
        Self {
            launched_nodes: Vec::new(),
        }
    }

    /// Launch all nodes from configuration
    pub async fn launch(&mut self, config: LaunchConfig) -> Result<()> {
        info!("🚀 Starting launch: {}", config.name);

        for node_config in config.nodes {
            // Apply delay if specified
            if node_config.delay > 0.0 {
                info!(
                    "⏳ Waiting {:.1}s before launching {}",
                    node_config.delay, node_config.name
                );
                sleep(Duration::from_secs_f64(node_config.delay)).await;
            }

            match self
                .launch_node(node_config.clone(), &config.global_env)
                .await
            {
                Ok(()) => info!("✅ Launched node: {}", node_config.name),
                Err(e) => {
                    error!("❌ Failed to launch node {}: {}", node_config.name, e);
                    if !node_config.respawn {
                        return Err(e);
                    }
                }
            }
        }

        info!("🎉 All nodes launched successfully");
        Ok(())
    }

    /// Launch a single node
    async fn launch_node(
        &mut self,
        node_config: NodeLaunchConfig,
        global_env: &HashMap<String, String>,
    ) -> Result<()> {
        let mut cmd = if node_config.python {
            // Python node
            let mut python_cmd = Command::new("python3");
            python_cmd.arg(&node_config.executable);
            python_cmd
        } else {
            // Rust node (as example)
            let mut cargo_cmd = Command::new("cargo");
            cargo_cmd.args(["run", "--example", &node_config.executable]);

            // Add features if needed (for visualization examples)
            if node_config.executable.contains("visualization")
                || node_config.executable.contains("simulator")
            {
                cargo_cmd.args(["--features", "visualization"]);
            }

            cargo_cmd
        };

        // Set working directory
        if let Some(ref cwd) = node_config.cwd {
            cmd.current_dir(cwd);
        }

        // Set environment variables
        for (key, value) in global_env {
            cmd.env(key, value);
        }
        for (key, value) in &node_config.env {
            cmd.env(key, value);
        }

        // Add command line arguments
        cmd.args(&node_config.args);

        // Configure stdio
        cmd.stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .stdin(Stdio::null());

        // Spawn process
        let process = cmd.spawn().map_err(|e| {
            MiniRosError::Custom(format!(
                "Failed to spawn process for {}: {}",
                node_config.name, e
            ))
        })?;

        let launched_node = LaunchedNode {
            name: node_config.name.clone(),
            process: Some(process),
            config: node_config,
            start_time: std::time::Instant::now(),
        };

        self.launched_nodes.push(launched_node);
        Ok(())
    }

    /// Monitor running nodes and handle respawning
    pub async fn monitor(&mut self) -> Result<()> {
        loop {
            let mut restart_nodes = Vec::new();

            // Check all nodes
            for (i, node) in self.launched_nodes.iter_mut().enumerate() {
                if !node.is_running() {
                    warn!("💀 Node {} has stopped", node.name);

                    if node.config.respawn {
                        info!("🔄 Respawning node: {}", node.name);
                        restart_nodes.push(i);
                    }
                }
            }

            // Restart failed nodes
            for &index in restart_nodes.iter().rev() {
                let node_config = self.launched_nodes[index].config.clone();
                self.launched_nodes.remove(index);

                // Wait a bit before respawning
                sleep(Duration::from_secs(1)).await;

                if let Err(e) = self.launch_node(node_config.clone(), &HashMap::new()).await {
                    error!("❌ Failed to respawn {}: {}", node_config.name, e);
                }
            }

            // Check every second
            sleep(Duration::from_secs(1)).await;
        }
    }

    /// Stop all launched nodes
    pub async fn stop_all(&mut self) -> Result<()> {
        info!("🛑 Stopping all launched nodes...");

        for node in &mut self.launched_nodes {
            if let Err(e) = node.stop() {
                warn!("⚠️ Error stopping {}: {}", node.name, e);
            } else {
                info!("✅ Stopped node: {}", node.name);
            }
        }

        self.launched_nodes.clear();
        info!("🏁 All nodes stopped");
        Ok(())
    }

    /// Get status of all nodes
    pub fn get_status(&mut self) -> Vec<(String, bool, Duration)> {
        self.launched_nodes
            .iter_mut()
            .map(|node| {
                let uptime = node.start_time.elapsed();
                (node.name.clone(), node.is_running(), uptime)
            })
            .collect()
    }
}

impl Drop for LaunchManager {
    fn drop(&mut self) {
        // Attempt to stop all nodes when manager is dropped
        for node in &mut self.launched_nodes {
            let _ = node.stop();
        }
    }
}
