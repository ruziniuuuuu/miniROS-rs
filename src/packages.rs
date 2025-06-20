//! # Package System
//!
//! Provides package management functionality for organizing miniROS code.
//! Enables creating reusable packages with executables, libraries, and launch files.

use crate::{MiniRosError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use tracing::info;

/// Package metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageManifest {
    /// Package name
    pub name: String,
    /// Package version
    pub version: String,
    /// Package description
    pub description: Option<String>,
    /// Package author
    pub author: Option<String>,
    /// License
    pub license: Option<String>,
    /// Dependencies on other packages
    pub dependencies: Vec<String>,
    /// Executables provided by this package
    pub executables: HashMap<String, ExecutableInfo>,
    /// Launch files provided by this package
    pub launch_files: HashMap<String, String>,
    /// Whether this package provides Python nodes
    pub python_support: bool,
}

/// Information about an executable
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutableInfo {
    /// Executable name
    pub name: String,
    /// Path to executable (relative to package root)
    pub path: String,
    /// Whether it's a Python script
    pub python: bool,
    /// Description
    pub description: Option<String>,
    /// Required features
    pub features: Vec<String>,
}

impl Default for PackageManifest {
    fn default() -> Self {
        Self {
            name: String::new(),
            version: "0.1.0".to_string(),
            description: None,
            author: None,
            license: Some("MIT OR Apache-2.0".to_string()),
            dependencies: Vec::new(),
            executables: HashMap::new(),
            launch_files: HashMap::new(),
            python_support: false,
        }
    }
}

/// A miniROS package
#[derive(Debug, Clone)]
pub struct Package {
    /// Package manifest
    pub manifest: PackageManifest,
    /// Package root directory
    pub root_path: PathBuf,
}

impl Package {
    /// Create new package
    pub fn new(name: &str, root_path: PathBuf) -> Self {
        let manifest = PackageManifest {
            name: name.to_string(),
            ..PackageManifest::default()
        };

        Self {
            manifest,
            root_path,
        }
    }

    /// Load package from directory
    pub fn load_from_dir<P: AsRef<Path>>(path: P) -> Result<Self> {
        let root_path = path.as_ref().to_path_buf();
        let manifest_path = root_path.join("package.yaml");

        if !manifest_path.exists() {
            return Err(MiniRosError::ConfigError(format!(
                "Package manifest not found at: {}",
                manifest_path.display()
            )));
        }

        let content = std::fs::read_to_string(&manifest_path).map_err(|e| {
            MiniRosError::ConfigError(format!("Failed to read package manifest: {}", e))
        })?;

        let manifest: PackageManifest = serde_yaml::from_str(&content).map_err(|e| {
            MiniRosError::ConfigError(format!("Failed to parse package manifest: {}", e))
        })?;

        Ok(Self {
            manifest,
            root_path,
        })
    }

    /// Save package manifest
    pub fn save_manifest(&self) -> Result<()> {
        let manifest_path = self.root_path.join("package.yaml");
        let content = serde_yaml::to_string(&self.manifest).map_err(|e| {
            MiniRosError::ConfigError(format!("Failed to serialize manifest: {}", e))
        })?;

        std::fs::write(&manifest_path, content)
            .map_err(|e| MiniRosError::ConfigError(format!("Failed to write manifest: {}", e)))?;

        Ok(())
    }

    /// Add executable to package
    pub fn add_executable(&mut self, name: &str, path: &str, python: bool) -> &mut Self {
        let info = ExecutableInfo {
            name: name.to_string(),
            path: path.to_string(),
            python,
            description: None,
            features: Vec::new(),
        };
        self.manifest.executables.insert(name.to_string(), info);
        self
    }

    /// Add launch file to package
    pub fn add_launch_file(&mut self, name: &str, path: &str) -> &mut Self {
        self.manifest
            .launch_files
            .insert(name.to_string(), path.to_string());
        self
    }

    /// Get executable path
    pub fn get_executable_path(&self, name: &str) -> Option<PathBuf> {
        self.manifest
            .executables
            .get(name)
            .map(|info| self.root_path.join(&info.path))
    }

    /// Get launch file path
    pub fn get_launch_file_path(&self, name: &str) -> Option<PathBuf> {
        self.manifest
            .launch_files
            .get(name)
            .map(|path| self.root_path.join(path))
    }

    /// Initialize package directory structure
    pub fn init_structure(&self) -> Result<()> {
        // Create basic directory structure
        let dirs = ["src", "launch", "config", "scripts"];

        for dir in &dirs {
            let dir_path = self.root_path.join(dir);
            if !dir_path.exists() {
                std::fs::create_dir_all(&dir_path).map_err(|e| {
                    MiniRosError::ConfigError(format!("Failed to create directory {}: {}", dir, e))
                })?;
            }
        }

        // Create basic files
        self.create_basic_files()?;
        self.save_manifest()?;

        info!("✅ Package structure initialized: {}", self.manifest.name);
        Ok(())
    }

    /// Create basic package files
    fn create_basic_files(&self) -> Result<()> {
        // Create README.md
        let readme_path = self.root_path.join("README.md");
        if !readme_path.exists() {
            let readme_content = format!(
                "# {}\n\n{}\n\n## Usage\n\n```bash\n# Run examples\ncargo run --example <example_name>\n```\n",
                self.manifest.name,
                self.manifest
                    .description
                    .as_deref()
                    .unwrap_or("A miniROS package")
            );
            std::fs::write(&readme_path, readme_content).map_err(|e| {
                MiniRosError::ConfigError(format!("Failed to create README: {}", e))
            })?;
        }

        // Create Cargo.toml if it's a Rust package
        let cargo_path = self.root_path.join("Cargo.toml");
        if !cargo_path.exists() && !self.manifest.python_support {
            let cargo_content = format!(
                r#"[package]
name = "{}"
version = "{}"
edition = "2021"

[dependencies]
mini-ros = {{ path = "../.." }}
tokio = {{ version = "1.0", features = ["full"] }}
tracing = "0.1"
"#,
                self.manifest.name.replace("-", "_"),
                self.manifest.version
            );
            std::fs::write(&cargo_path, cargo_content).map_err(|e| {
                MiniRosError::ConfigError(format!("Failed to create Cargo.toml: {}", e))
            })?;
        }

        Ok(())
    }
}

/// Package manager for discovering and managing packages
pub struct PackageManager {
    /// Registry of discovered packages
    packages: HashMap<String, Package>,
    /// Search paths for packages
    search_paths: Vec<PathBuf>,
}

impl Default for PackageManager {
    fn default() -> Self {
        Self::new()
    }
}

impl PackageManager {
    /// Create new package manager
    pub fn new() -> Self {
        let mut search_paths = Vec::new();

        // Default search paths
        search_paths.push(PathBuf::from("packages"));
        search_paths.push(PathBuf::from("../packages"));

        // Add from environment variable
        if let Ok(paths) = std::env::var("MINI_ROS_PACKAGE_PATH") {
            for path in paths.split(':') {
                search_paths.push(PathBuf::from(path));
            }
        }

        Self {
            packages: HashMap::new(),
            search_paths,
        }
    }

    /// Add search path
    pub fn add_search_path<P: AsRef<Path>>(&mut self, path: P) -> &mut Self {
        self.search_paths.push(path.as_ref().to_path_buf());
        self
    }

    /// Discover packages in search paths
    pub fn discover_packages(&mut self) -> Result<()> {
        info!("🔍 Discovering packages...");

        for search_path in &self.search_paths {
            if !search_path.exists() {
                continue;
            }

            let entries = std::fs::read_dir(search_path).map_err(|e| {
                MiniRosError::ConfigError(format!(
                    "Failed to read directory {}: {}",
                    search_path.display(),
                    e
                ))
            })?;

            for entry in entries {
                let entry = entry.map_err(|e| {
                    MiniRosError::ConfigError(format!("Failed to read directory entry: {}", e))
                })?;
                let path = entry.path();

                if path.is_dir() {
                    match Package::load_from_dir(&path) {
                        Ok(package) => {
                            info!(
                                "📦 Found package: {} at {}",
                                package.manifest.name,
                                path.display()
                            );
                            self.packages.insert(package.manifest.name.clone(), package);
                        }
                        Err(_) => {
                            // Not a valid package directory, skip silently
                        }
                    }
                }
            }
        }

        info!("✅ Discovered {} packages", self.packages.len());
        Ok(())
    }

    /// Get package by name
    pub fn get_package(&self, name: &str) -> Option<&Package> {
        self.packages.get(name)
    }

    /// List all packages
    pub fn list_packages(&self) -> Vec<&Package> {
        self.packages.values().collect()
    }

    /// Create new package
    pub fn create_package(&mut self, name: &str, path: Option<PathBuf>) -> Result<&Package> {
        let package_path = path.unwrap_or_else(|| PathBuf::from("packages").join(name));

        // Create directory if it doesn't exist
        if !package_path.exists() {
            std::fs::create_dir_all(&package_path).map_err(|e| {
                MiniRosError::ConfigError(format!("Failed to create package directory: {}", e))
            })?;
        }

        let package = Package::new(name, package_path);
        package.init_structure()?;

        self.packages.insert(name.to_string(), package);
        Ok(self.packages.get(name).unwrap())
    }

    /// Get executable from package
    pub fn get_executable(&self, package_name: &str, executable_name: &str) -> Option<PathBuf> {
        self.packages
            .get(package_name)?
            .get_executable_path(executable_name)
    }

    /// Get launch file from package
    pub fn get_launch_file(&self, package_name: &str, launch_name: &str) -> Option<PathBuf> {
        self.packages
            .get(package_name)?
            .get_launch_file_path(launch_name)
    }

    /// Install built-in packages
    pub fn install_builtin_packages(&mut self) -> Result<()> {
        info!("📦 Installing built-in packages...");

        // Create turtlebot package
        let turtlebot_path = PathBuf::from("packages/turtlebot");
        let mut turtlebot = Package::new("turtlebot", turtlebot_path.clone());

        turtlebot.manifest.description =
            Some("Turtlebot simulation and control package".to_string());
        turtlebot.manifest.author = Some("miniROS Team".to_string());

        // Add executables
        turtlebot.add_executable("controller", "src/controller.rs", false);
        turtlebot.add_executable("teleop", "src/teleop.rs", false);
        turtlebot.add_executable("simulator", "src/simulator.rs", false);
        turtlebot.add_executable("py_controller", "scripts/controller.py", true);

        // Add launch files
        turtlebot.add_launch_file("simulation", "launch/simulation.yaml");
        turtlebot.add_launch_file("teleop", "launch/teleop.yaml");
        turtlebot.add_launch_file("full_system", "launch/full_system.yaml");

        // Create package structure
        if !turtlebot_path.exists() {
            turtlebot.init_structure()?;
        }

        self.packages.insert("turtlebot".to_string(), turtlebot);

        info!("✅ Built-in packages installed");
        Ok(())
    }
}
