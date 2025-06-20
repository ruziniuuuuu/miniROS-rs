//! # mini_ros CLI Tool
//!
//! Command-line interface for miniROS package and launch management.
//! Provides ROS2-like commands for launching nodes and managing packages.

use mini_ros::prelude::*;
use clap::{Arg, Command, ArgMatches};
use std::process;
use tracing::{info, error};

#[tokio::main]
async fn main() {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    let matches = Command::new("mini_ros")
        .version("0.1.0")
        .author("miniROS Team")
        .about("miniROS command-line interface")
        .subcommand(
            Command::new("launch")
                .about("Launch nodes from launch files")
                .arg(
                    Arg::new("package")
                        .help("Package name")
                        .required(true)
                        .index(1)
                )
                .arg(
                    Arg::new("launch_file")
                        .help("Launch file name")
                        .required(true)
                        .index(2)
                )
                .arg(
                    Arg::new("args")
                        .help("Additional arguments")
                        .num_args(0..)
                        .last(true)
                )
        )
        .subcommand(
            Command::new("pkg")
                .about("Package management commands")
                .subcommand(
                    Command::new("list")
                        .about("List available packages")
                )
                .subcommand(
                    Command::new("create")
                        .about("Create a new package")
                        .arg(
                            Arg::new("name")
                                .help("Package name")
                                .required(true)
                                .index(1)
                        )
                        .arg(
                            Arg::new("path")
                                .help("Package path (optional)")
                                .long("path")
                                .value_name("PATH")
                        )
                )
                .subcommand(
                    Command::new("info")
                        .about("Show package information")
                        .arg(
                            Arg::new("name")
                                .help("Package name")
                                .required(true)
                                .index(1)
                        )
                )
        )
        .subcommand(
            Command::new("run")
                .about("Run a single node")
                .arg(
                    Arg::new("package")
                        .help("Package name")
                        .required(true)
                        .index(1)
                )
                .arg(
                    Arg::new("executable")
                        .help("Executable name")
                        .required(true)
                        .index(2)
                )
                .arg(
                    Arg::new("args")
                        .help("Additional arguments")
                        .num_args(0..)
                        .last(true)
                )
        )
        .get_matches();

    let result = match matches.subcommand() {
        Some(("launch", sub_matches)) => handle_launch_command(sub_matches).await,
        Some(("pkg", sub_matches)) => handle_pkg_command(sub_matches).await,
        Some(("run", sub_matches)) => handle_run_command(sub_matches).await,
        _ => {
            println!("Use --help for usage information");
            Ok(())
        }
    };

    if let Err(e) = result {
        error!("Command failed: {}", e);
        process::exit(1);
    }
}

/// Handle launch command
async fn handle_launch_command(matches: &ArgMatches) -> Result<()> {
    let package_name = matches.get_one::<String>("package").unwrap();
    let launch_file = matches.get_one::<String>("launch_file").unwrap();
    
    info!("🚀 Launching: {} {}", package_name, launch_file);

    // Initialize package manager
    let mut pkg_manager = PackageManager::new();
    
    // Install built-in packages first
    pkg_manager.install_builtin_packages()?;
    
    // Discover additional packages
    pkg_manager.discover_packages()?;

    // Get launch file path
    let launch_path = pkg_manager
        .get_launch_file(package_name, launch_file)
        .ok_or_else(|| MiniRosError::ConfigError(format!(
            "Launch file '{}' not found in package '{}'", 
            launch_file, package_name
        )))?;

    info!("📄 Loading launch file: {}", launch_path.display());

    // Load launch configuration
    let launch_config = LaunchConfig::from_yaml(launch_path.to_str().unwrap())?;

    // Create launch manager and start
    let mut launch_manager = LaunchManager::new();
    
    // Handle Ctrl+C gracefully
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let running_clone = running.clone();
    
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        info!("📡 Received Ctrl+C, stopping launch...");
        running_clone.store(false, std::sync::atomic::Ordering::Relaxed);
    });

    // Launch all nodes
    launch_manager.launch(launch_config).await?;

    // Monitor nodes
    tokio::select! {
        result = launch_manager.monitor() => {
            if let Err(e) = result {
                error!("Monitor failed: {}", e);
            }
        }
        _ = tokio::time::sleep(std::time::Duration::from_millis(100)) => {
            while running.load(std::sync::atomic::Ordering::Relaxed) {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            }
        }
    }

    // Stop all nodes
    launch_manager.stop_all().await?;
    
    info!("🏁 Launch completed");
    Ok(())
}

/// Handle package management commands
async fn handle_pkg_command(matches: &ArgMatches) -> Result<()> {
    let mut pkg_manager = PackageManager::new();
    pkg_manager.install_builtin_packages()?;
    pkg_manager.discover_packages()?;

    match matches.subcommand() {
        Some(("list", _)) => {
            info!("📦 Available packages:");
            let packages = pkg_manager.list_packages();
            
            if packages.is_empty() {
                println!("No packages found.");
            } else {
                for pkg in packages {
                    println!("  {} ({})", pkg.manifest.name, pkg.manifest.version);
                    if let Some(ref desc) = pkg.manifest.description {
                        println!("    {}", desc);
                    }
                    println!("    Path: {}", pkg.root_path.display());
                    println!("    Executables: {}", pkg.manifest.executables.len());
                    println!("    Launch files: {}", pkg.manifest.launch_files.len());
                    println!();
                }
            }
        }
        
        Some(("create", sub_matches)) => {
            let name = sub_matches.get_one::<String>("name").unwrap();
            let path = sub_matches.get_one::<String>("path").map(std::path::PathBuf::from);
            
            info!("🔨 Creating package: {}", name);
            
            match pkg_manager.create_package(name, path) {
                Ok(pkg) => {
                    info!("✅ Package created successfully!");
                    println!("Package: {}", pkg.manifest.name);
                    println!("Path: {}", pkg.root_path.display());
                }
                Err(e) => {
                    error!("Failed to create package: {}", e);
                    return Err(e);
                }
            }
        }
        
        Some(("info", sub_matches)) => {
            let name = sub_matches.get_one::<String>("name").unwrap();
            
            if let Some(pkg) = pkg_manager.get_package(name) {
                println!("📋 Package Information:");
                println!("  Name: {}", pkg.manifest.name);
                println!("  Version: {}", pkg.manifest.version);
                println!("  Path: {}", pkg.root_path.display());
                
                if let Some(ref desc) = pkg.manifest.description {
                    println!("  Description: {}", desc);
                }
                
                if let Some(ref author) = pkg.manifest.author {
                    println!("  Author: {}", author);
                }
                
                if let Some(ref license) = pkg.manifest.license {
                    println!("  License: {}", license);
                }

                println!("  Python Support: {}", pkg.manifest.python_support);
                
                if !pkg.manifest.dependencies.is_empty() {
                    println!("  Dependencies:");
                    for dep in &pkg.manifest.dependencies {
                        println!("    - {}", dep);
                    }
                }
                
                if !pkg.manifest.executables.is_empty() {
                    println!("  Executables:");
                    for (name, info) in &pkg.manifest.executables {
                        println!("    - {} ({})", name, info.path);
                        if let Some(ref desc) = info.description {
                            println!("      {}", desc);
                        }
                        if info.python {
                            println!("      Language: Python");
                        } else {
                            println!("      Language: Rust");
                        }
                        if !info.features.is_empty() {
                            println!("      Features: {}", info.features.join(", "));
                        }
                    }
                }
                
                if !pkg.manifest.launch_files.is_empty() {
                    println!("  Launch Files:");
                    for (name, path) in &pkg.manifest.launch_files {
                        println!("    - {} ({})", name, path);
                    }
                }
            } else {
                error!("Package '{}' not found", name);
                return Err(MiniRosError::ConfigError(format!("Package '{}' not found", name)));
            }
        }
        
        _ => {
            println!("Use 'mini_ros pkg --help' for usage information");
        }
    }

    Ok(())
}

/// Handle run command
async fn handle_run_command(matches: &ArgMatches) -> Result<()> {
    let package_name = matches.get_one::<String>("package").unwrap();
    let executable_name = matches.get_one::<String>("executable").unwrap();
    
    info!("🏃 Running: {} {}", package_name, executable_name);

    // Initialize package manager
    let mut pkg_manager = PackageManager::new();
    pkg_manager.install_builtin_packages()?;
    pkg_manager.discover_packages()?;

    // Create a simple launch config for single executable
    let mut launch_config = LaunchConfig::new(&format!("{} {}", package_name, executable_name));
    
    // Find the package and executable
    let pkg = pkg_manager.get_package(package_name)
        .ok_or_else(|| MiniRosError::ConfigError(format!("Package '{}' not found", package_name)))?;
    
    let exec_info = pkg.manifest.executables.get(executable_name)
        .ok_or_else(|| MiniRosError::ConfigError(format!("Executable '{}' not found in package '{}'", executable_name, package_name)))?;

    // Create node launch config
    let node_config = NodeLaunchConfig {
        name: format!("{}_{}", package_name, executable_name),
        package: Some(package_name.to_string()),
        executable: if exec_info.python {
            // For Python executables, run the script directly
            pkg.root_path.join(&exec_info.path).to_str().unwrap().to_string()
        } else {
            // For Rust executables, use cargo run
            executable_name.to_string()
        },
        args: Vec::new(),
        env: std::collections::HashMap::new(),
        cwd: Some(pkg.root_path.to_str().unwrap().to_string()),
        respawn: false,
        delay: 0.0,
        python: exec_info.python,
    };

    launch_config.add_node(node_config);

    // Create launch manager and start
    let mut launch_manager = LaunchManager::new();
    
    // Handle Ctrl+C gracefully
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let running_clone = running.clone();
    
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        info!("📡 Received Ctrl+C, stopping...");
        running_clone.store(false, std::sync::atomic::Ordering::Relaxed);
    });

    // Launch the node
    launch_manager.launch(launch_config).await?;

    // Wait for completion or interruption
    while running.load(std::sync::atomic::Ordering::Relaxed) {
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    }

    // Stop the node
    launch_manager.stop_all().await?;
    
    info!("🏁 Run completed");
    Ok(())
} 