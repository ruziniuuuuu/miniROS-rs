//! Example 14: Turtlebot Simulator with Visualization
//!
//! This example demonstrates essential turtlebot simulation capabilities:
//! - Physics-based robot simulation with differential drive kinematics
//! - Subscribing to velocity commands for robot control
//! - Publishing odometry data for position feedback
//! - Optional real-time visualization with Rerun integration
//! - Simple, focused robotics simulation
//!
//! Following miniROS philosophy: Essential robotics functionality with minimal complexity
//!
//! Run with: cargo run --example 14_turtlebot_simulator
//! With visualization: cargo run --example 14_turtlebot_simulator --features visualization
//!
//! Use together with the teleop example for complete system:
//! ```bash
//! # Terminal 1: Start simulator
//! cargo run --example 14_turtlebot_simulator --features visualization
//!
//! # Terminal 2: Start teleop
//! cargo run --example 13_turtlebot_teleop
//! ```

use mini_ros::prelude::*;
use mini_ros::types::{geometry_msgs, nav_msgs, std_msgs};
use std::f32::consts::PI;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::time::{Instant, sleep};
use tracing::{info, warn};

#[cfg(feature = "visualization")]
use mini_ros::features::visualization::{VisualizationClient, VisualizationConfig};

/// Turtlebot simulator with physics and visualization
pub struct TurtlebotSimulator {
    node: Node,
    cmd_vel_sub: Subscriber<geometry_msgs::Twist>,
    odom_pub: Publisher<nav_msgs::Odometry>,

    // Robot state - minimal state representation
    x: f32,
    y: f32,
    theta: f32,
    linear_vel: f32,
    angular_vel: f32,

    // Simulation parameters
    dt: f32,
    
    // Visualization (optional)
    #[cfg(feature = "visualization")]
    viz_client: Option<VisualizationClient>,

    // Path tracking for visualization
    path_points: Vec<[f32; 2]>,
    last_update: Instant,
    
    // Velocity command state sharing
    velocity_commands: std::sync::Arc<std::sync::Mutex<(f32, f32)>>, // (linear, angular)
}

impl TurtlebotSimulator {
    /// Create new turtlebot simulator with minimal configuration
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        info!("🤖 Creating turtlebot simulator: {}", node_name);

        // Create subscribers and publishers using standard ROS2 message types
        let cmd_vel_sub = node.create_subscriber::<geometry_msgs::Twist>("/cmd_vel").await?;
        let odom_pub = node.create_publisher::<nav_msgs::Odometry>("/odom").await?;

        info!("📥 Created velocity subscriber on /cmd_vel");
        info!("📤 Created odometry publisher on /odom");

        // Initialize optional visualization
        #[cfg(feature = "visualization")]
        let viz_client = {
            match VisualizationClient::new(VisualizationConfig {
                application_id: "turtlebot_simulator".to_string(),
                spawn_viewer: true,
            }) {
                Ok(client) => {
                    info!("📊 Rerun visualization initialized");
                    Some(client)
                }
                Err(e) => {
                    warn!("⚠️  Failed to initialize visualization: {}", e);
                    None
                }
            }
        };

        Ok(Self {
            node,
            cmd_vel_sub,
            odom_pub,
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
            dt: 0.02, // 50 Hz simulation rate
            #[cfg(feature = "visualization")]
            viz_client,
            path_points: Vec::new(),
            last_update: Instant::now(),
            velocity_commands: std::sync::Arc::new(std::sync::Mutex::new((0.0, 0.0))),
        })
    }

    /// Start the main simulation loop
    pub async fn start_simulation(&mut self) -> Result<()> {
        info!("🚀 Starting turtlebot simulator");
        info!("📋 Simulation features:");
        info!("   • Differential drive kinematics");
        info!("   • Real-time odometry publishing");
        info!("   • Velocity command processing");
        
        #[cfg(feature = "visualization")]
        info!("   • Real-time visualization with Rerun");
        
        #[cfg(not(feature = "visualization"))]
        warn!("   • Visualization disabled (use --features visualization)");

        // Setup velocity command callback
        self.setup_velocity_callback().await?;

        #[cfg(feature = "visualization")]
        {
            if let Some(ref viz) = self.viz_client {
                self.setup_visualization_scene(viz)?;
            }
        }

        // Main simulation loop - core functionality
        loop {
            let now = Instant::now();
            let dt = now.duration_since(self.last_update).as_secs_f32();
            self.last_update = now;

            // Update velocity from received commands
            self.update_velocity_from_commands();

            // Update robot physics
            self.update_physics(dt);

            // Publish odometry
            self.publish_odometry().await?;

            // Update visualization if enabled
            #[cfg(feature = "visualization")]
            {
                if let Some(ref viz) = self.viz_client {
                    self.update_visualization(viz)?;
                }
            }

            // Print status periodically
            self.print_status();

            sleep(Duration::from_millis(20)).await; // 50 Hz simulation rate
        }
    }

    /// Setup velocity command callback for robot control
    async fn setup_velocity_callback(&mut self) -> Result<()> {
        info!("🔗 Setting up velocity command processing");

        let velocity_state = self.velocity_commands.clone();

        self.cmd_vel_sub.on_message(move |twist: geometry_msgs::Twist| {
            let mut state = velocity_state.lock().unwrap();
            state.0 = twist.linear.x as f32;
            state.1 = twist.angular.z as f32;
            
            info!(
                "📡 Received velocity command: linear={:.2} m/s, angular={:.2} rad/s",
                state.0, state.1
            );
        })?;

        Ok(())
    }

    /// Update robot velocity from received commands
    fn update_velocity_from_commands(&mut self) {
        let commands = self.velocity_commands.lock().unwrap();
        self.linear_vel = commands.0;
        self.angular_vel = commands.1;
    }

    /// Update robot physics using differential drive kinematics
    fn update_physics(&mut self, dt: f32) {
        // Simple differential drive kinematics - core robotics functionality
        let dx = self.linear_vel * self.theta.cos() * dt;
        let dy = self.linear_vel * self.theta.sin() * dt;
        let dtheta = self.angular_vel * dt;

        // Update robot state
        self.x += dx;
        self.y += dy;
        self.theta += dtheta;

        // Normalize angle to [-π, π]
        self.theta = self.normalize_angle(self.theta);

        // Track path for visualization (minimal overhead)
        if self.path_points.is_empty()
            || (self.x - self.path_points.last().unwrap()[0]).abs() > 0.05
            || (self.y - self.path_points.last().unwrap()[1]).abs() > 0.05
        {
            self.path_points.push([self.x, self.y]);

            // Limit path length to prevent memory growth
            if self.path_points.len() > 1000 {
                self.path_points.remove(0);
            }
        }
    }

    /// Publish odometry message with current robot state
    async fn publish_odometry(&self) -> Result<()> {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;

        // Convert theta to quaternion (z-axis rotation)
        let half_theta = self.theta / 2.0;
        let quaternion = geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: half_theta.sin() as f64,
            w: half_theta.cos() as f64,
        };

        // Create properly structured odometry message
        let odom = nav_msgs::Odometry {
            header: std_msgs::Header {
                stamp: timestamp,
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            pose: geometry_msgs::PoseWithCovariance {
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: self.x as f64,
                        y: self.y as f64,
                        z: 0.0,
                    },
                    orientation: quaternion,
                },
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
            twist: geometry_msgs::TwistWithCovariance {
                twist: geometry_msgs::Twist {
                    linear: geometry_msgs::Vector3 {
                        x: self.linear_vel as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: geometry_msgs::Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: self.angular_vel as f64,
                    },
                },
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
        };

        self.odom_pub.publish(&odom).await?;
        Ok(())
    }

    /// Setup visualization scene (optional feature)
    #[cfg(feature = "visualization")]
    fn setup_visualization_scene(&self, viz: &VisualizationClient) -> Result<()> {
        info!("📊 Setting up visualization scene");

        // Log coordinate system
        viz.log_transform("world", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])?;

        // Log ground plane
        let ground_points = vec![
            [-5.0, -5.0, 0.0],
            [5.0, -5.0, 0.0],
            [5.0, 5.0, 0.0],
            [-5.0, 5.0, 0.0],
        ];
        viz.log_points("world/ground", ground_points)?;

        info!("✅ Visualization scene setup complete");
        Ok(())
    }

    /// Update visualization with current robot state
    #[cfg(feature = "visualization")]
    fn update_visualization(&self, viz: &VisualizationClient) -> Result<()> {
        // Set current simulation time
        let timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;
        let _ = viz.set_time("sim_time", timestamp_ns);

        // Log robot position and orientation
        let robot_pos = [self.x, self.y, 0.1];
        let half_theta = self.theta / 2.0;
        let robot_rot = [0.0, 0.0, half_theta.sin(), half_theta.cos()];
        viz.log_transform("world/robot", robot_pos, robot_rot)?;

        // Log robot path
        if self.path_points.len() > 1 {
            let path_3d: Vec<[f32; 3]> = self
                .path_points
                .iter()
                .map(|p| [p[0], p[1], 0.02])
                .collect();
            viz.log_points("world/robot_path", path_3d)?;
        }

        // Log velocity vectors for visual feedback
        if self.linear_vel.abs() > 0.01 || self.angular_vel.abs() > 0.01 {
            let vel_end_x = self.x + self.linear_vel * self.theta.cos() * 0.5;
            let vel_end_y = self.y + self.linear_vel * self.theta.sin() * 0.5;

            viz.log_points(
                "world/velocity",
                vec![[self.x, self.y, 0.15], [vel_end_x, vel_end_y, 0.15]],
            )?;
        }

        // Log telemetry data
        viz.log_scalar("robot/position_x", self.x as f64)?;
        viz.log_scalar("robot/position_y", self.y as f64)?;
        viz.log_scalar("robot/theta_degrees", self.theta.to_degrees() as f64)?;
        viz.log_scalar("robot/linear_velocity", self.linear_vel as f64)?;
        viz.log_scalar("robot/angular_velocity", self.angular_vel as f64)?;

        Ok(())
    }

    /// Print simulation status periodically
    fn print_status(&self) {
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            if COUNTER % 25 == 0 {
                // Print every 0.5 seconds (25 * 20ms)
                info!(
                    "🤖 Robot state: pos=({:.2}, {:.2}), θ={:.1}°, vel=({:.2}, {:.2})",
                    self.x,
                    self.y,
                    self.theta.to_degrees(),
                    self.linear_vel,
                    self.angular_vel
                );
            }
        }
    }

    /// Normalize angle to [-π, π] range
    fn normalize_angle(&self, angle: f32) -> f32 {
        let mut normalized = angle;
        while normalized > PI {
            normalized -= 2.0 * PI;
        }
        while normalized < -PI {
            normalized += 2.0 * PI;
        }
        normalized
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging for better debugging
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 14: Turtlebot Simulator ===");
    info!("🎯 Demonstrating essential robotics simulation");

    #[cfg(not(feature = "visualization"))]
    {
        warn!("⚠️  Visualization not enabled");
        warn!("   Run with: cargo run --example 14_turtlebot_simulator --features visualization");
    }

    #[cfg(feature = "visualization")]
    info!("📊 Visualization enabled - Rerun viewer will open automatically");

    // Create simulator with minimal setup
    let mut simulator = TurtlebotSimulator::new("turtlebot_simulator").await?;

    info!("🚀 Starting simulation loop...");
    info!("💡 Send velocity commands to /cmd_vel to control the robot");
    info!("📊 Odometry data published on /odom");

    // Start simulation (runs indefinitely)
    simulator.start_simulation().await?;

    Ok(())
}
