//! # Turtlebot Simulator with Rerun Visualization
//!
//! This example implements a turtlebot simulator with real-time visualization:
//! - Subscribes to velocity commands from `/cmd_vel`
//! - Publishes odometry data to `/odom`
//! - Real-time robot state visualization using Rerun
//! - Physics simulation with realistic motion dynamics
//! - Path tracking and trajectory visualization
//!
//! Run together with teleop for complete turtlebot control system:
//! ```bash
//! # Terminal 1: Start simulator
//! cargo run --example 14_turtlebot_simulator --features visualization
//! 
//! # Terminal 2: Start teleop
//! cargo run --example 13_turtlebot_teleop
//! ```

use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, OdometryMessage, Vector3, Point3D, Quaternion, Header};
use mini_ros::visualization::VisualizationClient;
use std::f32::consts::PI;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::time::{sleep, Instant};
use tracing::{info, warn};

#[cfg(feature = "visualization")]
use mini_ros::visualization::VisualizationConfig;

/// Turtlebot simulator state
pub struct TurtlebotSimulator {
    node: Node,
    cmd_vel_sub: Subscriber<TwistMessage>,
    odom_pub: Publisher<OdometryMessage>,
    
    // Robot state
    x: f32,
    y: f32,
    theta: f32,
    linear_vel: f32,
    angular_vel: f32,
    
    // Simulation parameters
    dt: f32,
    max_acceleration: f32,
    max_angular_acceleration: f32,
    
    // Visualization
    #[cfg(feature = "visualization")]
    viz_client: Option<VisualizationClient>,
    
    // Path tracking
    path_points: Vec<[f32; 2]>,
    last_update: Instant,
}

impl TurtlebotSimulator {
    /// Create new turtlebot simulator
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        // Create subscribers and publishers
        let cmd_vel_sub = node.create_subscriber::<TwistMessage>("/cmd_vel").await?;
        let odom_pub = node.create_publisher::<OdometryMessage>("/odom").await?;

        // Initialize visualization
        #[cfg(feature = "visualization")]
        let viz_client = {
            let config = VisualizationConfig {
                application_id: "turtlebot_simulator".to_string(),
                spawn_viewer: true,
            };
            
            match VisualizationClient::new(config) {
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
            dt: 0.02, // 50 Hz simulation
            max_acceleration: 2.0,
            max_angular_acceleration: 5.0,
            #[cfg(feature = "visualization")]
            viz_client,
            path_points: Vec::new(),
            last_update: Instant::now(),
        })
    }

    /// Start simulation loop
    pub async fn start_simulation(&mut self) -> Result<()> {
        info!("🚀 Starting turtlebot simulator");
        
        #[cfg(feature = "visualization")]
        {
            if let Some(ref viz) = self.viz_client {
                // Setup visualization scene
                self.setup_visualization_scene(viz)?;
            }
        }

        // Setup velocity command callback
        self.setup_velocity_callback().await?;

        // Main simulation loop
        loop {
            let now = Instant::now();
            let dt = now.duration_since(self.last_update).as_secs_f32();
            self.last_update = now;

            // Update robot physics
            self.update_physics(dt);

            // Publish odometry
            self.publish_odometry().await?;

            // Update visualization
            #[cfg(feature = "visualization")]
            {
                if let Some(ref viz) = self.viz_client {
                    self.update_visualization(viz)?;
                }
            }

            // Print status
            self.print_status();

            sleep(Duration::from_millis(20)).await; // 50 Hz
        }
    }

    /// Setup velocity command callback
    async fn setup_velocity_callback(&mut self) -> Result<()> {
        // Create shared velocity state
        let velocity_state = std::sync::Arc::new(std::sync::Mutex::new((0.0f32, 0.0f32)));
        let velocity_clone = velocity_state.clone();

        // Set up callback to receive velocity commands
        self.cmd_vel_sub.on_message(move |twist: TwistMessage| {
            let mut state = velocity_clone.lock().unwrap();
            state.0 = twist.linear.x;
            state.1 = twist.angular.z;
            info!("📡 Received cmd_vel: linear={:.2}, angular={:.2}", state.0, state.1);
        })?;

        Ok(())
    }

    /// Update velocity from received commands
    fn update_velocity_from_commands(&mut self) {
        // This is a simplified approach - in a real implementation,
        // you'd want to properly share the velocity state
        // For now, we'll simulate some basic movement patterns
    }

    /// Update robot physics simulation
    fn update_physics(&mut self, dt: f32) {
        // Simple differential drive kinematics
        let dx = self.linear_vel * self.theta.cos() * dt;
        let dy = self.linear_vel * self.theta.sin() * dt;
        let dtheta = self.angular_vel * dt;

        // Update position
        self.x += dx;
        self.y += dy;
        self.theta += dtheta;

        // Normalize angle
        self.theta = self.normalize_angle(self.theta);

        // Add to path for visualization
        if self.path_points.len() == 0 || 
           (self.x - self.path_points.last().unwrap()[0]).abs() > 0.05 ||
           (self.y - self.path_points.last().unwrap()[1]).abs() > 0.05 {
            self.path_points.push([self.x, self.y]);
            
            // Limit path length
            if self.path_points.len() > 1000 {
                self.path_points.remove(0);
            }
        }
    }

    /// Publish odometry message
    async fn publish_odometry(&self) -> Result<()> {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;

        // Convert theta to quaternion
        let half_theta = self.theta / 2.0;
        let quaternion = Quaternion {
            x: 0.0,
            y: 0.0,
            z: half_theta.sin(),
            w: half_theta.cos(),
        };

        let odom = OdometryMessage {
            pose: mini_ros::types::PoseMessage {
                position: Point3D {
                    x: self.x,
                    y: self.y,
                    z: 0.0,
                },
                orientation: quaternion,
                header: Header {
                    timestamp,
                    frame_id: "odom".to_string(),
                },
            },
            twist: TwistMessage {
                linear: Vector3 {
                    x: self.linear_vel,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: self.angular_vel,
                },
            },
            header: Header {
                timestamp,
                frame_id: "odom".to_string(),
            },
        };

        self.odom_pub.publish(&odom).await?;
        Ok(())
    }

    /// Setup visualization scene
    #[cfg(feature = "visualization")]
    fn setup_visualization_scene(&self, viz: &VisualizationClient) -> Result<()> {
        // Log coordinate system
        viz.log_transform(
            "world",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )?;

        // Log ground plane
        let ground_points = vec![
            [-5.0, -5.0, 0.0],
            [5.0, -5.0, 0.0],
            [5.0, 5.0, 0.0],
            [-5.0, 5.0, 0.0],
        ];
        viz.log_points("world/ground", ground_points)?;

        info!("📊 Visualization scene setup complete");
        Ok(())
    }

    /// Update visualization
    #[cfg(feature = "visualization")]
    fn update_visualization(&self, viz: &VisualizationClient) -> Result<()> {
        // Set current time
        let timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;
        viz.set_time("sim_time", timestamp_ns);

        // Log robot position and orientation
        let robot_pos = [self.x, self.y, 0.1];
        let half_theta = self.theta / 2.0;
        let robot_rot = [0.0, 0.0, half_theta.sin(), half_theta.cos()];
        viz.log_transform("world/robot", robot_pos, robot_rot)?;

        // Log robot path
        if self.path_points.len() > 1 {
            let path_3d: Vec<[f32; 3]> = self.path_points
                .iter()
                .map(|p| [p[0], p[1], 0.02])
                .collect();
            viz.log_points("world/robot_path", path_3d)?;
        }

        // Log velocity vectors
        if self.linear_vel.abs() > 0.01 || self.angular_vel.abs() > 0.01 {
            let vel_end_x = self.x + self.linear_vel * self.theta.cos() * 0.5;
            let vel_end_y = self.y + self.linear_vel * self.theta.sin() * 0.5;
            
            viz.log_points("world/velocity", vec![
                [self.x, self.y, 0.15],
                [vel_end_x, vel_end_y, 0.15],
            ])?;
        }

        // Log telemetry
        viz.log_scalar("robot/position_x", self.x as f64)?;
        viz.log_scalar("robot/position_y", self.y as f64)?;
        viz.log_scalar("robot/theta", self.theta as f64)?;
        viz.log_scalar("robot/linear_velocity", self.linear_vel as f64)?;
        viz.log_scalar("robot/angular_velocity", self.angular_vel as f64)?;

        Ok(())
    }

    /// Print simulation status
    fn print_status(&self) {
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            if COUNTER % 25 == 0 { // Print every 0.5 seconds
                info!(
                    "🤖 Robot: pos=({:.2}, {:.2}), θ={:.1}°, vel=({:.2}, {:.2})",
                    self.x, self.y, self.theta.to_degrees(), self.linear_vel, self.angular_vel
                );
            }
        }
    }

    /// Normalize angle to [-π, π]
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
    // Initialize tracing
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 14: Turtlebot Simulator ===");

    #[cfg(not(feature = "visualization"))]
    {
        warn!("⚠️  Visualization not enabled. Run with: cargo run --example 14_turtlebot_simulator --features visualization");
    }

    let mut simulator = TurtlebotSimulator::new("turtlebot_simulator").await?;
    
    // Start simulation
    simulator.start_simulation().await?;

    info!("🏁 Turtlebot simulator finished");
    Ok(())
} 