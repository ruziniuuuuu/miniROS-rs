//! # Complete Turtlebot Demo
//!
//! This example demonstrates the complete turtlebot control system:
//! - Physics simulation with Rerun visualization
//! - Keyboard control interface
//! - Real-time robot state feedback
//! - All running in a single process to avoid port conflicts
//!
//! Controls:
//! - W: Forward
//! - S: Backward  
//! - A: Turn Left
//! - D: Turn Right
//! - Q: Increase speed
//! - E: Decrease speed
//! - Space: Stop
//! - X: Exit

use mini_ros::prelude::*;
use mini_ros::visualization::VisualizationClient;
use std::f32::consts::PI;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::time::{Instant, sleep};
use tracing::info;

#[cfg(feature = "visualization")]
use mini_ros::visualization::VisualizationConfig;

/// Complete turtlebot demo system
pub struct TurtlebotDemo {
    // Simulation state
    x: f32,
    y: f32,
    theta: f32,
    linear_vel: f32,
    angular_vel: f32,

    // Control state
    target_linear: f32,
    target_angular: f32,
    speed_setting: f32,

    // Visualization
    #[cfg(feature = "visualization")]
    viz_client: Option<VisualizationClient>,

    // Path tracking
    path_points: Vec<[f32; 2]>,
    last_update: Instant,
}

impl TurtlebotDemo {
    /// Create new demo
    pub async fn new() -> Result<Self> {
        // Initialize visualization
        #[cfg(feature = "visualization")]
        let viz_client = {
            let config = VisualizationConfig {
                application_id: "turtlebot_demo".to_string(),
                spawn_viewer: true,
            };

            match VisualizationClient::new(config) {
                Ok(client) => {
                    info!("📊 Rerun visualization initialized");
                    Some(client)
                }
                Err(e) => {
                    info!("⚠️  Visualization disabled: {}", e);
                    None
                }
            }
        };

        Ok(Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
            target_linear: 0.0,
            target_angular: 0.0,
            speed_setting: 0.5,
            #[cfg(feature = "visualization")]
            viz_client,
            path_points: Vec::new(),
            last_update: Instant::now(),
        })
    }

    /// Start the complete demo
    pub async fn run_demo(&mut self) -> Result<()> {
        info!("🚀 Starting complete turtlebot demo");
        self.print_instructions();

        #[cfg(feature = "visualization")]
        {
            if let Some(ref viz) = self.viz_client {
                self.setup_visualization_scene(viz)?;
            }
        }

        // Start input handling in a separate task
        let (tx, mut rx) = tokio::sync::mpsc::channel::<char>(32);

        tokio::spawn(async move {
            Self::keyboard_input_task(tx).await;
        });

        // Main simulation loop
        loop {
            let now = Instant::now();
            let dt = now.duration_since(self.last_update).as_secs_f32();
            self.last_update = now;

            // Process input commands
            while let Ok(key) = rx.try_recv() {
                if self.process_key(key) {
                    info!("🏁 Demo completed!");
                    return Ok(()); // Exit requested
                }
            }

            // Update physics
            self.update_physics(dt);

            // Update visualization
            #[cfg(feature = "visualization")]
            {
                if let Some(ref viz) = self.viz_client {
                    self.update_visualization(viz)?;
                }
            }

            // Print status
            self.print_status();

            sleep(Duration::from_millis(50)).await; // 20 Hz
        }
    }

    /// Process keyboard input
    fn process_key(&mut self, key: char) -> bool {
        match key.to_ascii_lowercase() {
            'w' => {
                self.target_linear = self.speed_setting;
                self.target_angular = 0.0;
                info!("⬆️  Forward at {:.1} m/s", self.speed_setting);
            }
            's' => {
                self.target_linear = -self.speed_setting;
                self.target_angular = 0.0;
                info!("⬇️  Backward at {:.1} m/s", self.speed_setting);
            }
            'a' => {
                self.target_linear = 0.0;
                self.target_angular = self.speed_setting;
                info!("⬅️  Turn left at {:.1} rad/s", self.speed_setting);
            }
            'd' => {
                self.target_linear = 0.0;
                self.target_angular = -self.speed_setting;
                info!("➡️  Turn right at {:.1} rad/s", self.speed_setting);
            }
            'q' => {
                self.speed_setting = f32::min(self.speed_setting + 0.1, 2.0);
                info!("🔼 Speed increased to {:.1}", self.speed_setting);
            }
            'e' => {
                self.speed_setting = f32::max(self.speed_setting - 0.1, 0.1);
                info!("🔽 Speed decreased to {:.1}", self.speed_setting);
            }
            ' ' => {
                self.target_linear = 0.0;
                self.target_angular = 0.0;
                info!("🛑 Emergency stop!");
            }
            'x' => {
                info!("🚪 Exit requested");
                return true; // Signal exit
            }
            _ => {
                self.target_linear = 0.0;
                self.target_angular = 0.0;
                info!("🛑 Stop (unknown key: {})", key);
            }
        }
        false
    }

    /// Update robot physics
    fn update_physics(&mut self, dt: f32) {
        // Smooth velocity changes
        let alpha = 5.0 * dt; // Acceleration parameter
        self.linear_vel += alpha * (self.target_linear - self.linear_vel);
        self.angular_vel += alpha * (self.target_angular - self.angular_vel);

        // Update position (differential drive kinematics)
        let dx = self.linear_vel * self.theta.cos() * dt;
        let dy = self.linear_vel * self.theta.sin() * dt;
        let dtheta = self.angular_vel * dt;

        self.x += dx;
        self.y += dy;
        self.theta += dtheta;

        // Normalize angle
        while self.theta > PI {
            self.theta -= 2.0 * PI;
        }
        while self.theta < -PI {
            self.theta += 2.0 * PI;
        }

        // Add to path for visualization
        if self.path_points.is_empty()
            || (self.x - self.path_points.last().unwrap()[0]).abs() > 0.05
            || (self.y - self.path_points.last().unwrap()[1]).abs() > 0.05
        {
            self.path_points.push([self.x, self.y]);

            // Limit path length
            if self.path_points.len() > 1000 {
                self.path_points.remove(0);
            }
        }
    }

    /// Setup visualization scene
    #[cfg(feature = "visualization")]
    fn setup_visualization_scene(&self, viz: &VisualizationClient) -> Result<()> {
        // Log coordinate system
        viz.log_transform("world", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])?;

        // Log ground plane markers
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

        // Log velocity vectors
        if self.linear_vel.abs() > 0.01 || self.angular_vel.abs() > 0.01 {
            let vel_end_x = self.x + self.linear_vel * self.theta.cos() * 0.5;
            let vel_end_y = self.y + self.linear_vel * self.theta.sin() * 0.5;

            viz.log_points(
                "world/velocity",
                vec![[self.x, self.y, 0.15], [vel_end_x, vel_end_y, 0.15]],
            )?;
        }

        // Log telemetry
        viz.log_scalar("robot/position_x", self.x as f64)?;
        viz.log_scalar("robot/position_y", self.y as f64)?;
        viz.log_scalar("robot/theta", self.theta as f64)?;
        viz.log_scalar("robot/linear_velocity", self.linear_vel as f64)?;
        viz.log_scalar("robot/angular_velocity", self.angular_vel as f64)?;

        Ok(())
    }

    /// Print control instructions
    fn print_instructions(&self) {
        info!("📋 Turtlebot Demo Controls:");
        info!("   W: Forward    S: Backward");
        info!("   A: Turn Left  D: Turn Right");
        info!("   Q: Speed Up   E: Speed Down");
        info!("   Space: Stop   X: Exit");
        info!("   Current speed setting: {:.1}", self.speed_setting);
        info!("   Type commands and press Enter");
    }

    /// Print simulation status
    fn print_status(&self) {
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER += 1;
            if COUNTER % 40 == 0 {
                // Print every 2 seconds
                info!(
                    "🤖 Robot: pos=({:.2}, {:.2}), θ={:.1}°, vel=({:.2}, {:.2})",
                    self.x,
                    self.y,
                    self.theta.to_degrees(),
                    self.linear_vel,
                    self.angular_vel
                );
            }
        }
    }

    /// Keyboard input task
    async fn keyboard_input_task(tx: tokio::sync::mpsc::Sender<char>) {
        use std::io;

        loop {
            let mut input = String::new();
            if io::stdin().read_line(&mut input).is_ok() {
                if let Some(key) = input.trim().chars().next() {
                    if tx.send(key).await.is_err() {
                        break; // Channel closed
                    }
                    if key == 'x' {
                        break; // Exit requested
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 15: Complete Turtlebot Demo ===");

    #[cfg(not(feature = "visualization"))]
    {
        info!(
            "⚠️  For best experience, run with: cargo run --example 15_complete_demo --features visualization"
        );
    }

    let mut demo = TurtlebotDemo::new().await?;
    demo.run_demo().await?;

    Ok(())
}
