//! # Turtlebot Teleop Keyboard Control
//!
//! This example implements keyboard teleoperation for turtlebot control:
//! - Publishes velocity commands to `/cmd_vel` based on keyboard input
//! - Real-time keyboard control (WASD keys)
//! - Safety features with configurable speed limits
//! - Clean shutdown handling
//!
//! Controls:
//! - W/S: Forward/Backward
//! - A/D: Turn Left/Right  
//! - Q/E: Increase/Decrease speed
//! - Spacebar: Emergency stop
//! - ESC: Exit

use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, Vector3, MiniRosMessage};
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use tokio::time::sleep;
use tracing::{info, warn};

/// Keyboard teleop controller for turtlebot
pub struct TurtlebotTeleop {
    #[allow(dead_code)]
    node: Node,
    cmd_vel_pub: Publisher<TwistMessage>,
    linear_speed: Arc<Mutex<f32>>,
    angular_speed: Arc<Mutex<f32>>,
    max_linear_speed: f32,
    max_angular_speed: f32,
    running: Arc<Mutex<bool>>,
}

impl TurtlebotTeleop {
    /// Create new teleop controller
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        let cmd_vel_pub = node.create_publisher::<TwistMessage>("/cmd_vel").await?;

        Ok(Self {
            node,
            cmd_vel_pub,
            linear_speed: Arc::new(Mutex::new(0.0)),
            angular_speed: Arc::new(Mutex::new(0.0)),
            max_linear_speed: 1.0,  // 1 m/s max
            max_angular_speed: 2.0, // 2 rad/s max
            running: Arc::new(Mutex::new(true)),
        })
    }

    /// Start keyboard input handling
    pub async fn start_teleop(&mut self) -> Result<()> {
        info!("🎮 Starting turtlebot teleop control");
        self.print_instructions();

        // Start keyboard input thread
        let linear_speed = self.linear_speed.clone();
        let angular_speed = self.angular_speed.clone();
        let running = self.running.clone();
        let max_linear = self.max_linear_speed;
        let max_angular = self.max_angular_speed;

        thread::spawn(move || {
            Self::keyboard_input_loop(linear_speed, angular_speed, running, max_linear, max_angular);
        });

        // Main control loop
        while *self.running.lock().unwrap() {
            let (linear, angular) = {
                let lin = *self.linear_speed.lock().unwrap();
                let ang = *self.angular_speed.lock().unwrap();
                (lin, ang)
            };

            // Publish velocity command
            self.publish_velocity(linear, angular).await?;
            
            sleep(Duration::from_millis(50)).await; // 20 Hz
        }

        // Send stop command before exit
        self.publish_velocity(0.0, 0.0).await?;
        info!("🛑 Teleop control stopped");
        
        Ok(())
    }

    /// Publish velocity command
    async fn publish_velocity(&self, linear_x: f32, angular_z: f32) -> Result<()> {
        let twist = TwistMessage {
            linear: Vector3 { x: linear_x, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: angular_z },
        };

        // Validate and publish
        if twist.validate().is_ok() {
            self.cmd_vel_pub.publish(&twist).await?;
        } else {
            warn!("⚠️  Invalid velocity command: linear={:.2}, angular={:.2}", linear_x, angular_z);
        }

        Ok(())
    }

    /// Print control instructions
    fn print_instructions(&self) {
        info!("📋 Turtlebot Teleop Controls:");
        info!("   W + Enter: Forward");
        info!("   S + Enter: Backward");
        info!("   A + Enter: Turn Left");
        info!("   D + Enter: Turn Right");
        info!("   Q + Enter: Increase speed");
        info!("   E + Enter: Decrease speed");
        info!("   Space + Enter: Emergency stop");
        info!("   X + Enter: Exit");
        info!("   Max speeds: {:.1} m/s, {:.1} rad/s", self.max_linear_speed, self.max_angular_speed);
        info!("   Note: Press key then Enter for cross-platform compatibility");
    }

    /// Keyboard input handling loop (runs in separate thread)
    fn keyboard_input_loop(
        linear_speed: Arc<Mutex<f32>>,
        angular_speed: Arc<Mutex<f32>>,
        running: Arc<Mutex<bool>>,
        max_linear: f32,
        max_angular: f32,
    ) {
        let speed_increment = 0.1f32;
        let mut current_linear_speed = 0.5f32;  // Default speed
        let mut current_angular_speed = 1.0f32; // Default turn speed

        info!("🎮 Keyboard control active. Press keys to control robot.");
        info!("Note: For cross-platform compatibility, press Enter after each key.");
        
        // For simplicity, we'll use a basic approach that works across platforms

        // Use a simple line-based input for cross-platform compatibility
        let stdin = io::stdin();

        loop {
            if !*running.lock().unwrap() {
                break;
            }

            // Read line input
            let mut input = String::new();
            if stdin.read_line(&mut input).is_ok() {
                let key = input.trim().chars().next().unwrap_or(' ');
                
                let (linear, angular) = match key.to_ascii_lowercase() {
                    'w' => {
                        info!("⬆️  Forward");
                        (current_linear_speed, 0.0)
                    },
                    's' => {
                        info!("⬇️  Backward");
                        (-current_linear_speed, 0.0)
                    },
                    'a' => {
                        info!("⬅️  Turn left");
                        (0.0, current_angular_speed)
                    },
                    'd' => {
                        info!("➡️  Turn right");
                        (0.0, -current_angular_speed)
                    },
                    
                    // Speed control
                    'q' => {
                        current_linear_speed = f32::min(current_linear_speed + speed_increment, max_linear);
                        current_angular_speed = f32::min(current_angular_speed + speed_increment, max_angular);
                        info!("🔼 Speed increased: linear={:.1}, angular={:.1}", current_linear_speed, current_angular_speed);
                        (0.0, 0.0)
                    },
                    'e' => {
                        current_linear_speed = f32::max(current_linear_speed - speed_increment, 0.1);
                        current_angular_speed = f32::max(current_angular_speed - speed_increment, 0.1);
                        info!("🔽 Speed decreased: linear={:.1}, angular={:.1}", current_linear_speed, current_angular_speed);
                        (0.0, 0.0)
                    },
                    
                    ' ' => {
                        info!("🛑 Emergency stop!");
                        (0.0, 0.0)
                    },
                    
                    'x' => { // Use 'x' for exit (easier than ESC)
                        info!("🚪 Exit requested");
                        *running.lock().unwrap() = false;
                        break;
                    },
                    
                    _ => {
                        info!("🛑 Stop (unknown key)");
                        (0.0, 0.0)
                    }
                };

                // Update shared speed values
                *linear_speed.lock().unwrap() = linear;
                *angular_speed.lock().unwrap() = angular;
            }

            thread::sleep(Duration::from_millis(10));
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 13: Turtlebot Teleop ===");

    let mut teleop = TurtlebotTeleop::new("turtlebot_teleop").await?;
    
    // Start teleop control
    teleop.start_teleop().await?;

    info!("🏁 Turtlebot teleop finished");
    Ok(())
} 