//! # Turtlebot Teleop Controller
//!
//! Keyboard teleoperation interface for turtlebot control:
//! - Cross-platform keyboard input handling
//! - Real-time velocity control with configurable speed limits
//! - Safety features and emergency stop
//! - Improved user interface with status display
//!
//! Controls:
//! - W/S: Forward/Backward
//! - A/D: Turn Left/Right  
//! - Q/E: Increase/Decrease speed
//! - Spacebar: Emergency stop
//! - X: Exit

use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, Vector3, MiniRosMessage};
use std::io::{self, Write};
use std::sync::{Arc, AtomicBool, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tokio::time::sleep;
use tracing::{info, warn, error};

/// Keyboard teleop controller for turtlebot with improved input handling
pub struct TurtlebotTeleop {
    node: Node,
    cmd_vel_pub: Publisher<TwistMessage>,
    current_linear: Arc<Mutex<f32>>,
    current_angular: Arc<Mutex<f32>>,
    linear_speed_scale: Arc<Mutex<f32>>,
    angular_speed_scale: Arc<Mutex<f32>>,
    max_linear_speed: f32,
    max_angular_speed: f32,
    running: Arc<AtomicBool>,
    last_command_time: Arc<Mutex<Instant>>,
    safety_timeout: Duration,
}

impl TurtlebotTeleop {
    /// Create new teleop controller with safety features
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        let cmd_vel_pub = node.create_publisher::<TwistMessage>("/cmd_vel").await?;

        Ok(Self {
            node,
            cmd_vel_pub,
            current_linear: Arc::new(Mutex::new(0.0)),
            current_angular: Arc::new(Mutex::new(0.0)),
            linear_speed_scale: Arc::new(Mutex::new(0.5)),  // Default speed scale
            angular_speed_scale: Arc::new(Mutex::new(1.0)), // Default turn scale
            max_linear_speed: 1.0,  // 1 m/s max
            max_angular_speed: 2.0, // 2 rad/s max
            running: Arc::new(AtomicBool::new(true)),
            last_command_time: Arc::new(Mutex::new(Instant::now())),
            safety_timeout: Duration::from_secs(1), // Stop if no input for 1 second
        })
    }

    /// Set maximum speed limits
    pub fn set_speed_limits(mut self, max_linear: f32, max_angular: f32) -> Self {
        self.max_linear_speed = max_linear;
        self.max_angular_speed = max_angular;
        self
    }

    /// Start teleoperation control
    pub async fn start_teleop(&mut self) -> Result<()> {
        info!("🎮 Starting turtlebot teleop control");
        self.print_instructions();

        // Spawn keyboard input thread
        let running = self.running.clone();
        let current_linear = self.current_linear.clone();
        let current_angular = self.current_angular.clone();
        let linear_scale = self.linear_speed_scale.clone();
        let angular_scale = self.angular_speed_scale.clone();
        let last_command = self.last_command_time.clone();
        let max_linear = self.max_linear_speed;
        let max_angular = self.max_angular_speed;

        thread::spawn(move || {
            Self::keyboard_input_loop(
                running.clone(),
                current_linear,
                current_angular,
                linear_scale,
                angular_scale,
                last_command,
                max_linear,
                max_angular,
            );
        });

        // Start status display thread
        let running_status = self.running.clone();
        let linear_display = self.current_linear.clone();
        let angular_display = self.current_angular.clone();
        let linear_scale_display = self.linear_speed_scale.clone();
        let angular_scale_display = self.angular_speed_scale.clone();

        thread::spawn(move || {
            Self::status_display_loop(
                running_status,
                linear_display,
                angular_display,
                linear_scale_display,
                angular_scale_display,
            );
        });

        // Main control loop
        while self.running.load(std::sync::atomic::Ordering::Relaxed) {
            let (linear, angular) = {
                let lin = *self.current_linear.lock().unwrap();
                let ang = *self.current_angular.lock().unwrap();
                (lin, ang)
            };

            // Check for safety timeout
            let should_stop = {
                let last_time = *self.last_command_time.lock().unwrap();
                last_time.elapsed() > self.safety_timeout && (linear != 0.0 || angular != 0.0)
            };

            if should_stop {
                // Safety timeout - stop the robot
                self.publish_velocity(0.0, 0.0).await?;
                // Reset current velocities
                *self.current_linear.lock().unwrap() = 0.0;
                *self.current_angular.lock().unwrap() = 0.0;
                warn!("⚠️ Safety timeout - robot stopped");
            } else {
                // Publish current velocity command
                self.publish_velocity(linear, angular).await?;
            }
            
            sleep(Duration::from_millis(50)).await; // 20 Hz control loop
        }

        // Send final stop command
        self.publish_velocity(0.0, 0.0).await?;
        info!("🛑 Teleop control stopped");
        
        Ok(())
    }

    /// Publish velocity command with safety checks
    async fn publish_velocity(&self, linear_x: f32, angular_z: f32) -> Result<()> {
        // Apply speed limits
        let safe_linear = linear_x.clamp(-self.max_linear_speed, self.max_linear_speed);
        let safe_angular = angular_z.clamp(-self.max_angular_speed, self.max_angular_speed);

        let twist = TwistMessage {
            linear: Vector3 { x: safe_linear, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: safe_angular },
        };

        // Validate and publish
        match twist.validate() {
            Ok(()) => {
                self.cmd_vel_pub.publish(&twist).await?;
            }
            Err(e) => {
                warn!("⚠️ Invalid velocity command: {}", e);
            }
        }

        Ok(())
    }

    /// Print control instructions
    fn print_instructions(&self) {
        println!("\n┌─────────────────────────────────────────────┐");
        println!("│           🤖 Turtlebot Teleop              │");
        println!("├─────────────────────────────────────────────┤");
        println!("│ Movement Controls:                          │");
        println!("│   W + Enter: Move Forward                   │");
        println!("│   S + Enter: Move Backward                  │");
        println!("│   A + Enter: Turn Left                      │");
        println!("│   D + Enter: Turn Right                     │");
        println!("├─────────────────────────────────────────────┤");
        println!("│ Speed Controls:                             │");
        println!("│   Q + Enter: Increase Speed                 │");
        println!("│   E + Enter: Decrease Speed                 │");
        println!("├─────────────────────────────────────────────┤");
        println!("│ Safety & Control:                           │");
        println!("│   Space + Enter: Emergency Stop             │");
        println!("│   X + Enter: Exit Program                   │");
        println!("├─────────────────────────────────────────────┤");
        println!("│ Speed Limits: {:.1} m/s, {:.1} rad/s           │", 
                 self.max_linear_speed, self.max_angular_speed);
        println!("│ Safety Timeout: {:?}                        │", self.safety_timeout);
        println!("└─────────────────────────────────────────────┘");
        println!("🎮 Ready for input. Type commands and press Enter:");
    }

    /// Keyboard input handling loop (runs in separate thread)
    fn keyboard_input_loop(
        running: Arc<AtomicBool>,
        current_linear: Arc<Mutex<f32>>,
        current_angular: Arc<Mutex<f32>>,
        linear_scale: Arc<Mutex<f32>>,
        angular_scale: Arc<Mutex<f32>>,
        last_command_time: Arc<Mutex<Instant>>,
        max_linear: f32,
        max_angular: f32,
    ) {
        let speed_increment = 0.1f32;
        let stdin = io::stdin();

        while running.load(std::sync::atomic::Ordering::Relaxed) {
            let mut input = String::new();
            match stdin.read_line(&mut input) {
                Ok(_) => {
                    let key = input.trim().to_lowercase();
                    
                    // Update last command time
                    *last_command_time.lock().unwrap() = Instant::now();
                    
                    let (linear, angular) = match key.as_str() {
                        "w" => {
                            let scale = *linear_scale.lock().unwrap();
                            println!("⬆️ Forward (speed: {:.1})", scale);
                            (scale, 0.0)
                        },
                        "s" => {
                            let scale = *linear_scale.lock().unwrap();
                            println!("⬇️ Backward (speed: {:.1})", -scale);
                            (-scale, 0.0)
                        },
                        "a" => {
                            let scale = *angular_scale.lock().unwrap();
                            println!("⬅️ Turn Left (speed: {:.1})", scale);
                            (0.0, scale)
                        },
                        "d" => {
                            let scale = *angular_scale.lock().unwrap();
                            println!("➡️ Turn Right (speed: {:.1})", -scale);
                            (0.0, -scale)
                        },
                        
                        // Speed control
                        "q" => {
                            let mut lin_scale = linear_scale.lock().unwrap();
                            let mut ang_scale = angular_scale.lock().unwrap();
                            *lin_scale = f32::min(*lin_scale + speed_increment, max_linear);
                            *ang_scale = f32::min(*ang_scale + speed_increment, max_angular);
                            println!("🔼 Speed Increased: linear={:.1}, angular={:.1}", *lin_scale, *ang_scale);
                            (0.0, 0.0)
                        },
                        "e" => {
                            let mut lin_scale = linear_scale.lock().unwrap();
                            let mut ang_scale = angular_scale.lock().unwrap();
                            *lin_scale = f32::max(*lin_scale - speed_increment, 0.1);
                            *ang_scale = f32::max(*ang_scale - speed_increment, 0.1);
                            println!("🔽 Speed Decreased: linear={:.1}, angular={:.1}", *lin_scale, *ang_scale);
                            (0.0, 0.0)
                        },
                        
                        // Special commands
                        " " | "space" => {
                            println!("🛑 Emergency Stop!");
                            (0.0, 0.0)
                        },
                        
                        "x" | "exit" | "quit" => {
                            println!("🚪 Exit requested");
                            running.store(false, std::sync::atomic::Ordering::Relaxed);
                            break;
                        },
                        
                        "h" | "help" => {
                            println!("📋 Available commands: w/s (forward/back), a/d (left/right), q/e (speed), space (stop), x (exit)");
                            (0.0, 0.0)
                        },
                        
                        "" => {
                            // Empty input - stop movement
                            (0.0, 0.0)
                        },
                        
                        _ => {
                            println!("❓ Unknown command: '{}'. Type 'h' for help.", key);
                            (0.0, 0.0)
                        }
                    };

                    // Update shared velocity values
                    *current_linear.lock().unwrap() = linear;
                    *current_angular.lock().unwrap() = angular;
                }
                Err(e) => {
                    eprintln!("❌ Input error: {}", e);
                    break;
                }
            }
        }
        
        println!("📱 Input thread terminated");
    }

    /// Status display loop (runs in separate thread)
    fn status_display_loop(
        running: Arc<AtomicBool>,
        current_linear: Arc<Mutex<f32>>,
        current_angular: Arc<Mutex<f32>>,
        linear_scale: Arc<Mutex<f32>>,
        angular_scale: Arc<Mutex<f32>>,
    ) {
        let mut last_status = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        
        while running.load(std::sync::atomic::Ordering::Relaxed) {
            thread::sleep(Duration::from_secs(5)); // Update every 5 seconds
            
            let current_status = {
                let lin = *current_linear.lock().unwrap();
                let ang = *current_angular.lock().unwrap();
                let lin_scale = *linear_scale.lock().unwrap();
                let ang_scale = *angular_scale.lock().unwrap();
                (lin, ang, lin_scale, ang_scale)
            };

            // Only print if status changed or robot is moving
            if current_status != last_status || current_status.0 != 0.0 || current_status.1 != 0.0 {
                println!("📊 Status: cmd_vel=({:.2}, {:.2}), scales=({:.1}, {:.1})", 
                        current_status.0, current_status.1, current_status.2, current_status.3);
                last_status = current_status;
            }
        }
    }

    /// Get current teleop status
    pub fn get_status(&self) -> (f32, f32, f32, f32) {
        let linear = *self.current_linear.lock().unwrap();
        let angular = *self.current_angular.lock().unwrap();
        let linear_scale = *self.linear_speed_scale.lock().unwrap();
        let angular_scale = *self.angular_speed_scale.lock().unwrap();
        (linear, angular, linear_scale, angular_scale)
    }

    /// Emergency stop
    pub async fn emergency_stop(&self) -> Result<()> {
        *self.current_linear.lock().unwrap() = 0.0;
        *self.current_angular.lock().unwrap() = 0.0;
        self.publish_velocity(0.0, 0.0).await?;
        warn!("🚨 Emergency stop activated via API");
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing with better formatting
    tracing_subscriber::fmt()
        .with_env_filter("info")
        .init();

    info!("=== miniROS Turtlebot Teleop ===");
    info!("🤖 Package: turtlebot");
    info!("📍 Executable: teleop");

    // Create teleop controller with safety limits
    let mut teleop = TurtlebotTeleop::new("turtlebot_teleop")
        .await?
        .set_speed_limits(0.8, 2.0); // Conservative limits for safety

    // Handle Ctrl+C gracefully
    let running = teleop.running.clone();
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        info!("📡 Received Ctrl+C, stopping teleop...");
        running.store(false, std::sync::atomic::Ordering::Relaxed);
    });
    
    // Start teleop control
    match teleop.start_teleop().await {
        Ok(()) => info!("✅ Teleop completed successfully"),
        Err(e) => error!("❌ Teleop failed: {}", e),
    }

    // Ensure final stop command
    teleop.emergency_stop().await?;

    info!("🏁 Turtlebot teleop finished");
    Ok(())
} 