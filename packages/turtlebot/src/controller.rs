//! # Turtlebot Controller
//!
//! A comprehensive turtlebot control system with:
//! - Movement pattern execution (forward, backward, circular, square)
//! - Odometry feedback processing
//! - Safety features with velocity limits
//! - Real-time status monitoring
//!
//! This demonstrates miniROS core robotics functionality with minimal complexity.

use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, OdometryMessage, Vector3, Point3D, Quaternion, Header, MiniRosMessage};
use std::f32::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::{sleep, Instant};
use tracing::{info, warn, error};

/// Movement patterns for the turtlebot
#[derive(Debug, Clone)]
pub enum MovementPattern {
    Stop,
    Forward(f32),          // Forward velocity in m/s
    Backward(f32),         // Backward velocity in m/s
    TurnLeft(f32),         // Angular velocity in rad/s
    TurnRight(f32),        // Angular velocity in rad/s
    Circle(f32, f32),      // (linear_vel, angular_vel)
    Square(f32),           // Side length in meters
    Figure8(f32),          // Radius in meters
}

/// Robot pose information
#[derive(Debug, Clone)]
pub struct RobotPose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub linear_vel: f32,
    pub angular_vel: f32,
    pub timestamp: Instant,
}

impl Default for RobotPose {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
            timestamp: Instant::now(),
        }
    }
}

/// Turtlebot controller with improved state management
pub struct TurtlebotController {
    node: Node,
    cmd_vel_pub: Publisher<TwistMessage>,
    odom_sub: Subscriber<OdometryMessage>,
    current_pose: Arc<Mutex<RobotPose>>,
    movement_pattern: MovementPattern,
    start_time: Instant,
    max_linear_speed: f32,
    max_angular_speed: f32,
    running: bool,
}

impl TurtlebotController {
    /// Create a new turtlebot controller with safety limits
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        // Create publishers and subscribers
        let cmd_vel_pub = node.create_publisher::<TwistMessage>("/cmd_vel").await?;
        let odom_sub = node.create_subscriber::<OdometryMessage>("/odom").await?;

        Ok(Self {
            node,
            cmd_vel_pub,
            odom_sub,
            current_pose: Arc::new(Mutex::new(RobotPose::default())),
            movement_pattern: MovementPattern::Stop,
            start_time: Instant::now(),
            max_linear_speed: 1.0,  // 1 m/s max
            max_angular_speed: 2.0, // 2 rad/s max
            running: true,
        })
    }

    /// Set safety limits
    pub fn set_speed_limits(mut self, max_linear: f32, max_angular: f32) -> Self {
        self.max_linear_speed = max_linear;
        self.max_angular_speed = max_angular;
        self
    }

    /// Set up odometry callback with improved error handling
    pub async fn setup_odometry_callback(&mut self) -> Result<()> {
        let pose_ref = self.current_pose.clone();

        self.odom_sub.on_message(move |odom: OdometryMessage| {
            let yaw = 2.0 * odom.pose.orientation.z.atan2(odom.pose.orientation.w);
            
            let new_pose = RobotPose {
                x: odom.pose.position.x,
                y: odom.pose.position.y,
                yaw,
                linear_vel: odom.twist.linear.x,
                angular_vel: odom.twist.angular.z,
                timestamp: Instant::now(),
            };

            if let Ok(mut pose) = pose_ref.lock() {
                *pose = new_pose;
            }

            info!(
                "📍 Robot pose: x={:.2}, y={:.2}, yaw={:.2}°, vel={:.2}m/s",
                odom.pose.position.x,
                odom.pose.position.y,
                yaw.to_degrees(),
                odom.twist.linear.x
            );
        })?;

        Ok(())
    }

    /// Publish velocity command with safety checks
    pub async fn publish_velocity(&self, linear: Vector3, angular: Vector3) -> Result<()> {
        // Apply safety limits
        let safe_linear = Vector3 {
            x: linear.x.clamp(-self.max_linear_speed, self.max_linear_speed),
            y: linear.y.clamp(-self.max_linear_speed, self.max_linear_speed),
            z: linear.z.clamp(-self.max_linear_speed, self.max_linear_speed),
        };
        
        let safe_angular = Vector3 {
            x: angular.x.clamp(-self.max_angular_speed, self.max_angular_speed),
            y: angular.y.clamp(-self.max_angular_speed, self.max_angular_speed),
            z: angular.z.clamp(-self.max_angular_speed, self.max_angular_speed),
        };

        let twist = TwistMessage { 
            linear: safe_linear, 
            angular: safe_angular 
        };
        
        // Validate command
        if let Err(e) = twist.validate() {
            warn!("⚠️ Invalid twist command: {}", e);
            return Ok(());
        }
        
        self.cmd_vel_pub.publish(&twist).await?;
        
        if twist.linear.x != 0.0 || twist.angular.z != 0.0 {
            info!(
                "🎮 Velocity: linear={:.2}m/s, angular={:.2}rad/s",
                twist.linear.x, twist.angular.z
            );
        }
        
        Ok(())
    }

    /// Execute movement pattern with improved timing
    pub async fn execute_pattern(&mut self, pattern: MovementPattern, duration_sec: f64) -> Result<()> {
        self.movement_pattern = pattern.clone();
        self.start_time = Instant::now();
        
        info!("🤖 Starting movement pattern: {:?} for {:.1}s", pattern, duration_sec);
        
        let end_time = Instant::now() + Duration::from_secs_f64(duration_sec);
        let mut last_status = Instant::now();
        
        while Instant::now() < end_time && self.running {
            // Calculate current phase for time-based patterns
            let elapsed = self.start_time.elapsed().as_secs_f32();
            
            match &self.movement_pattern {
                MovementPattern::Stop => {
                    self.publish_velocity(
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }
                
                MovementPattern::Forward(speed) => {
                    self.publish_velocity(
                        Vector3 { x: *speed, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }
                
                MovementPattern::Backward(speed) => {
                    self.publish_velocity(
                        Vector3 { x: -speed, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }
                
                MovementPattern::TurnLeft(angular_speed) => {
                    self.publish_velocity(
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: *angular_speed },
                    ).await?;
                }
                
                MovementPattern::TurnRight(angular_speed) => {
                    self.publish_velocity(
                        Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: -angular_speed },
                    ).await?;
                }
                
                MovementPattern::Circle(linear_vel, angular_vel) => {
                    self.publish_velocity(
                        Vector3 { x: *linear_vel, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: *angular_vel },
                    ).await?;
                }
                
                MovementPattern::Square(side_length) => {
                    // Improved square pattern with proper timing
                    let side_time = side_length / 0.3; // 0.3 m/s forward speed
                    let turn_time = (PI / 2.0) / 1.0;  // 1.0 rad/s turn speed
                    let cycle_time = side_time + turn_time;
                    
                    let phase = (elapsed % cycle_time) / cycle_time;
                    
                    if phase < (side_time / cycle_time) {
                        // Move forward
                        self.publish_velocity(
                            Vector3 { x: 0.3, y: 0.0, z: 0.0 },
                            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        ).await?;
                    } else {
                        // Turn 90 degrees
                        self.publish_velocity(
                            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                            Vector3 { x: 0.0, y: 0.0, z: 1.0 },
                        ).await?;
                    }
                }
                
                MovementPattern::Figure8(radius) => {
                    // Figure-8 pattern using sinusoidal motion
                    let frequency = 0.5; // Complete figure-8 every 2 seconds
                    let linear_speed = 0.3;
                    let angular_speed = (2.0 * PI * frequency * (elapsed * frequency).sin()) / radius;
                    
                    self.publish_velocity(
                        Vector3 { x: linear_speed, y: 0.0, z: 0.0 },
                        Vector3 { x: 0.0, y: 0.0, z: angular_speed },
                    ).await?;
                }
            }
            
            // Print status every 2 seconds
            if last_status.elapsed() > Duration::from_secs(2) {
                self.print_status().await;
                last_status = Instant::now();
            }
            
            sleep(Duration::from_millis(50)).await; // 20Hz control loop
        }
        
        // Stop the robot
        self.publish_velocity(
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        ).await?;
        
        info!("✅ Movement pattern completed");
        Ok(())
    }

    /// Print current status
    async fn print_status(&self) {
        if let Ok(pose) = self.current_pose.lock() {
            info!(
                "📊 Status: pos=({:.2}, {:.2}), yaw={:.1}°, speeds=({:.2}, {:.2})",
                pose.x, pose.y, pose.yaw.to_degrees(), pose.linear_vel, pose.angular_vel
            );
        }
    }

    /// Stop the robot immediately
    pub async fn emergency_stop(&mut self) -> Result<()> {
        self.running = false;
        self.publish_velocity(
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        ).await?;
        warn!("🛑 Emergency stop activated!");
        Ok(())
    }

    /// Run comprehensive demonstration sequence
    pub async fn run_demo(&mut self) -> Result<()> {
        info!("🚀 Starting turtlebot controller demo");
        
        // Setup odometry callback
        self.setup_odometry_callback().await?;
        
        // Publish initial mock odometry for demonstration
        self.publish_mock_odometry().await?;
        
        sleep(Duration::from_millis(1000)).await;
        
        // Execute demonstration patterns
        info!("📋 Demo sequence - Turtlebot movement patterns:");
        
        // 1. Forward movement
        info!("🔸 1. Forward movement (3s)");
        self.execute_pattern(MovementPattern::Forward(0.4), 3.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 2. Backward movement
        info!("🔸 2. Backward movement (2s)");
        self.execute_pattern(MovementPattern::Backward(0.3), 2.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 3. Circular motion
        info!("🔸 3. Circular motion (5s)");
        self.execute_pattern(MovementPattern::Circle(0.3, 0.8), 5.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 4. Square pattern
        info!("🔸 4. Square pattern (15s)");
        self.execute_pattern(MovementPattern::Square(1.0), 15.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 5. Figure-8 pattern
        info!("🔸 5. Figure-8 pattern (10s)");
        self.execute_pattern(MovementPattern::Figure8(0.5), 10.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 6. Spin in place
        info!("🔸 6. Spin in place (4s)");
        self.execute_pattern(MovementPattern::TurnLeft(1.5), 4.0).await?;
        
        info!("🎉 Turtlebot controller demo completed!");
        Ok(())
    }

    /// Publish mock odometry data for demonstration
    async fn publish_mock_odometry(&mut self) -> Result<()> {
        let odom_pub = self.node.create_publisher::<OdometryMessage>("/odom").await?;
        
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;
        
        let odom = OdometryMessage {
            pose: mini_ros::types::PoseMessage {
                position: Point3D { x: 0.0, y: 0.0, z: 0.0 },
                orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                header: Header {
                    timestamp: current_time,
                    frame_id: "odom".to_string(),
                },
            },
            twist: TwistMessage {
                linear: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            header: Header {
                timestamp: current_time,
                frame_id: "odom".to_string(),
            },
        };
        
        odom_pub.publish(&odom).await?;
        info!("📡 Published initial odometry data");
        
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing with better formatting
    tracing_subscriber::fmt()
        .with_env_filter("info")
        .init();

    info!("=== miniROS Turtlebot Controller ===");
    info!("🤖 Package: turtlebot");
    info!("📍 Executable: controller");

    // Create turtlebot controller with safety limits
    let mut controller = TurtlebotController::new("turtlebot_controller")
        .await?
        .set_speed_limits(0.8, 2.0); // Reasonable speed limits
    
    // Handle Ctrl+C gracefully
    let running = Arc::new(Mutex::new(true));
    let running_clone = running.clone();
    
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        info!("📡 Received Ctrl+C, stopping...");
        *running_clone.lock().unwrap() = false;
    });
    
    // Run the demonstration
    match controller.run_demo().await {
        Ok(()) => info!("✅ Demo completed successfully"),
        Err(e) => error!("❌ Demo failed: {}", e),
    }
    
    // Ensure robot is stopped
    controller.emergency_stop().await?;
    
    // Keep node alive briefly to process final messages
    sleep(Duration::from_secs(1)).await;

    info!("🏁 Turtlebot controller finished");
    Ok(())
} 