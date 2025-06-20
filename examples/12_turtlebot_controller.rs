//! # Turtlebot Controller Example
//!
//! This example demonstrates a classic ROS/ROS2 turtlebot control pattern:
//! - Publishes velocity commands to `/cmd_vel` topic
//! - Subscribes to odometry data from `/odom` topic  
//! - Implements basic movement patterns (circle, square, etc.)
//!
//! This showcases the miniROS core principle: essential robotics functionality
//! with minimal complexity.

use mini_ros::prelude::*;
use mini_ros::types::{TwistMessage, OdometryMessage, Vector3, Point3D, Quaternion, Header, MiniRosMessage};
use std::f32::consts::PI;
use std::time::Duration;
use tokio::time::{sleep, Instant};
use tracing::info;

/// Movement patterns for the turtlebot
#[derive(Debug, Clone)]
pub enum MovementPattern {
    Stop,
    Forward(f32),          // Forward velocity in m/s
    Backward(f32),         // Backward velocity in m/s
    TurnLeft(f32),         // Angular velocity in rad/s
    TurnRight(f32),        // Angular velocity in rad/s
    Circle(f32, f32),      // (linear_vel, angular_vel)
    Square(f32),           // Side length completion time
}

/// Turtlebot controller state
pub struct TurtlebotController {
    node: Node,
    cmd_vel_pub: Publisher<TwistMessage>,
    odom_sub: Subscriber<OdometryMessage>,
    movement_pattern: MovementPattern,
    start_time: Instant,
}

impl TurtlebotController {
    /// Create a new turtlebot controller
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        // Create velocity command publisher
        let cmd_vel_pub = node.create_publisher::<TwistMessage>("/cmd_vel").await?;
        
        // Create odometry subscriber
        let odom_sub = node.create_subscriber::<OdometryMessage>("/odom").await?;

        Ok(Self {
            node,
            cmd_vel_pub,
            odom_sub,
            movement_pattern: MovementPattern::Stop,
            start_time: Instant::now(),
        })
    }

    /// Set up odometry callback
    pub async fn setup_odometry_callback(&mut self) -> Result<()> {
        let pose_ref: std::sync::Arc<std::sync::Mutex<Option<Point3D>>> = 
            std::sync::Arc::new(std::sync::Mutex::new(None));
        let vel_ref: std::sync::Arc<std::sync::Mutex<Option<Vector3>>> = 
            std::sync::Arc::new(std::sync::Mutex::new(None));

        let pose_clone = pose_ref.clone();
        let vel_clone = vel_ref.clone();

        self.odom_sub.on_message(move |odom: OdometryMessage| {
            // Update current pose
            {
                let mut pose = pose_clone.lock().unwrap();
                *pose = Some(odom.pose.position.clone());
            }
            
            // Update current velocity
            {
                let mut vel = vel_clone.lock().unwrap();
                *vel = Some(odom.twist.linear.clone());
            }

            info!(
                "📍 Robot pose: x={:.2}, y={:.2}, yaw={:.2}°",
                odom.pose.position.x,
                odom.pose.position.y,
                2.0 * odom.pose.orientation.z.atan2(odom.pose.orientation.w).to_degrees()
            );
        })?;

        Ok(())
    }

    /// Publish velocity command
    pub async fn publish_velocity(&self, linear: Vector3, angular: Vector3) -> Result<()> {
        let twist = TwistMessage { linear, angular };
        
        // Validate command before publishing
        twist.validate()?;
        
        self.cmd_vel_pub.publish(&twist).await?;
        
        info!(
            "🎮 Velocity command: linear=[{:.2}, {:.2}, {:.2}], angular=[{:.2}, {:.2}, {:.2}]",
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z
        );
        
        Ok(())
    }

    /// Execute movement pattern
    pub async fn execute_pattern(&mut self, pattern: MovementPattern, duration_sec: f64) -> Result<()> {
        self.movement_pattern = pattern.clone();
        self.start_time = Instant::now();
        
        info!("🤖 Starting movement pattern: {:?}", pattern);
        
        let end_time = Instant::now() + Duration::from_secs_f64(duration_sec);
        
        while Instant::now() < end_time {
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
                
                MovementPattern::Square(_side_time) => {
                    // Square pattern: forward -> turn 90° -> repeat
                    let elapsed = self.start_time.elapsed().as_secs_f32();
                    let cycle_time = 8.0; // 8 seconds per side (6s forward + 2s turn)
                    let phase = (elapsed % cycle_time) / cycle_time;
                    
                    if phase < 0.75 {
                        // Move forward for 75% of cycle
                        self.publish_velocity(
                            Vector3 { x: 0.3, y: 0.0, z: 0.0 },
                            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        ).await?;
                    } else {
                        // Turn 90 degrees for 25% of cycle
                        self.publish_velocity(
                            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                            Vector3 { x: 0.0, y: 0.0, z: PI / 2.0 },
                        ).await?;
                    }
                }
            }
            
            sleep(Duration::from_millis(100)).await;
        }
        
        // Stop the robot
        self.publish_velocity(
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        ).await?;
        
        info!("✅ Movement pattern completed");
        Ok(())
    }

    /// Run demonstration sequence
    pub async fn run_demo(&mut self) -> Result<()> {
        info!("🚀 Starting turtlebot controller demo");
        
        // Setup odometry callback
        self.setup_odometry_callback().await?;
        
        // Publish some mock odometry for demonstration
        self.publish_mock_odometry().await?;
        
        sleep(Duration::from_millis(500)).await;
        
        // Execute different movement patterns
        info!("📋 Demo sequence:");
        info!("   1. Forward movement");
        info!("   2. Circular motion");  
        info!("   3. Square pattern");
        info!("   4. Spin in place");
        
        // 1. Move forward
        self.execute_pattern(MovementPattern::Forward(0.5), 3.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 2. Circle motion
        self.execute_pattern(MovementPattern::Circle(0.3, 0.5), 5.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 3. Square pattern
        self.execute_pattern(MovementPattern::Square(2.0), 10.0).await?;
        sleep(Duration::from_secs(1)).await;
        
        // 4. Spin in place
        self.execute_pattern(MovementPattern::TurnLeft(1.0), 3.0).await?;
        
        info!("🎉 Turtlebot controller demo completed!");
        Ok(())
    }

    /// Publish mock odometry data for demonstration
    async fn publish_mock_odometry(&mut self) -> Result<()> {
        // Create a mock odometry publisher for demonstration
        let odom_pub = self.node.create_publisher::<OdometryMessage>("/odom").await?;
        
        let odom = OdometryMessage {
            pose: mini_ros::types::PoseMessage {
                position: Point3D { x: 0.0, y: 0.0, z: 0.0 },
                orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                header: Header {
                    timestamp: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as i64,
                    frame_id: "odom".to_string(),
                },
            },
            twist: TwistMessage {
                linear: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            header: Header {
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as i64,
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
    // Initialize tracing
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 12: Turtlebot Controller ===");

    // Create turtlebot controller
    let mut controller = TurtlebotController::new("turtlebot_controller").await?;
    
    // Run the demonstration
    controller.run_demo().await?;
    
    // Keep node alive for a bit to see final messages
    sleep(Duration::from_secs(2)).await;

    info!("🏁 Turtlebot controller example finished");
    Ok(())
} 