//! Example 12: Turtlebot Controller
//!
//! This example demonstrates essential turtlebot control patterns:
//! - Publishing velocity commands to control robot movement
//! - Subscribing to odometry data for position feedback
//! - Implementing standard movement patterns (forward, circle, square)
//! - ROS2-compatible message types for robotics applications
//!
//! Following miniROS philosophy: Essential robotics functionality with minimal complexity
//!
//! Run with: cargo run --example 12_turtlebot_controller

use mini_ros::prelude::*;
use mini_ros::types::{geometry_msgs, nav_msgs, std_msgs, MiniRosMessage};
use std::f32::consts::PI;
use std::time::Duration;
use tokio::time::{Instant, sleep};
use tracing::info;

/// Robot movement patterns for autonomous control
#[derive(Debug, Clone)]
pub enum MovementPattern {
    Stop,
    Forward(f32),     // Forward velocity in m/s
    Backward(f32),    // Backward velocity in m/s
    TurnLeft(f32),    // Angular velocity in rad/s
    TurnRight(f32),   // Angular velocity in rad/s
    Circle(f32, f32), // (linear_vel, angular_vel)
    Square(f32),      // Side length completion time
}

/// Turtlebot controller state and communication interfaces
pub struct TurtlebotController {
    node: Node,
    cmd_vel_pub: Publisher<geometry_msgs::Twist>,
    odom_sub: Subscriber<nav_msgs::Odometry>,
    movement_pattern: MovementPattern,
    start_time: Instant,
}

impl TurtlebotController {
    /// Create a new turtlebot controller with minimal setup
    pub async fn new(node_name: &str) -> Result<Self> {
        let mut node = Node::new(node_name)?;
        node.init().await?;

        info!("🤖 Creating turtlebot controller: {}", node_name);

        // Create velocity command publisher using standard ROS2 message type
        let cmd_vel_pub = node.create_publisher::<geometry_msgs::Twist>("/cmd_vel").await?;

        // Create odometry subscriber using standard navigation message
        let odom_sub = node.create_subscriber::<nav_msgs::Odometry>("/odom").await?;

        info!("📤 Created velocity publisher on /cmd_vel");
        info!("📥 Created odometry subscriber on /odom");

        Ok(Self {
            node,
            cmd_vel_pub,
            odom_sub,
            movement_pattern: MovementPattern::Stop,
            start_time: Instant::now(),
        })
    }

    /// Set up odometry callback for position feedback
    pub async fn setup_odometry_callback(&mut self) -> Result<()> {
        info!("🔗 Setting up odometry feedback");

        self.odom_sub.on_message(move |odom: nav_msgs::Odometry| {
            // Extract position and orientation from odometry
            let pos = &odom.pose.pose.position;
            let orient = &odom.pose.pose.orientation;
            
            // Convert quaternion to yaw angle (simplified)
            let yaw = 2.0 * orient.z.atan2(orient.w);
            
            info!(
                "📍 Robot pose: x={:.2}, y={:.2}, yaw={:.1}°",
                pos.x,
                pos.y,
                yaw.to_degrees()
            );
        })?;

        Ok(())
    }

    /// Publish velocity command with validation
    pub async fn publish_velocity(&self, linear: geometry_msgs::Vector3, angular: geometry_msgs::Vector3) -> Result<()> {
        let twist = geometry_msgs::Twist { linear, angular };

        // Validate command before publishing (built-in validation)
        twist.validate()?;

        self.cmd_vel_pub.publish(&twist).await?;

        info!(
            "🎮 Velocity: linear=[{:.2}, {:.2}, {:.2}], angular=[{:.2}, {:.2}, {:.2}]",
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z
        );

        Ok(())
    }

    /// Execute a movement pattern for specified duration
    pub async fn execute_pattern(
        &mut self,
        pattern: MovementPattern,
        duration_sec: f64,
    ) -> Result<()> {
        self.movement_pattern = pattern.clone();
        self.start_time = Instant::now();

        info!("🤖 Executing movement pattern: {:?} for {:.1}s", pattern, duration_sec);

        let end_time = Instant::now() + Duration::from_secs_f64(duration_sec);

        while Instant::now() < end_time {
            match &self.movement_pattern {
                MovementPattern::Stop => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }

                MovementPattern::Forward(speed) => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: *speed as f64, y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }

                MovementPattern::Backward(speed) => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: -(*speed as f64), y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    ).await?;
                }

                MovementPattern::TurnLeft(angular_speed) => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: *angular_speed as f64 },
                    ).await?;
                }

                MovementPattern::TurnRight(angular_speed) => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: -(*angular_speed as f64) },
                    ).await?;
                }

                MovementPattern::Circle(linear_vel, angular_vel) => {
                    self.publish_velocity(
                        geometry_msgs::Vector3 { x: *linear_vel as f64, y: 0.0, z: 0.0 },
                        geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: *angular_vel as f64 },
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
                            geometry_msgs::Vector3 { x: 0.3, y: 0.0, z: 0.0 },
                            geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                        ).await?;
                    } else {
                        // Turn 90 degrees for 25% of cycle
                        self.publish_velocity(
                            geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                            geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: (PI / 2.0) as f64 },
                        ).await?;
                    }
                }
            }

            sleep(Duration::from_millis(100)).await;
        }

        // Stop the robot after pattern completion
        self.publish_velocity(
            geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        ).await?;

        info!("✅ Movement pattern completed");
        Ok(())
    }

    /// Run comprehensive turtlebot demonstration
    pub async fn run_demo(&mut self) -> Result<()> {
        info!("🚀 Starting turtlebot controller demo");

        // Setup odometry callback for position feedback
        self.setup_odometry_callback().await?;

        // Publish mock odometry for demonstration
        self.publish_mock_odometry().await?;

        sleep(Duration::from_millis(500)).await;

        // Execute different movement patterns demonstrating core functionality
        info!("📋 Demo sequence - demonstrating core turtlebot control:");
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
        info!("📡 Publishing mock odometry data");

        // Create a mock odometry publisher for demonstration
        let odom_pub = self.node.create_publisher::<nav_msgs::Odometry>("/odom").await?;

        // Create properly structured odometry message
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as i64;

        let odom = nav_msgs::Odometry {
            header: std_msgs::Header {
                stamp: current_time,
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            pose: geometry_msgs::PoseWithCovariance {
                pose: geometry_msgs::Pose {
                    position: geometry_msgs::Point {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: geometry_msgs::Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
            twist: geometry_msgs::TwistWithCovariance {
                twist: geometry_msgs::Twist {
                    linear: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                    angular: geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 },
                },
                covariance: vec![0.0; 36], // 6x6 covariance matrix
            },
        };

        odom_pub.publish(&odom).await?;
        info!("✅ Published initial odometry data");

        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging for better debugging
    tracing_subscriber::fmt::init();

    info!("=== miniROS-rs Example 12: Turtlebot Controller ===");
    info!("🎯 Demonstrating essential robotics control patterns");

    // Create turtlebot controller with minimal setup
    let mut controller = TurtlebotController::new("turtlebot_controller").await?;

    // Run the comprehensive demonstration
    controller.run_demo().await?;

    // Brief pause to see final messages
    sleep(Duration::from_secs(2)).await;

    info!("🏁 Turtlebot controller example completed successfully!");
    info!("💡 Key miniROS principles demonstrated:");
    info!("   • Essential robotics functionality - velocity control & odometry");
    info!("   • Minimal complexity - focused movement patterns");
    info!("   • ROS2 compatibility - standard message types");
    info!("   • Performance focus - efficient real-time control");
    
    Ok(())
}
