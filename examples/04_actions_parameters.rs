//! Example 04: Actions and Parameters
//! 
//! This example demonstrates:
//! - Action servers and clients (long-running tasks)
//! - Parameter server and client (configuration)
//! - Domain ID isolation for multi-robot systems
//! 
//! Run with: cargo run --example 04_actions_parameters

use mini_ros::{
    prelude::*,
    message::{StringMsg, Int32Msg},
    action::{ActionGoal, ActionFeedback},
    parameter::ParameterValue,
};
use std::time::Duration;
use tokio::time::sleep;
use tracing::info;
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    
    info!("=== miniROS Example 04: Actions & Parameters ===");

    // Test domain isolation - use domain 42 for robot system isolation
    let domain_id = 42;
    let context = Context::with_domain_id(domain_id)?;
    context.init().await?;
    
    info!("🌐 Using domain ID: {} (isolated from other robots)", domain_id);

    // === Parameter System Demo ===
    info!("\n📋 Setting up Parameter Server...");
    
    let parameter_server = ParameterServer::new(context.clone()).await?;
    
    // Declare robot configuration parameters
    parameter_server.declare_parameter(
        "robot.name",
        ParameterValue::String("miniROS_Robot_42".to_string()),
        "Robot identification name",
        true, // read-only
    ).await?;
    
    parameter_server.declare_parameter(
        "robot.max_speed",
        ParameterValue::Float(2.5),
        "Maximum robot speed in m/s",
        false,
    ).await?;
    
    parameter_server.declare_parameter(
        "debug.verbose",
        ParameterValue::Bool(true),
        "Enable verbose debug output",
        false,
    ).await?;

    // Create parameter client to read configuration
    let param_client = ParameterClient::new(context.clone()).await?;
    
    // Wait for parameter server to be ready
    sleep(Duration::from_millis(500)).await;
    
    // Read robot configuration
    if let Ok(Some(robot_name)) = param_client.get_string("robot.name").await {
        info!("🤖 Robot Name: {}", robot_name);
    }
    
    if let Ok(Some(max_speed)) = param_client.get_float("robot.max_speed").await {
        info!("⚡ Max Speed: {:.1} m/s", max_speed);
    }
    
    // === Action System Demo ===
    info!("\n🎯 Setting up Action Server for long-running tasks...");

    // Create action server for "navigate_to_goal" action
    let action_server = ActionServer::new(
        "navigate_to_goal",
        context.clone(),
        |goal_request: ActionGoal<StringMsg>| -> Result<(StringMsg, Vec<StringMsg>)> {
            let destination = &goal_request.goal.data;
            let goal_id = goal_request.goal_id;
            
            info!("🚀 Navigation goal received: {} -> '{}'", goal_id, destination);
            
            // Simulate navigation with feedback
            let mut feedbacks = Vec::new();
            
            // Planning phase
            feedbacks.push(StringMsg {
                data: format!("Planning path to '{}'...", destination),
            });
            
            // Execution phases
            let waypoints = vec![
                format!("Starting navigation to '{}'", destination),
                format!("25% complete - moving towards '{}'", destination),
                format!("50% complete - halfway to '{}'", destination),
                format!("75% complete - approaching '{}'", destination),
                format!("95% complete - arriving at '{}'", destination),
            ];
            
            for waypoint in waypoints {
                feedbacks.push(StringMsg { data: waypoint });
            }
            
            // Final result
            let result = StringMsg {
                data: format!("Successfully reached destination: '{}'", destination),
            };
            
            info!("✅ Navigation completed: {}", goal_id);
            Ok((result, feedbacks))
        },
        |goal_id: Uuid| -> Result<bool> {
            info!("🛑 Cancel request for navigation goal: {}", goal_id);
            // In a real robot, this would stop the navigation
            Ok(true)
        },
    ).await?;

    info!("🔧 Action server '{}' is ready", action_server.name());

    // Create action client
    let action_client = ActionClient::<StringMsg, StringMsg, StringMsg>::new(
        "navigate_to_goal",
        context.clone(),
    ).await?;

    // Wait for action server
    info!("⏳ Waiting for action server...");
    action_client.wait_for_server(Duration::from_secs(2)).await?;

    // Set up feedback callback
    action_client.on_feedback(|feedback: ActionFeedback<StringMsg>| {
        info!("📡 Feedback: {}", feedback.feedback.data);
    }).await?;

    // Set up result callback
    action_client.on_result(|result| {
        match result.status {
            GoalStatus::Succeeded => {
                if let Some(res) = result.result {
                    info!("🎉 Goal succeeded: {}", res.data);
                }
            }
            GoalStatus::Aborted => {
                info!("❌ Goal aborted");
            }
            GoalStatus::Preempted => {
                info!("🛑 Goal preempted");
            }
            _ => {
                info!("📊 Goal status: {:?}", result.status);
            }
        }
    }).await?;

    // === Test Multiple Goals ===
    info!("\n🎯 Testing Action System with multiple goals...");
    
    let destinations = vec![
        "Kitchen",
        "Living Room", 
        "Bedroom",
        "Garage",
    ];

    for (i, destination) in destinations.iter().enumerate() {
        info!("\n--- Goal {} ---", i + 1);
        
        // Send navigation goal
        let goal = StringMsg {
            data: destination.to_string(),
        };
        
        match action_client.send_goal(goal).await {
            Ok(goal_id) => {
                info!("🎯 Sent navigation goal {}: {} -> '{}'", i + 1, goal_id, destination);
                
                // Let the action run for a bit
                sleep(Duration::from_millis(1000)).await;
                
                // Optionally cancel some goals to demonstrate cancellation
                if i == 2 { // Cancel the 3rd goal
                    info!("🛑 Cancelling goal to demonstrate cancellation...");
                    match action_client.cancel_goal(goal_id).await {
                        Ok(cancelled) => {
                            if cancelled {
                                info!("✅ Goal cancellation successful");
                            } else {
                                info!("⚠️  Goal was not active for cancellation");
                            }
                        }
                        Err(e) => {
                            info!("❌ Failed to cancel goal: {}", e);
                        }
                    }
                }
            }
            Err(e) => {
                info!("❌ Failed to send goal: {}", e);
            }
        }
        
        sleep(Duration::from_millis(500)).await;
    }

    // === Parameter Updates ===
    info!("\n📋 Testing parameter updates...");
    
    // Update max speed parameter
    param_client.set_parameter(
        "robot.max_speed", 
        ParameterValue::Float(3.0)
    ).await?;
    
    info!("⚡ Updated max speed parameter");
    
    // Try to update read-only parameter (should fail)
    match param_client.set_parameter(
        "robot.name",
        ParameterValue::String("Modified_Robot".to_string())
    ).await {
        Ok(_) => info!("⚠️  Unexpectedly succeeded in changing read-only parameter"),
        Err(_) => info!("✅ Correctly rejected read-only parameter change"),
    }
    
    // List all parameters
    let param_names = param_client.list_parameters("").await?;
    info!("📝 All parameters: {:?}", param_names);

    // === Summary ===
    info!("\n🎉 Example completed successfully!");
    info!("📊 Demonstrated features:");
    info!("   • Domain isolation (domain {})", domain_id);
    info!("   • Parameter server with type safety");
    info!("   • Action server/client for long-running tasks");
    info!("   • Goal cancellation and feedback");
    info!("   • Read-only parameter protection");
    
    info!("\n💡 Next: Try running multiple instances with different domain IDs!");
    info!("   cargo run --example 04_actions_parameters  # Domain 42");
    info!("   DOMAIN_ID=10 cargo run --example 04_actions_parameters  # Domain 10");
    
    Ok(())
} 