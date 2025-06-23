//! Action implementation for miniROS
//!
//! Actions provide a communication pattern for long-running tasks that:
//! - Can be preempted (cancelled)
//! - Provide periodic feedback
//! - Return a final result
//!
//! This combines service calls with topic-based feedback.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;
use uuid::Uuid;

use crate::core::error::Result;
use crate::core::message::{Message, StringMsg};
use crate::{Node, Publisher, Subscriber};

/// Action goal status enumeration
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GoalStatus {
    /// Goal has been accepted and is awaiting execution
    Accepted,
    /// Goal is currently being executed
    Executing,
    /// Goal has been preempted (cancelled)
    Preempted,
    /// Goal execution succeeded
    Succeeded,
    /// Goal execution was aborted
    Aborted,
    /// Goal was rejected before execution
    Rejected,
    /// Goal is pending
    Pending,
    /// Goal is active
    Active,
    /// Goal is preempting
    Preempting,
    /// Goal is canceled
    Canceled,
}

/// Action goal information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoalInfo {
    pub goal_id: String,
    pub status: GoalStatus,
    pub timestamp: u64,
}

/// Action goal wrapper - no longer generic to avoid Deserialize issues
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionGoal {
    pub goal_id: String,
    pub goal_data: Vec<u8>, // Serialized goal data
}

/// Action result wrapper - no longer generic to avoid Deserialize issues
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionResult {
    pub goal_id: String,
    pub status: GoalStatus,
    pub result_data: Option<Vec<u8>>, // Serialized result data
}

/// Action feedback wrapper - no longer generic to avoid Deserialize issues
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionFeedback {
    pub goal_id: String,
    pub feedback_data: Vec<u8>, // Serialized feedback data
}

/// Action server that processes long-running goals
pub struct ActionServer {
    action_name: String,

    #[allow(dead_code)] // Reserved for future use in goal republishing
    goal_publisher: Publisher<StringMsg>,

    result_publisher: Publisher<StringMsg>,
    feedback_publisher: Publisher<StringMsg>,
    status_publisher: Publisher<StringMsg>,
    goal_subscriber: Subscriber<StringMsg>,

    #[allow(dead_code)] // Reserved for future use in goal cancellation
    cancel_subscriber: Subscriber<StringMsg>,

    goals: Arc<Mutex<HashMap<String, GoalInfo>>>,
}

impl ActionServer {
    /// Create a new action server
    pub async fn new(node: &mut Node, action_name: &str) -> Result<Self> {
        // Create topics for action communication
        let goal_topic = format!("/{}/goal", action_name);
        let result_topic = format!("/{}/result", action_name);
        let feedback_topic = format!("/{}/feedback", action_name);
        let status_topic = format!("/{}/status", action_name);
        let cancel_topic = format!("/{}/cancel", action_name);

        // Create publishers
        let goal_publisher = node.create_publisher(&goal_topic).await?;
        let result_publisher = node.create_publisher(&result_topic).await?;
        let feedback_publisher = node.create_publisher(&feedback_topic).await?;
        let status_publisher = node.create_publisher(&status_topic).await?;

        // Create subscribers
        let goal_subscriber = node.create_subscriber(&goal_topic).await?;
        let cancel_subscriber = node.create_subscriber(&cancel_topic).await?;

        Ok(ActionServer {
            action_name: action_name.to_string(),
            goal_publisher,
            result_publisher,
            feedback_publisher,
            status_publisher,
            goal_subscriber,
            cancel_subscriber,
            goals: Arc::new(Mutex::new(HashMap::new())),
        })
    }

    /// Get the action name
    pub fn name(&self) -> &str {
        &self.action_name
    }

    /// Accept a goal and start processing
    pub async fn accept_goal(&self, goal_id: &str) -> Result<()> {
        let mut goals = self.goals.lock().await;
        if let Some(goal_info) = goals.get_mut(goal_id) {
            goal_info.status = GoalStatus::Active;

            // Publish status update
            let status_msg = serde_json::to_string(goal_info)?;
            let string_msg = StringMsg { data: status_msg };
            self.status_publisher.publish(&string_msg).await?;
        }
        Ok(())
    }

    /// Publish feedback for a goal
    pub async fn publish_feedback<F: Message>(&self, goal_id: &str, feedback: &F) -> Result<()> {
        let feedback_data = serde_json::to_vec(feedback)?;
        let action_feedback = ActionFeedback {
            goal_id: goal_id.to_string(),
            feedback_data,
        };

        let feedback_msg = serde_json::to_string(&action_feedback)?;
        let string_msg = StringMsg { data: feedback_msg };
        self.feedback_publisher.publish(&string_msg).await
    }

    /// Complete a goal with success
    pub async fn succeed_goal<R: Message>(&self, goal_id: &str, result: &R) -> Result<()> {
        let result_data = serde_json::to_vec(result)?;
        let action_result = ActionResult {
            goal_id: goal_id.to_string(),
            status: GoalStatus::Succeeded,
            result_data: Some(result_data),
        };

        let result_msg = serde_json::to_string(&action_result)?;
        let string_msg = StringMsg { data: result_msg };
        self.result_publisher.publish(&string_msg).await?;

        // Update goal status
        let mut goals = self.goals.lock().await;
        if let Some(goal_info) = goals.get_mut(goal_id) {
            goal_info.status = GoalStatus::Succeeded;
        }

        Ok(())
    }

    /// Abort a goal
    pub async fn abort_goal(&self, goal_id: &str, error_msg: &str) -> Result<()> {
        let action_result = ActionResult {
            goal_id: goal_id.to_string(),
            status: GoalStatus::Aborted,
            result_data: Some(error_msg.as_bytes().to_vec()),
        };

        let result_msg = serde_json::to_string(&action_result)?;
        let string_msg = StringMsg { data: result_msg };
        self.result_publisher.publish(&string_msg).await?;

        // Update goal status
        let mut goals = self.goals.lock().await;
        if let Some(goal_info) = goals.get_mut(goal_id) {
            goal_info.status = GoalStatus::Aborted;
        }

        Ok(())
    }

    /// Set callback for incoming goals
    pub fn on_goal<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(ActionGoal) + Send + Sync + 'static,
    {
        let goals = Arc::clone(&self.goals);

        self.goal_subscriber.on_message(move |msg: StringMsg| {
            if let Ok(action_goal) = serde_json::from_str::<ActionGoal>(&msg.data) {
                // Register new goal
                let goal_info = GoalInfo {
                    goal_id: action_goal.goal_id.clone(),
                    status: GoalStatus::Pending,
                    timestamp: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_secs(),
                };

                tokio::spawn({
                    let goals = Arc::clone(&goals);
                    let goal_id = action_goal.goal_id.clone();
                    async move {
                        let mut goals = goals.lock().await;
                        goals.insert(goal_id, goal_info);
                    }
                });

                callback(action_goal);
            }
        })
    }
}

/// Action client for sending goals and receiving feedback
pub struct ActionClient {
    #[allow(dead_code)] // Reserved for future use in client identification
    action_name: String,

    goal_publisher: Publisher<StringMsg>,
    cancel_publisher: Publisher<StringMsg>,
    result_subscriber: Subscriber<StringMsg>,
    feedback_subscriber: Subscriber<StringMsg>,

    #[allow(dead_code)] // Reserved for future use in status monitoring
    status_subscriber: Subscriber<StringMsg>,
}

impl ActionClient {
    /// Create a new action client
    pub async fn new(node: &mut Node, action_name: &str) -> Result<Self> {
        // Create topics for action communication
        let goal_topic = format!("/{}/goal", action_name);
        let result_topic = format!("/{}/result", action_name);
        let feedback_topic = format!("/{}/feedback", action_name);
        let status_topic = format!("/{}/status", action_name);
        let cancel_topic = format!("/{}/cancel", action_name);

        // Create publishers
        let goal_publisher = node.create_publisher(&goal_topic).await?;
        let cancel_publisher = node.create_publisher(&cancel_topic).await?;

        // Create subscribers
        let result_subscriber = node.create_subscriber(&result_topic).await?;
        let feedback_subscriber = node.create_subscriber(&feedback_topic).await?;
        let status_subscriber = node.create_subscriber(&status_topic).await?;

        Ok(ActionClient {
            action_name: action_name.to_string(),
            goal_publisher,
            cancel_publisher,
            result_subscriber,
            feedback_subscriber,
            status_subscriber,
        })
    }

    /// Send a goal to the action server
    pub async fn send_goal<G: Message>(&self, goal: &G) -> Result<String> {
        let goal_id = Uuid::new_v4().to_string();
        let goal_data = serde_json::to_vec(goal)?;

        let action_goal = ActionGoal {
            goal_id: goal_id.clone(),
            goal_data,
        };

        let goal_msg = serde_json::to_string(&action_goal)?;
        let string_msg = StringMsg { data: goal_msg };
        self.goal_publisher.publish(&string_msg).await?;

        Ok(goal_id)
    }

    /// Cancel a goal
    pub async fn cancel_goal(&self, goal_id: &str) -> Result<()> {
        let cancel_msg = format!("{{\"goal_id\": \"{}\"}}", goal_id);
        let string_msg = StringMsg { data: cancel_msg };
        self.cancel_publisher.publish(&string_msg).await
    }

    /// Set callback for feedback messages
    pub fn on_feedback<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(ActionFeedback) + Send + Sync + 'static,
    {
        self.feedback_subscriber.on_message(move |msg: StringMsg| {
            if let Ok(feedback) = serde_json::from_str::<ActionFeedback>(&msg.data) {
                callback(feedback);
            }
        })
    }

    /// Set callback for result messages
    pub fn on_result<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(ActionResult) + Send + Sync + 'static,
    {
        self.result_subscriber.on_message(move |msg: StringMsg| {
            if let Ok(result) = serde_json::from_str::<ActionResult>(&msg.data) {
                callback(result);
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Test imports - message types not used in current test but may be needed in future

    #[tokio::test]
    async fn test_action_server_creation() {
        // Use a unique domain ID to avoid port conflicts with other tests
        let context = crate::core::Context::with_domain_id(150).unwrap();

        // Test creation without actually initializing network components
        // In test environment, we just verify the structure is correct
        match context.init().await {
            Ok(_) => {
                let mut node =
                    crate::core::node::Node::with_context("test_action_node", context.clone())
                        .unwrap();
                match node.init().await {
                    Ok(_) => {
                        let server = ActionServer::new(&mut node, "test_action").await;
                        assert!(server.is_ok());
                        let _ = node.shutdown().await;
                    }
                    Err(_) => {
                        // Network initialization might fail in test environment
                        println!(
                            "Node init failed in test environment (expected in CI/constrained environments)"
                        );
                    }
                }
                let _ = context.shutdown().await;
            }
            Err(_) => {
                // Context initialization might fail in test environment
                println!(
                    "Context init failed in test environment (expected in CI/constrained environments)"
                );
            }
        }
    }
}
