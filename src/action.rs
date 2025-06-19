//! Action implementation for miniROS
//! 
//! Actions provide a communication pattern for long-running tasks that:
//! - Can be preempted (cancelled)
//! - Provide periodic feedback
//! - Return a final result
//! 
//! This combines service calls with topic-based feedback.

use crate::core::Context;
use crate::error::{Result, MiniRosError};
use crate::message::Message;
use crate::service::{Service, ServiceClient};
use crate::publisher::Publisher;
use crate::subscriber::Subscriber;

use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{mpsc, Mutex};
use uuid::Uuid;

/// Action goal status enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
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
}

/// Action goal request message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionGoal<G: Message> {
    pub goal_id: Uuid,
    pub goal: G,
}

/// Action result message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionResult<R: Message> {
    pub goal_id: Uuid,
    pub status: GoalStatus,
    pub result: Option<R>,
}

/// Action feedback message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionFeedback<F: Message> {
    pub goal_id: Uuid,
    pub feedback: F,
}

/// Action server that processes long-running goals
pub struct ActionServer<G: Message, R: Message, F: Message> {
    name: String,
    context: Context,
    goal_service: Service<ActionGoal<G>, GoalStatus>,
    cancel_service: Service<Uuid, bool>,
    feedback_publisher: Publisher<ActionFeedback<F>>,
    result_publisher: Publisher<ActionResult<R>>,
    active_goals: Arc<Mutex<Vec<Uuid>>>,
    _phantom: PhantomData<(G, R, F)>,
}

impl<G: Message, R: Message, F: Message> ActionServer<G, R, F> {
    /// Create a new action server
    pub async fn new<GoalCallback, CancelCallback>(
        name: &str,
        context: Context,
        goal_callback: GoalCallback,
        cancel_callback: CancelCallback,
    ) -> Result<Self>
    where
        GoalCallback: Fn(ActionGoal<G>) -> Result<(R, Vec<F>)> + Send + Sync + 'static,
        CancelCallback: Fn(Uuid) -> Result<bool> + Send + Sync + 'static,
        G: 'static,
        R: 'static,
        F: 'static,
    {
        let active_goals = Arc::new(Mutex::new(Vec::new()));
        let active_goals_clone = active_goals.clone();

        // Create goal processing service
        let goal_service = {
            let goal_callback = Arc::new(goal_callback);
            let context_clone = context.clone();
            let name_clone = name.to_string();
            let active_goals_ref = active_goals_clone.clone();

            // Create service endpoint
            let mut node = crate::node::Node::with_context(&format!("{}_server", name), context_clone.clone())?;
            node.init().await?;

            node.create_service(
                &format!("{}/goal", name),
                move |goal_request: ActionGoal<G>| -> Result<GoalStatus> {
                    let goal_id = goal_request.goal_id;
                    tracing::info!("Action server '{}' received goal: {}", name_clone, goal_id);

                    // Add to active goals
                    let active_goals_task = active_goals_ref.clone();
                    let goal_callback_task = goal_callback.clone();
                    let context_task = context_clone.clone();
                    let name_task = name_clone.clone();

                    // Spawn goal execution task
                    tokio::spawn(async move {
                        {
                            let mut goals = active_goals_task.lock().await;
                            goals.push(goal_id);
                        }

                        // Execute goal
                        match goal_callback_task(goal_request) {
                            Ok((result, feedbacks)) => {
                                // Send feedback messages
                                for feedback in feedbacks {
                                    // In a real implementation, we'd publish feedback
                                    tracing::debug!("Action feedback for goal {}", goal_id);
                                }

                                // Send result
                                let action_result = ActionResult {
                                    goal_id,
                                    status: GoalStatus::Succeeded,
                                    result: Some(result),
                                };
                                tracing::info!("Action goal {} succeeded", goal_id);
                            }
                            Err(e) => {
                                let action_result = ActionResult {
                                    goal_id,
                                    status: GoalStatus::Aborted,
                                    result: None,
                                };
                                tracing::warn!("Action goal {} aborted: {}", goal_id, e);
                            }
                        }

                        // Remove from active goals
                        {
                            let mut goals = active_goals_task.lock().await;
                            goals.retain(|&id| id != goal_id);
                        }
                    });

                    Ok(GoalStatus::Accepted)
                }
            ).await?
        };

        // Create cancellation service
        let cancel_service = {
            let active_goals_cancel = active_goals.clone();
            let cancel_callback = Arc::new(cancel_callback);

            let mut node = crate::node::Node::with_context(&format!("{}_cancel", name), context.clone())?;
            node.init().await?;

            node.create_service(
                &format!("{}/cancel", name),
                move |goal_id: Uuid| -> Result<bool> {
                    tracing::info!("Cancel request for goal: {}", goal_id);
                    
                    // Check if goal is active
                    let active_goals_check = active_goals_cancel.clone();
                    let cancel_result = tokio::task::block_in_place(|| {
                        tokio::runtime::Handle::current().block_on(async {
                            let goals = active_goals_check.lock().await;
                            goals.contains(&goal_id)
                        })
                    });

                    if cancel_result {
                        cancel_callback(goal_id)
                    } else {
                        Ok(false) // Goal not active
                    }
                }
            ).await?
        };

        // Create feedback and result publishers
        let mut feedback_node = crate::node::Node::with_context(&format!("{}_feedback", name), context.clone())?;
        feedback_node.init().await?;
        let feedback_publisher = feedback_node.create_publisher::<ActionFeedback<F>>(&format!("{}/feedback", name)).await?;

        let mut result_node = crate::node::Node::with_context(&format!("{}_result", name), context.clone())?;
        result_node.init().await?;
        let result_publisher = result_node.create_publisher::<ActionResult<R>>(&format!("{}/result", name)).await?;

        Ok(ActionServer {
            name: name.to_string(),
            context,
            goal_service,
            cancel_service,
            feedback_publisher,
            result_publisher,
            active_goals,
            _phantom: PhantomData,
        })
    }

    /// Get the action name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get count of active goals
    pub async fn active_goal_count(&self) -> usize {
        self.active_goals.lock().await.len()
    }
}

/// Action client for sending goals and receiving feedback
pub struct ActionClient<G: Message, R: Message, F: Message> {
    name: String,
    context: Context,
    goal_client: ServiceClient<ActionGoal<G>, GoalStatus>,
    cancel_client: ServiceClient<Uuid, bool>,
    feedback_subscriber: Subscriber<ActionFeedback<F>>,
    result_subscriber: Subscriber<ActionResult<R>>,
    _phantom: PhantomData<(G, R, F)>,
}

impl<G: Message, R: Message, F: Message> ActionClient<G, R, F> {
    /// Create a new action client
    pub async fn new(name: &str, context: Context) -> Result<Self>
    where
        G: 'static,
        R: 'static,
        F: 'static,
    {
        // Create service clients for goal and cancel
        let mut goal_node = crate::node::Node::with_context(&format!("{}_goal_client", name), context.clone())?;
        goal_node.init().await?;
        let goal_client = goal_node.create_service_client::<ActionGoal<G>, GoalStatus>(&format!("{}/goal", name)).await?;

        let mut cancel_node = crate::node::Node::with_context(&format!("{}_cancel_client", name), context.clone())?;
        cancel_node.init().await?;
        let cancel_client = cancel_node.create_service_client::<Uuid, bool>(&format!("{}/cancel", name)).await?;

        // Create subscribers for feedback and result
        let mut feedback_node = crate::node::Node::with_context(&format!("{}_feedback_client", name), context.clone())?;
        feedback_node.init().await?;
        let feedback_subscriber = feedback_node.create_subscriber::<ActionFeedback<F>>(&format!("{}/feedback", name)).await?;

        let mut result_node = crate::node::Node::with_context(&format!("{}_result_client", name), context.clone())?;
        result_node.init().await?;
        let result_subscriber = result_node.create_subscriber::<ActionResult<R>>(&format!("{}/result", name)).await?;

        Ok(ActionClient {
            name: name.to_string(),
            context,
            goal_client,
            cancel_client,
            feedback_subscriber,
            result_subscriber,
            _phantom: PhantomData,
        })
    }

    /// Send a goal to the action server
    pub async fn send_goal(&self, goal: G) -> Result<Uuid> {
        let goal_id = Uuid::new_v4();
        let action_goal = ActionGoal { goal_id, goal };

        let status = self.goal_client.call(action_goal).await?;
        
        match status {
            GoalStatus::Accepted => {
                tracing::info!("Action goal {} accepted", goal_id);
                Ok(goal_id)
            }
            GoalStatus::Rejected => {
                Err(MiniRosError::Other(format!("Action goal {} rejected", goal_id)))
            }
            _ => {
                Err(MiniRosError::Other(format!("Unexpected goal status: {:?}", status)))
            }
        }
    }

    /// Cancel a goal
    pub async fn cancel_goal(&self, goal_id: Uuid) -> Result<bool> {
        self.cancel_client.call(goal_id).await
    }

    /// Wait for action server to be available
    pub async fn wait_for_server(&self, timeout: Duration) -> Result<()> {
        self.goal_client.wait_for_service(timeout).await
    }

    /// Set feedback callback
    pub async fn on_feedback<F2>(&self, callback: F2) -> Result<()>
    where
        F2: Fn(ActionFeedback<F>) + Send + 'static,
    {
        self.feedback_subscriber.on_message(callback).await
    }

    /// Set result callback
    pub async fn on_result<R2>(&self, callback: R2) -> Result<()>
    where
        R2: Fn(ActionResult<R>) + Send + 'static,
    {
        self.result_subscriber.on_message(callback).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::{StringMsg, Int32Msg};

    #[tokio::test]
    async fn test_action_server_creation() {
        let context = crate::core::Context::with_domain_id(50).unwrap();
        context.init().await.unwrap();

        let _server = ActionServer::new(
            "test_action",
            context.clone(),
            |goal: ActionGoal<StringMsg>| -> Result<(Int32Msg, Vec<StringMsg>)> {
                let result = Int32Msg { data: goal.goal.data.len() as i32 };
                let feedback = vec![StringMsg { data: "Processing...".to_string() }];
                Ok((result, feedback))
            },
            |_goal_id: Uuid| -> Result<bool> { Ok(true) },
        ).await;

        assert!(server.is_ok());
        context.shutdown().await.unwrap();
    }
} 