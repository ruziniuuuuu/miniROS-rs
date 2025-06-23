# action_msgs Package

Action system message definitions for miniROS - compatible with ROS2 action_msgs.

## Overview

This package provides the core message types needed for the action system in miniROS. Actions are used for long-running tasks that provide feedback during execution and can be preempted.

## Message Types

### GoalInfo
Goal identification information containing a unique ID and timestamp.

**Fields:**
- `goal_id` (string): Unique identifier for this goal
- `stamp` (int64): Timestamp when goal was created (nanoseconds since epoch)

### GoalStatus
Status information for a single goal.

**Fields:**
- `goal_info` (GoalInfo): Goal identification information
- `status` (int8): Status code (see constants below)

**Status Constants:**
- `STATUS_UNKNOWN` (0): Goal status is unknown
- `STATUS_ACCEPTED` (1): Goal has been accepted for execution
- `STATUS_EXECUTING` (2): Goal is currently being executed
- `STATUS_CANCELING` (3): Goal is in the process of being canceled
- `STATUS_SUCCEEDED` (4): Goal execution completed successfully
- `STATUS_CANCELED` (5): Goal was canceled before completion
- `STATUS_ABORTED` (6): Goal execution was aborted due to error

### GoalStatusArray
Array of goal status messages for monitoring multiple goals.

**Fields:**
- `status_list` (GoalStatus[]): Array of individual goal statuses

## Usage Example

```rust
use mini_ros::types::action_msgs::*;

// Create goal info
let goal_info = GoalInfo {
    goal_id: "navigation_goal_123".to_string(),
    stamp: std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64,
};

// Create goal status
let goal_status = GoalStatus {
    goal_info,
    status: STATUS_EXECUTING,
};

// Create status array for monitoring
let status_array = GoalStatusArray {
    status_list: vec![goal_status],
};
```

## ROS2 Compatibility

This package is fully compatible with ROS2 action_msgs, allowing seamless integration with existing ROS2 action systems. 