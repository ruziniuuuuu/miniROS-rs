# diagnostic_msgs Package

System diagnostic message definitions for miniROS - compatible with ROS2 diagnostic_msgs.

## Overview

This package provides message types for system diagnostics and health monitoring. It allows components to report their status, errors, and diagnostic information in a standardized format.

## Message Types

### KeyValue
Simple key-value pair for diagnostic data.

**Fields:**
- `key` (string): Parameter name
- `value` (string): Parameter value

### DiagnosticStatus
Comprehensive status information for a single system component.

**Fields:**
- `level` (uint8): Severity level (see constants below)
- `name` (string): Component name (e.g., "Motor Controller")
- `message` (string): Human-readable status message
- `hardware_id` (string): Hardware identifier
- `values` (KeyValue[]): Key-value pairs for diagnostic data

**Severity Level Constants:**
- `OK` (0): Component is functioning normally
- `WARN` (1): Component has warnings but is still functional
- `ERROR` (2): Component has errors and may not function properly
- `STALE` (3): Component data is stale (not recently updated)

### DiagnosticArray
Collection of diagnostic status messages with header information.

**Fields:**
- `header` (std_msgs/Header): Header with timestamp and frame info
- `status` (DiagnosticStatus[]): Array of diagnostic status messages

## Usage Example

```rust
use mini_ros::types::{diagnostic_msgs::*, std_msgs::Header};

// Create diagnostic key-value pairs
let temperature_kv = KeyValue {
    key: "temperature".to_string(),
    value: "42.5".to_string(),
};

let voltage_kv = KeyValue {
    key: "voltage".to_string(),
    value: "12.1".to_string(),
};

// Create diagnostic status
let motor_status = DiagnosticStatus {
    level: OK,
    name: "Motor Controller".to_string(),
    message: "Motor operating normally".to_string(),
    hardware_id: "motor_ctrl_001".to_string(),
    values: vec![temperature_kv, voltage_kv],
};

// Create diagnostic array
let diagnostic_array = DiagnosticArray {
    header: Header {
        stamp: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64,
        frame_id: "base_link".to_string(),
    },
    status: vec![motor_status],
};
```

## Common Use Cases

- **Hardware Monitoring**: Monitor temperatures, voltages, and other hardware parameters
- **Software Health**: Report software component status and errors
- **System Integration**: Aggregate diagnostic information from multiple components
- **Debugging**: Provide detailed diagnostic information for troubleshooting

## ROS2 Compatibility

This package is fully compatible with ROS2 diagnostic_msgs, enabling integration with ROS2 diagnostic tools and monitoring systems. 