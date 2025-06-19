//! Cross-Language Type System Demo
//!
//! This example demonstrates MiniROS's unified type system that works
//! across Rust and Python with automatic validation and serialization.

use mini_ros::types::{
    GLOBAL_TYPE_REGISTRY, Header, MiniRosMessage, Point3D, PointCloudMessage, PoseMessage,
    Quaternion,
};
use mini_ros::{Context, Node};
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Cross-Language Type System Demo ===");

    // Create context and initialize
    let context = Context::with_domain_id(201)?;
    context.init().await?;

    // Create nodes
    let mut publisher_node = Node::new("type_publisher")?;
    let mut subscriber_node = Node::new("type_subscriber")?;

    // Demonstrate type registry
    println!("\n1. Type Registry Demo:");
    {
        let registry = GLOBAL_TYPE_REGISTRY.lock().unwrap();
        println!("Available message types:");
        let types = [
            "String",
            "Int32",
            "Int64",
            "Float32",
            "Float64",
            "Bool",
            "Bytes",
            "PointCloud",
            "Pose",
        ];
        for type_name in types {
            if let Some(schema) = registry.get_schema(type_name) {
                println!("  - {}: {} fields", type_name, schema.fields.len());
            }
        }
    }

    // Demonstrate Pose message type with validation
    println!("\n2. Pose Message Demo:");
    let pose_pub = publisher_node
        .create_publisher::<PoseMessage>("robot/pose")
        .await?;
    let pose_sub = subscriber_node
        .create_subscriber::<PoseMessage>("robot/pose")
        .await?;

    // Set up subscriber callback
    let received_poses = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let poses_clone = received_poses.clone();

    pose_sub.on_message(move |pose: PoseMessage| {
        poses_clone.lock().unwrap().push(pose.clone());
        println!("Received pose: position=({:.2}, {:.2}, {:.2}), orientation=({:.2}, {:.2}, {:.2}, {:.2})",
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    })?;

    // Publish valid poses
    for i in 0..3 {
        let pose = PoseMessage {
            position: Point3D {
                x: i as f32,
                y: (i * 2) as f32,
                z: 0.5,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }, // Valid unit quaternion
            header: Header {
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as i64,
                frame_id: "base_link".to_string(),
            },
        };

        // Validate before publishing
        match pose.validate() {
            Ok(_) => {
                pose_pub.publish(&pose).await?;
                println!("Published valid pose {}", i);
            }
            Err(e) => {
                println!("Pose validation failed: {}", e);
            }
        }

        sleep(Duration::from_millis(200)).await;
    }

    // Demonstrate invalid pose (will fail validation)
    println!("\n3. Validation Demo:");
    let invalid_pose = PoseMessage {
        position: Point3D {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        orientation: Quaternion {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            w: 1.0,
        }, // Invalid quaternion (not normalized)
        header: Header {
            timestamp: 0,
            frame_id: "test".to_string(),
        },
    };

    match invalid_pose.validate() {
        Ok(_) => println!("Invalid pose somehow passed validation"),
        Err(e) => println!("✓ Validation correctly rejected invalid pose: {}", e),
    }

    // Demonstrate PointCloud type
    println!("\n4. PointCloud Demo:");
    let cloud_pub = publisher_node
        .create_publisher::<PointCloudMessage>("sensor/pointcloud")
        .await?;
    let cloud_sub = subscriber_node
        .create_subscriber::<PointCloudMessage>("sensor/pointcloud")
        .await?;

    let received_clouds = std::sync::Arc::new(std::sync::Mutex::new(0u32));
    let clouds_clone = received_clouds.clone();

    cloud_sub.on_message(move |cloud: PointCloudMessage| {
        let mut count = clouds_clone.lock().unwrap();
        *count += 1;
        println!(
            "Received PointCloud {} with {} points",
            *count,
            cloud.points.len()
        );
    })?;

    // Generate and publish point clouds
    for i in 0..3 {
        let mut points = Vec::new();
        let num_points = (i + 1) * 5;

        // Generate random points in a sphere
        for j in 0..num_points {
            let angle = (j as f32) * 2.0 * std::f32::consts::PI / (num_points as f32);
            let x = angle.cos() * (i as f32 + 1.0);
            let y = angle.sin() * (i as f32 + 1.0);
            let z = 0.0;
            points.push(Point3D { x, y, z });
        }

        let cloud = PointCloudMessage {
            points,
            header: Header {
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as i64,
                frame_id: "sensor_frame".to_string(),
            },
        };

        // Validate point cloud
        match cloud.validate() {
            Ok(_) => {
                cloud_pub.publish(&cloud).await?;
                println!(
                    "Published PointCloud {} with {} points",
                    i,
                    cloud.points.len()
                );
            }
            Err(e) => {
                println!("PointCloud validation failed: {}", e);
            }
        }

        sleep(Duration::from_millis(300)).await;
    }

    // Wait for messages to be processed
    sleep(Duration::from_millis(500)).await;

    // Check results
    println!("\n5. Results:");
    let pose_count = received_poses.lock().unwrap().len();
    let cloud_count = *received_clouds.lock().unwrap();
    println!(
        "Received {} poses and {} point clouds",
        pose_count, cloud_count
    );

    // Demonstrate serialization/deserialization
    println!("\n6. Serialization Demo:");
    let test_pose = PoseMessage {
        position: Point3D {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        header: Header {
            timestamp: 0,
            frame_id: "test".to_string(),
        },
    };

    let serialized = test_pose.to_bytes()?;
    println!("Serialized pose size: {} bytes", serialized.len());

    let deserialized = PoseMessage::from_bytes(&serialized)?;
    println!(
        "Deserialized pose: position=({}, {}, {})",
        deserialized.position.x, deserialized.position.y, deserialized.position.z
    );

    // Cleanup
    context.shutdown().await?;
    println!("\nType system demo completed successfully!");

    Ok(())
}
