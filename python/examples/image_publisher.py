#!/usr/bin/env python3
"""
Enhanced Image Publisher Example for miniROS-rs

This example demonstrates:
1. Publishing synthetic images with OpenCV
2. Real-time visualization with Rerun
3. Multiple image types (RGB, depth, segmentation)
4. Frame rate control and performance monitoring
"""

import os
import sys
import time
import numpy as np
import cv2
import rerun as rr

# Add the parent directory to the path to import mini_ros
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import mini_ros
    from mini_ros import Node
except ImportError as e:
    print(f"Error importing mini_ros: {e}")
    print("Make sure you've built the Python bindings with 'maturin develop --features python'")
    sys.exit(1)

class ImagePublisher:
    def __init__(self):
        # Initialize miniROS
        mini_ros.init()
        
        # Initialize Rerun with new API
        rec = rr.new_recording("miniROS_Image_Publisher")
        rr.connect()
        
        # Create a node
        self.node = Node("image_publisher")
        self.logger = self.node.get_logger()
        
        # Create publishers for different image types
        self.rgb_publisher = self.node.create_publisher(
            mini_ros.String, "/camera/rgb", 10
        )
        self.depth_publisher = self.node.create_publisher(
            mini_ros.String, "/camera/depth", 10
        )
        self.seg_publisher = self.node.create_publisher(
            mini_ros.String, "/camera/segmentation", 10
        )
        
        # Image generation parameters
        self.width = 640
        self.height = 480
        self.frame_count = 0
        
        # Performance monitoring
        self.start_time = time.time()
        self.last_report_time = time.time()
        self.published_frames = 0
        
        self.logger.info("Image Publisher initialized with Rerun visualization")
        self.logger.info(f"Publishing images at {self.width}x{self.height} resolution")

    def generate_rgb_image(self):
        """Generate a synthetic RGB image with moving patterns"""
        t = self.frame_count * 0.1
        
        # Create base image
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Add moving gradient
        x_grad = np.linspace(0, 1, self.width)
        y_grad = np.linspace(0, 1, self.height)
        X, Y = np.meshgrid(x_grad, y_grad)
        
        # Animated patterns
        img[:, :, 0] = (128 + 127 * np.sin(2 * np.pi * (X + t))).astype(np.uint8)  # Red
        img[:, :, 1] = (128 + 127 * np.cos(2 * np.pi * (Y + t))).astype(np.uint8)  # Green
        img[:, :, 2] = (128 + 127 * np.sin(2 * np.pi * (X + Y + t))).astype(np.uint8)  # Blue
        
        # Add some geometric shapes
        center_x, center_y = self.width // 2, self.height // 2
        radius = int(50 + 30 * np.sin(t))
        cv2.circle(img, (center_x, center_y), radius, (255, 255, 255), 2)
        
        # Add moving rectangle
        rect_x = int(center_x + 100 * np.cos(t))
        rect_y = int(center_y + 50 * np.sin(t))
        cv2.rectangle(img, (rect_x-20, rect_y-20), (rect_x+20, rect_y+20), (0, 255, 255), -1)
        
        return img

    def generate_depth_image(self):
        """Generate a synthetic depth image"""
        t = self.frame_count * 0.05
        
        # Create depth data (simulating distance in mm)
        x = np.linspace(-1, 1, self.width)
        y = np.linspace(-1, 1, self.height)
        X, Y = np.meshgrid(x, y)
        
        # Simulate a moving wave surface
        depth = 1000 + 500 * np.sin(3 * X + t) * np.cos(3 * Y + t)
        depth = depth + 200 * np.sin(5 * (X**2 + Y**2) + t)
        
        # Convert to uint16 (typical depth image format)
        depth_img = np.clip(depth, 0, 5000).astype(np.uint16)
        
        return depth_img

    def generate_segmentation_image(self):
        """Generate a synthetic segmentation image with different object classes"""
        seg = np.zeros((self.height, self.width), dtype=np.uint8)
        
        t = self.frame_count * 0.1
        
        # Background (class 0)
        seg.fill(0)
        
        # Create different objects with different class IDs
        center_x, center_y = self.width // 2, self.height // 2
        
        # Object 1: Moving circle (class 1)
        circle_x = int(center_x + 80 * np.cos(t))
        circle_y = int(center_y + 80 * np.sin(t))
        cv2.circle(seg, (circle_x, circle_y), 40, 1, -1)
        
        # Object 2: Static rectangle (class 2)
        cv2.rectangle(seg, (50, 50), (150, 150), 2, -1)
        
        # Object 3: Moving triangle (class 3)
        triangle_x = int(center_x + 60 * np.sin(t * 1.5))
        triangle_y = int(center_y + 60 * np.cos(t * 1.5))
        points = np.array([
            [triangle_x, triangle_y - 30],
            [triangle_x - 25, triangle_y + 15],
            [triangle_x + 25, triangle_y + 15]
        ], np.int32)
        cv2.fillPoly(seg, [points], 3)
        
        return seg

    def publish_frame(self):
        """Generate and publish a complete frame of images"""
        
        # Generate images
        rgb_img = self.generate_rgb_image()
        depth_img = self.generate_depth_image()
        seg_img = self.generate_segmentation_image()
        
        # Log to Rerun for visualization
        rr.set_time_sequence("frame", self.frame_count)
        
        # Log RGB image
        rr.log("camera/rgb", rr.Image(rgb_img))
        
        # Log depth image (convert to visualization format)
        rr.log("camera/depth", rr.DepthImage(depth_img))
        
        # Log segmentation image
        rr.log("camera/segmentation", rr.SegmentationImage(seg_img))
        
        # Create miniROS message (simplified - just frame info)
        rgb_msg = mini_ros.String(f"RGB frame {self.frame_count}: {self.width}x{self.height}")
        depth_msg = mini_ros.String(f"Depth frame {self.frame_count}: {self.width}x{self.height}")
        seg_msg = mini_ros.String(f"Segmentation frame {self.frame_count}: {self.width}x{self.height}")
        
        # Publish to miniROS topics
        self.rgb_publisher.publish(rgb_msg)
        self.depth_publisher.publish(depth_msg)
        self.seg_publisher.publish(seg_msg)
        
        self.frame_count += 1
        self.published_frames += 1
        
        # Performance reporting
        current_time = time.time()
        if current_time - self.last_report_time >= 5.0:  # Report every 5 seconds
            elapsed = current_time - self.start_time
            fps = self.published_frames / elapsed
            self.logger.info(f"Published {self.published_frames} frames in {elapsed:.2f}s ({fps:.2f} FPS)")
            self.last_report_time = current_time

    def run(self):
        """Main execution loop"""
        self.logger.info("Starting image publishing...")
        
        target_fps = 10  # Target frame rate
        frame_duration = 1.0 / target_fps
        
        try:
            while mini_ros.ok():
                start_time = time.time()
                
                # Publish frame
                self.publish_frame()
                
                # Sleep to maintain frame rate
                elapsed = time.time() - start_time
                sleep_time = max(0, frame_duration - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # Stop after 100 frames for demo
                if self.frame_count >= 100:
                    break
                    
        except KeyboardInterrupt:
            self.logger.info("Interrupted by user")
        except Exception as e:
            self.logger.error(f"Error during execution: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.logger.info(f"Published {self.published_frames} total frames")
        self.logger.info("Shutting down image publisher...")
        self.node.destroy_node()
        mini_ros.shutdown()

def main():
    """Main function"""
    print("=== miniROS-rs Enhanced Image Publisher ===")
    print("Publishing synthetic images with real-time visualization")
    print("Press Ctrl+C to stop")
    print()
    
    try:
        publisher = ImagePublisher()
        publisher.run()
    except Exception as e:
        print(f"Failed to start image publisher: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 