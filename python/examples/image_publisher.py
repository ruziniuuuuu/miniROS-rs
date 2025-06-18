#!/usr/bin/env python3
"""
Image Publisher Example for miniROS Python API

This example demonstrates:
- Publishing image data as custom messages
- Using OpenCV for image processing
- Integration with rerun for visualization
- ROS2-compatible image message structure
"""

import mini_ros
import time
import numpy as np
import cv2
import rerun as rr
from dataclasses import dataclass
from typing import List

@dataclass
class ImageMsg:
    """ROS2-compatible image message"""
    header: dict
    height: int
    width: int  
    encoding: str
    is_bigendian: bool
    step: int
    data: List[int]

class ImagePublisher(mini_ros.Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create publisher for image topic
        self.image_pub = self.create_publisher(mini_ros.String, 'camera/image_raw', 10)
        
        # Initialize rerun for visualization
        rr.init("miniROS_Image_Publisher", spawn=True)
        
        # Camera setup (use webcam if available, otherwise synthetic data)
        self.cap = cv2.VideoCapture(0)
        self.use_webcam = self.cap.isOpened()
        
        if not self.use_webcam:
            self.get_logger().info("No webcam found, using synthetic image data")
            self.cap.release()
        else:
            self.get_logger().info("Using webcam for image capture")
        
        # Image parameters
        self.frame_count = 0
        self.width = 640
        self.height = 480
        
    def create_synthetic_image(self) -> np.ndarray:
        """Create synthetic image with moving pattern"""
        # Create a colorful moving pattern
        t = time.time()
        x = np.linspace(0, 4*np.pi, self.width)
        y = np.linspace(0, 4*np.pi, self.height)
        X, Y = np.meshgrid(x, y)
        
        # Moving sine wave pattern
        pattern = np.sin(X + t) * np.cos(Y + t * 0.5)
        
        # Convert to RGB
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:,:,0] = ((pattern + 1) * 127.5).astype(np.uint8)  # Red channel
        image[:,:,1] = ((np.sin(X * 2 + t * 2) + 1) * 127.5).astype(np.uint8)  # Green
        image[:,:,2] = ((np.cos(Y * 2 + t * 1.5) + 1) * 127.5).astype(np.uint8)  # Blue
        
        return image
    
    def capture_frame(self) -> np.ndarray:
        """Capture frame from webcam or generate synthetic"""
        if self.use_webcam:
            ret, frame = self.cap.read()
            if ret:
                return cv2.resize(frame, (self.width, self.height))
            else:
                self.get_logger().warn("Failed to capture frame, switching to synthetic")
                self.use_webcam = False
                return self.create_synthetic_image()
        else:
            return self.create_synthetic_image()
    
    def process_image(self, image: np.ndarray) -> np.ndarray:
        """Apply image processing effects"""
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        
        # Add edge detection overlay
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Combine original with edge overlay
        result = blurred.copy()
        result[edges > 0] = [0, 255, 255]  # Yellow edges
        
        return result
    
    def image_to_message(self, image: np.ndarray) -> mini_ros.String:
        """Convert OpenCV image to miniROS message"""
        # Create header
        header = {
            'stamp': time.time(),
            'frame_id': 'camera_frame'
        }
        
        # Convert image to bytes
        image_bytes = image.tobytes()
        
        # Create image message structure (simplified)
        image_info = {
            'header': header,
            'height': image.shape[0],
            'width': image.shape[1],
            'encoding': 'bgr8',
            'is_bigendian': False,
            'step': image.shape[1] * 3,
            'data_size': len(image_bytes)
        }
        
        # Encode as JSON string for transport (simplified)
        import json
        message_data = json.dumps(image_info)
        
        return mini_ros.String(data=message_data)
    
    def publish_loop(self):
        """Main publishing loop"""
        rate = 10  # 10 Hz
        
        while mini_ros.ok():
            # Capture and process image
            raw_image = self.capture_frame()
            processed_image = self.process_image(raw_image)
            
            # Log to rerun for visualization
            rr.set_time_sequence("frame", self.frame_count)
            rr.log("camera/raw", rr.Image(raw_image))
            rr.log("camera/processed", rr.Image(processed_image))
            
            # Create and publish message
            msg = self.image_to_message(processed_image)
            self.image_pub.publish(msg)
            
            # Log info
            self.get_logger().info(f'Published image frame {self.frame_count} '
                                 f'({processed_image.shape[1]}x{processed_image.shape[0]})')
            
            self.frame_count += 1
            time.sleep(1.0 / rate)
    
    def shutdown(self):
        """Clean shutdown"""
        if self.use_webcam and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    # Initialize miniROS
    mini_ros.init()
    
    try:
        # Create and run publisher
        publisher = ImagePublisher()
        publisher.publish_loop()
        
    except KeyboardInterrupt:
        print("\nShutting down image publisher...")
    finally:
        # Cleanup
        if 'publisher' in locals():
            publisher.shutdown()
        mini_ros.shutdown()

if __name__ == '__main__':
    main() 