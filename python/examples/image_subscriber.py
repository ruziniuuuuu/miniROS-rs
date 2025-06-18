#!/usr/bin/env python3
"""
Image Subscriber Example for miniROS Python API

This example demonstrates:
- Subscribing to image data
- Deserializing image messages
- Real-time visualization with OpenCV and rerun
- Image processing pipeline
"""

import mini_ros
import json
import numpy as np
import cv2
import rerun as rr
import time

class ImageSubscriber(mini_ros.Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create subscriber for image topic
        self.image_sub = self.create_subscription(
            mini_ros.String,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize rerun for visualization
        rr.init("miniROS_Image_Subscriber", spawn=True)
        
        # Statistics
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info("Image subscriber initialized")
    
    def image_callback(self, msg: mini_ros.String):
        """Process incoming image messages"""
        try:
            # Parse JSON message data
            image_info = json.loads(msg.data)
            
            # Extract image metadata
            height = image_info['height']
            width = image_info['width']
            encoding = image_info['encoding']
            
            # For this example, we'll create a visualization of the received metadata
            # In a real implementation, you'd reconstruct the image from the data
            self.visualize_image_info(image_info)
            
            # Update statistics
            current_time = time.time()
            if self.frame_count > 0:
                dt = current_time - self.last_frame_time
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt if dt > 0 else 0)
            
            self.last_frame_time = current_time
            self.frame_count += 1
            
            # Log reception
            self.get_logger().info(f'Received image frame {self.frame_count} '
                                 f'({width}x{height}, {encoding}) - FPS: {self.fps:.1f}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process image message: {e}')
    
    def visualize_image_info(self, image_info: dict):
        """Create visualization of received image metadata"""
        width = image_info['width']
        height = image_info['height']
        
        # Create synthetic visualization based on metadata
        vis_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Add frame information overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Background gradient
        for y in range(height):
            for x in range(width):
                vis_image[y, x] = [
                    int(255 * x / width),      # Red gradient
                    int(255 * y / height),     # Green gradient
                    int(128 + 127 * np.sin(self.frame_count * 0.1))  # Blue animation
                ]
        
        # Add text overlays
        cv2.putText(vis_image, f'Frame: {self.frame_count}', 
                   (10, 30), font, 1, (255, 255, 255), 2)
        cv2.putText(vis_image, f'Size: {width}x{height}', 
                   (10, 70), font, 1, (255, 255, 255), 2)
        cv2.putText(vis_image, f'FPS: {self.fps:.1f}', 
                   (10, 110), font, 1, (255, 255, 255), 2)
        cv2.putText(vis_image, f'Encoding: {image_info["encoding"]}', 
                   (10, 150), font, 1, (255, 255, 255), 2)
        
        # Add timestamp
        timestamp = image_info['header']['stamp']
        cv2.putText(vis_image, f'Time: {timestamp:.3f}', 
                   (10, 190), font, 0.7, (255, 255, 255), 2)
        
        # Log to rerun
        rr.set_time_sequence("frame", self.frame_count)
        rr.log("subscriber/visualization", rr.Image(vis_image))
        rr.log("subscriber/fps", rr.Scalar(self.fps))
        rr.log("subscriber/frame_count", rr.Scalar(self.frame_count))
        
        # Show in OpenCV window
        cv2.imshow('Image Subscriber Visualization', vis_image)
        cv2.waitKey(1)

def main():
    # Initialize miniROS
    mini_ros.init()
    
    try:
        # Create subscriber
        subscriber = ImageSubscriber()
        
        # Spin to process callbacks
        subscriber.get_logger().info("Starting image subscriber...")
        mini_ros.spin(subscriber)
        
    except KeyboardInterrupt:
        print("\nShutting down image subscriber...")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        mini_ros.shutdown()

if __name__ == '__main__':
    main() 