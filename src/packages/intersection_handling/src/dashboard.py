#!/usr/bin/env python3

"""
ROS 1 node for aggregating multiple debug images into a single OpenCV dashboard.
Now includes an alpha-blended overlay of semantic masks on the raw camera feed.
"""

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class DashboardNode:
    """Subscribes to debug image feeds and concatenates them into a single UI."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_duckie")

        # Layout dimensions
        self.top_size = (300, 220)
        self.pad_size = 10
        self.fps = 10

        # Total width: 4 top images + 3 padding bars
        self.total_width = (self.top_size[0] * 4) + (self.pad_size * 3)

        # Initialize placeholder images to prevent concatenation crashes on startup
        self.img_lane = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_white = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_yellow = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_red = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        
        # Main camera placeholder
        self.img_camera = np.zeros((480, 640, 3), dtype=np.uint8)

        # Define visual separators (dark gray)
        self.pad_vertical = np.full((self.top_size[1], 10, 3), 50, dtype=np.uint8)
        self.pad_horizontal = np.full((10, self.total_width, 3), 50, dtype=np.uint8)

        # Topic Subscriptions
        base_topic = f"/{self._vehicle_name}"
        
        # Debug Feeds
        rospy.Subscriber(f"{base_topic}/debug/lane_croped", CompressedImage, self.cb_lane, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_white", CompressedImage, self.cb_white, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_yellow", CompressedImage, self.cb_yellow, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_red", CompressedImage, self.cb_red, queue_size=1)
        
        # Raw Camera Feed
        rospy.Subscriber(f"{base_topic}/camera_node/image/compressed", CompressedImage, self.cb_camera, queue_size=1)

    def decode_image(self, msg):
        """Convert a compressed ROS image message to an OpenCV BGR array."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        decoded_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if decoded_img is None:
            rospy.logwarn_throttle(1.0, "Failed to decode incoming image.")
            return np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8) 

        return decoded_img

    def cb_lane(self, msg): self.img_lane = self.decode_image(msg)
    def cb_white(self, msg): self.img_white = self.decode_image(msg)
    def cb_yellow(self, msg): self.img_yellow = self.decode_image(msg)
    def cb_red(self, msg): self.img_red = self.decode_image(msg)
    def cb_camera(self, msg): self.img_camera = self.decode_image(msg)

    def _apply_semantic_overlay(self, camera_img, mask_white, mask_yellow, mask_red):
        """Overlays the AI segmentation masks translucently over the raw camera feed."""
        if camera_img is None or camera_img.size == 0:
            return camera_img

        img = camera_img.copy()
        h, w = img.shape[:2]

        # The U-Net only processes the bottom 1/3 of the image
        crop_h = h // 3
        start_y = h - crop_h

        # Extract the bottom region where the AI is looking
        roi = img[start_y:h, 0:w]
        overlay = roi.copy()

        # Resize the square AI masks to fit the rectangular ROI
        mw_resized = cv2.resize(mask_white, (w, crop_h))
        my_resized = cv2.resize(mask_yellow, (w, crop_h))
        mr_resized = cv2.resize(mask_red, (w, crop_h))

        # Convert decoded BGR masks to strict grayscale for thresholding
        if len(mw_resized.shape) == 3: mw_resized = cv2.cvtColor(mw_resized, cv2.COLOR_BGR2GRAY)
        if len(my_resized.shape) == 3: my_resized = cv2.cvtColor(my_resized, cv2.COLOR_BGR2GRAY)
        if len(mr_resized.shape) == 3: mr_resized = cv2.cvtColor(mr_resized, cv2.COLOR_BGR2GRAY)

        # Apply specific colors to the overlay where masks are active
        overlay[mw_resized > 127] = [255, 255, 255] # White
        overlay[my_resized > 127] = [0, 255, 255]   # Yellow (BGR)
        overlay[mr_resized > 127] = [0, 0, 255]     # Red (BGR)

        # Alpha blend the colored overlay back onto the original ROI
        alpha = 0.45
        img[start_y:h, 0:w] = cv2.addWeighted(overlay, alpha, roi, 1 - alpha, 0)

        # Draw a subtle line showing the AI's "horizon"
        cv2.line(img, (0, start_y), (w, start_y), (255, 0, 255), 2)

        return img

    def _get_safe_top_image(self, img):
        """Force top-row images to exact defined dimensions."""
        if img.shape[:2] != (self.top_size[1], self.top_size[0]):
            img = cv2.resize(img, self.top_size)
        return img.copy()

    def _get_scaled_bottom_image(self, img):
        """Scale the main camera image to fill the dashboard width."""
        if img is None or img.size == 0:
            return np.zeros((520, self.total_width, 3), dtype=np.uint8)

        img_copy = img.copy()
        height, width = img_copy.shape[:2]

        max_bottom_height = 560
        scale = self.total_width / width
        new_width = self.total_width
        new_height = int(height * scale)

        if new_height > max_bottom_height:
            new_height = max_bottom_height
            new_width = int(width * (new_height / height))

        resized = cv2.resize(img_copy, (new_width, new_height))

        canvas = np.zeros((max_bottom_height, self.total_width, 3), dtype=np.uint8)
        x_offset = (self.total_width - new_width) // 2
        canvas[:new_height, x_offset:x_offset + new_width] = resized

        return canvas

    def run(self):
        """Main loop to stitch and display the dashboard."""
        rate = rospy.Rate(10) 
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        cv2.namedWindow("Debug Dashboard", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Debug Dashboard", 1220, 800)

        while not rospy.is_shutdown():
            # Standardize Top Row Components
            safe_lane = self._get_safe_top_image(self.img_lane)
            safe_white = self._get_safe_top_image(self.img_white)
            safe_yellow = self._get_safe_top_image(self.img_yellow)
            safe_red = self._get_safe_top_image(self.img_red)

            # Apply Semantic Overlay to Raw Camera
            camera_with_overlay = self._apply_semantic_overlay(
                self.img_camera, self.img_white, self.img_yellow, self.img_red
            )
            safe_main = self._get_scaled_bottom_image(camera_with_overlay)

            # Apply labels
            cv2.putText(safe_lane, "AI Search Window", (10, 30), font, 0.7, (0, 255, 0), 2)
            cv2.putText(safe_white, "White Mask", (10, 30), font, 0.7, (255, 255, 255), 2)
            cv2.putText(safe_yellow, "Yellow Mask", (10, 30), font, 0.7, (0, 255, 255), 2)
            cv2.putText(safe_red, "Red Mask", (10, 30), font, 0.7, (0, 0, 255), 2)
            cv2.putText(safe_main, "Raw Feed + Semantic Overlay", (10, 30), font, 0.7, (0, 255, 0), 2)

            # Assemble dashboard
            top_row = cv2.hconcat([
                safe_lane, self.pad_vertical,
                safe_white, self.pad_vertical,
                safe_yellow, self.pad_vertical,
                safe_red,
            ])

            dashboard = cv2.vconcat([
                top_row,
                self.pad_horizontal,
                safe_main
            ])

            # Render
            cv2.imshow("Debug Dashboard", dashboard)
            cv2.waitKey(1)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        node = DashboardNode("debug_dashboard_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()