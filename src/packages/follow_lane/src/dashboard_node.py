#!/usr/bin/env python3

"""
ROS 1 node for aggregating multiple debug images into a single OpenCV dashboard.
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
        self.top_size = (400, 400)
        self.pad_size = 10
        self.fps = 10
        
        # Calculate the total width: 3 images + 2 padding bars
        self.total_width = (self.top_size[0] * 3) + (self.pad_size * 2)

        # Initialize placeholder images to prevent concatenation crashes on startup
        self.img_lane = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_white = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_yellow = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        
        # Red placeholder starts wide to matche total_width for vconcat safety
        self.img_red = np.zeros((self.top_size[1], self.total_width, 3), dtype=np.uint8)

        # Define visual separators (dark gray)
        self.pad_vertical = np.full((self.top_size[1], 10, 3), 50, dtype=np.uint8)
        self.pad_horizontal = np.full((10, self.total_width, 3), 50, dtype=np.uint8)

        # Topic Subscriptions
        base_topic = f"/{self._vehicle_name}/debug"
        rospy.Subscriber(
            f"{base_topic}/lane_croped", CompressedImage, self.cb_lane, queue_size=1
        )
        rospy.Subscriber(
            f"{base_topic}/lane_white", CompressedImage, self.cb_white, queue_size=1
        )
        rospy.Subscriber(
            f"{base_topic}/lane_yellow", CompressedImage, self.cb_yellow, queue_size=1
        )
        rospy.Subscriber(
            f"{base_topic}/lane_red", CompressedImage, self.cb_red, queue_size=1
        )

    def decode_image(self, msg):
        """Convert a compressed ROS image message to an OpenCV BGR array."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        decoded_img =  cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Fallback to a black image if decoding fails
        if decoded_img is None:
            rospy.logwarn_throttle(1.0, "Failed to decode incoming image.")
            return np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8) 

        return decoded_img

    def cb_lane(self, msg: CompressedImage):
        """Callback for annotated lane image."""
        self.img_lane = self.decode_image(msg)

    def cb_white(self, msg: CompressedImage):
        """Callback for white mask image."""
        self.img_white = self.decode_image(msg)

    def cb_yellow(self, msg: CompressedImage):
        """Callback for yellow mask image."""
        self.img_yellow = self.decode_image(msg)

    def cb_red(self, msg: CompressedImage):
        """Callback for red mask image."""
        self.img_red = self.decode_image(msg)

    def _get_safe_top_image(self, img: np.ndarray) -> np.ndarray:
        """Force top-row images to exact defined dimensions."""
        if img.shape[:2] != (self.top_size[1], self.top_size[0]):
            img = cv2.resize(img, self.top_size)
        return img.copy()

    def _get_scaled_bottom_image(self, img: np.ndarray) -> np.ndarray:
        """
        Scale bottom image to match the top row's total width 
        while preserving its original aspect ratio.
        """
        img_copy = img.copy()
        height, width = img_copy.shape[:2]
        
        if width == 0:
            return np.zeros((self.top_size[1], self.total_width, 3), dtype=np.uint8)

        scale = self.total_width / width
        new_height = int(height * scale)
        
        return cv2.resize(img_copy, (self.total_width, new_height))

    def run(self):
        """Main loop to stitch and display the dashboard."""
        rate = rospy.Rate(10) 
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        cv2.namedWindow("Debug Dashboard", cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            # Standardize components
            safe_lane = self._get_safe_top_image(self.img_lane)
            safe_white = self._get_safe_top_image(self.img_white)
            safe_yellow = self._get_safe_top_image(self.img_yellow)
            safe_red = self._get_scaled_bottom_image(self.img_red)

            # Apply labels
            cv2.putText(safe_lane, "Annotated Lane", (10, 30), font, 0.7, (0, 255, 0), 2)
            cv2.putText(safe_white, "White Mask", (10, 30), font, 0.7, (255, 255, 255), 2)
            cv2.putText(safe_yellow, "Yellow Mask", (10, 30), font, 0.7, (0, 255, 255), 2)
            cv2.putText(safe_red, "Red Mask (Original Ratio)", (10, 30), font, 0.7, (0, 0, 255), 2)

            # Assemble dashboard
            top_row = cv2.hconcat([
                safe_lane, self.pad_vertical, 
                safe_white, self.pad_vertical, 
                safe_yellow
            ])

            dashboard = cv2.vconcat([
                top_row, 
                self.pad_horizontal, 
                safe_red
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