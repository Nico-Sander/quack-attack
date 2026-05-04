#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class DashboardNode:
    def __init__(self, node_name):
        # Initialize the ROS node
        rospy.init_node(node_name)

        # Get vehicle name from environment variables
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_duckie")

        # Define standard size for the top row cropped images
        self.top_size = (400, 400) # (width, height)
        
        # Calculate the total width of the top row: 3 images + 2 padding bars (10px each)
        self.total_width = (self.top_size[0] * 3) + 20

        # Initialize placeholder images (prevents crashes before first messages arrive)
        self.img_lane = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_white = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        self.img_yellow = np.zeros((self.top_size[1], self.top_size[0], 3), dtype=np.uint8)
        
        # Red placeholder needs to match the total width so vconcat doesn't crash on startup
        # We start with a generic wide aspect ratio (e.g., 1220 x 400)
        self.img_red = np.zeros((400, self.total_width, 3), dtype=np.uint8)

        # Define visual separators (dark gray)
        self.pad_vertical = np.full((self.top_size[1], 10, 3), 50, dtype=np.uint8)
        self.pad_horizontal = np.full((10, self.total_width, 3), 50, dtype=np.uint8)

        # Set up Subscribers
        rospy.Subscriber(
            f"/{self._vehicle_name}/debug/lane_croped", CompressedImage, self.cb_lane, queue_size=1
        )
        rospy.Subscriber(
            f"/{self._vehicle_name}/debug/lane_white", CompressedImage, self.cb_white, queue_size=1
        )
        rospy.Subscriber(
            f"/{self._vehicle_name}/debug/lane_yellow", CompressedImage, self.cb_yellow, queue_size=1
        )
        rospy.Subscriber(
            f"/{self._vehicle_name}/debug/lane_red", CompressedImage, self.cb_red, queue_size=1
        )

    def decode_image(self, msg):
        """Helper function to convert compressed ROS image back to OpenCV BGR"""
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def cb_lane(self, msg):
        self.img_lane = self.decode_image(msg)

    def cb_white(self, msg):
        self.img_white = self.decode_image(msg)

    def cb_yellow(self, msg):
        self.img_yellow = self.decode_image(msg)

    def cb_red(self, msg):
        self.img_red = self.decode_image(msg)

    def _get_safe_top_image(self, img):
        """Forces top-row images to exactly 400x400 to prevent hconcat crashes."""
        if img.shape[:2] != (self.top_size[1], self.top_size[0]):
            img = cv2.resize(img, self.top_size)
        return img.copy()

    def _get_scaled_bottom_image(self, img):
        """
        Scales the bottom image to exactly match the total width of the top row
        while strictly preserving its original aspect ratio.
        """
        img_copy = img.copy()
        h, w = img_copy.shape[:2]
        
        # Avoid division by zero if an empty array somehow slips through
        if w == 0:
            return np.zeros((400, self.total_width, 3), dtype=np.uint8)

        # Calculate scaling factor based on required width
        scale = self.total_width / w
        new_height = int(h * scale)
        
        return cv2.resize(img_copy, (self.total_width, new_height))

    def run(self):
        # Run the UI refresh loop at 10 Hz
        rate = rospy.Rate(10) 
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # --- NEW: Explicitly create a resizable window ---
        cv2.namedWindow("Debug Dashboard", cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            # 1. Safely grab and format all images
            safe_lane = self._get_safe_top_image(self.img_lane)
            safe_white = self._get_safe_top_image(self.img_white)
            safe_yellow = self._get_safe_top_image(self.img_yellow)
            
            # Scale red to span the full width of the dashboard while keeping aspect ratio
            safe_red = self._get_scaled_bottom_image(self.img_red)

            # 2. Draw dynamic labels
            cv2.putText(safe_lane, "Annotated Lane", (10, 30), font, 0.7, (0, 255, 0), 2)
            cv2.putText(safe_white, "White Mask", (10, 30), font, 0.7, (255, 255, 255), 2)
            cv2.putText(safe_yellow, "Yellow Mask", (10, 30), font, 0.7, (0, 255, 255), 2)
            cv2.putText(safe_red, "Red Mask (Original Ratio)", (10, 30), font, 0.7, (0, 0, 255), 2)

            # 3. Stitch the Top Row
            top_row = cv2.hconcat([
                safe_lane, self.pad_vertical, 
                safe_white, self.pad_vertical, 
                safe_yellow
            ])

            # 4. Stitch Top Row, Horizontal Padding, and Bottom Row together
            dashboard = cv2.vconcat([
                top_row, 
                self.pad_horizontal, 
                safe_red
            ])

            # 5. Display the dashboard inside the resizable window
            cv2.imshow("Debug Dashboard", dashboard)
            cv2.waitKey(1)
            
            rate.sleep()

if __name__ == "__main__":
    node = DashboardNode("debug_dashboard_node")
    node.run()