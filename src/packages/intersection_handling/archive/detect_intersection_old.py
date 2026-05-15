#!/usr/bin/env python3

"""
ROS node for detecting red intersection lines in Duckietown.
Processes camera images to find red pixels in the lower third of the frame.
"""

import os
import json
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from custom_enums import IntersectionState

class DetectIntersectionNode:
    """Node for purely sensing red intersection lines."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        self.frame_counter = 0
        self.current_state = IntersectionState.NO_INTERSECTION

        # Load configuration
        self.config = self._load_config()
        self.hsv_cfg = self.config["hsv"]
        self.thresh_cfg = self.config["pixel_thresholds"]

        # Topics
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        intersection_topic = f"/{self._vehicle_name}/detect/intersection"
        debug_topic = f"/{self._vehicle_name}/debug/lane_red"

        # Subscribers and Publishers
        self.sub_image = rospy.Subscriber(
            camera_topic, CompressedImage, self._cb_process_image, queue_size=1
        )
        self.pub_intersection = rospy.Publisher(
            intersection_topic, Int32, queue_size=1
        )
        self.pub_debug_red = rospy.Publisher(
            debug_topic, CompressedImage, queue_size=1
        )

    def _load_config(self):
        """Loads parameters from the central config.json file."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["detect_intersection"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logwarn(f"Using default intersection config due to: {e}")
            return {
                "pixel_thresholds": {"approaching": 1000, "at_intersection": 1000, "clear": 50},
                "hsv": {"hue_l": 160, "hue_h": 179, "sat_l": 100, "sat_h": 255, "val_l": 100, "val_h": 255}
            }

    def _cb_process_image(self, image_msg):
        """Processes incoming images, applies masks, and infers state."""
        self.frame_counter += 1
        if self.frame_counter % 1 != 0:
            return

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            rospy.logwarn("Failed to decode image.")
            return

        height = img.shape[0]
        cropped_img = img[int(height * 3.0 / 4.0):height, :]
        hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Upper Red Mask
        lower_red_u = np.array([self.hsv_cfg["hue_l"], self.hsv_cfg["sat_l"], self.hsv_cfg["val_l"]], dtype=np.uint8)
        upper_red_u = np.array([self.hsv_cfg["hue_h"], self.hsv_cfg["sat_h"], self.hsv_cfg["val_h"]], dtype=np.uint8)
        mask_upper = cv2.inRange(hsv, lower_red_u, upper_red_u)

        # Lower Red Mask (Wraparound)
        lower_red_l = np.array([0, self.hsv_cfg["sat_l"], self.hsv_cfg["val_l"]], dtype=np.uint8)
        upper_red_l = np.array([10, self.hsv_cfg["sat_h"], self.hsv_cfg["val_h"]], dtype=np.uint8)
        mask_lower = cv2.inRange(hsv, lower_red_l, upper_red_l)

        red_mask = cv2.bitwise_or(mask_lower, mask_upper)
        red_pixel_count = cv2.countNonZero(red_mask)

        # State transitions based purely on thresholds
        if self.current_state == IntersectionState.NO_INTERSECTION:
            if red_pixel_count > self.thresh_cfg["approaching"]:
                self.current_state = IntersectionState.APPROACHING_INTERSECTION
        elif self.current_state == IntersectionState.APPROACHING_INTERSECTION:
            if red_pixel_count < self.thresh_cfg["at_intersection"]:
                self.current_state = IntersectionState.AT_INTERSECTION
        elif self.current_state == IntersectionState.AT_INTERSECTION:
            if red_pixel_count < self.thresh_cfg["clear"]:
                self.current_state = IntersectionState.NO_INTERSECTION

        # Publish state
        msg = Int32(data=self.current_state.value)
        self.pub_intersection.publish(msg)

        # Debug Publishing
        if self.frame_counter % 2 == 0 and self.pub_debug_red.get_num_connections() > 0:
            debug_img = cropped_img.copy()
            debug_img[red_mask > 0] = [0, 255, 0]
            
            cv2.putText(
                debug_img, f"Pixels: {red_pixel_count}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 1, cv2.LINE_AA
            )

            debug_msg = CompressedImage()
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.format = "jpeg"
            debug_msg.data = np.array(cv2.imencode(".jpg", debug_img)[1]).tobytes()
            self.pub_debug_red.publish(debug_msg)


if __name__ == "__main__":
    try:
        node = DetectIntersectionNode("detect_intersection_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass