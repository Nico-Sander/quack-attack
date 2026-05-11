#!/usr/bin/env python3

"""
ROS 1 node for detecting red intersection lines in Duckietown.
Processes camera images to find red pixels in the lower third of the frame.
"""

import os

import cv2
import numpy as np
import rospy
import util
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from enum import Enum

class IntersectionState(Enum):
    NO_INTERSECTION = 0
    APPROACHING_INTERSECTION = 1
    AT_INTERSECTION = 2


class DetectIntersectionNode:
    """Node for detecting red intersection lines."""
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        self.frame_counter = 0
        self.red_pixel_threshold = 15000     #TODO: Needs to be fine tuned
        
        self.current_state = IntersectionState.NO_INTERSECTION
        
        # HSV thresholds initialized with defaults
        self.hue_red_l = 160
        self.hue_red_h = 179
        self.saturation_red_l = 100
        self.saturation_red_h = 255
        self.lightness_red_l = 100
        self.lightness_red_h = 255

        # Load parameters from the existing detect_lane_node.json file
        util.init_parameters("detect_lane", self.cbUpdateParameters)

        # Topics
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        intersection_topic = f"/{self._vehicle_name}/detect/intersection"
        debug_topic = f"/{self._vehicle_name}/debug/lane_red"

        # Subsrcibers and Publishers
        self.sub_image = rospy.Subscriber(
            camera_topic, CompressedImage, self.cbProcessImage, queue_size=1
        )
        self.pub_intersection = rospy.Publisher(
            intersection_topic, Int32, queue_size=1
        )
        self.pub_debug_red = rospy.Publisher(
            debug_topic, CompressedImage, queue_size=1
        )


    def cbUpdateParameters(self, parameters):
        """Update HSV thresholds dynamically from GUI parameters."""
        self.hue_red_l = parameters["red"]["hl"]["default"]
        self.hue_red_h = parameters["red"]["hh"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l = parameters["red"]["vl"]["default"]
        self.lightness_red_h = parameters["red"]["vh"]["default"]

    def cbProcessImage(self, image_msg):
        """Process incoming image, apply red masks, and publish detection."""

        # Currently only process every third frame
        self.frame_counter += 1
        if self.frame_counter % 1 != 0:
            return

        # Decode compressed image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if img is None:
            rospy.logwarn("Failed to decode image.")

        # Crop the bottom 1/3 of the image
        height = img.shape[0]
        cropped_img = img[int(height * 3.0 / 4.0) : height, :]

        # Convert to HSV colorspace
        hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Upper Red Mask (dynamically tuned)
        lower_red_upper = np.array(
            [self.hue_red_l, self.saturation_red_l, self.lightness_red_l],
            dtype=np.uint8,
        )
        upper_red_upper = np.array(
            [self.hue_red_h, self.saturation_red_h, self.lightness_red_h],
            dtype=np.uint8,
        )
        mask_upper = cv2.inRange(hsv, lower_red_upper, upper_red_upper)

        # Lower Red Mask (Hardcoded wraparound for 0-10)
        lower_red_lower = np.array(
            [0, self.saturation_red_l, self.lightness_red_l], dtype=np.uint8
        )
        upper_red_lower = np.array(
            [10, self.saturation_red_h, self.lightness_red_h], dtype=np.uint8
        )
        mask_lower = cv2.inRange(hsv, lower_red_lower, upper_red_lower)

        # Combine masks and count detected pixels
        red_mask = cv2.bitwise_or(mask_lower, mask_upper)
        red_pixel_count = cv2.countNonZero(red_mask)

        if self.current_state == IntersectionState.NO_INTERSECTION:
            if red_pixel_count > 1_000:    # Tresh 1
                self.current_state = IntersectionState.APPROACHING_INTERSECTION 

        elif self.current_state == IntersectionState.APPROACHING_INTERSECTION:
            if red_pixel_count < 1_000:    # Tresh 2
                self.current_state = IntersectionState.AT_INTERSECTION

        elif self.current_state == IntersectionState.AT_INTERSECTION:
            if red_pixel_count < 50:     # Tresh 3
                self.current_state = IntersectionState.NO_INTERSECTION
            

        msg = Int32()
        # rospy.loginfo(f"N red pixels: {red_pixel_count}")
        rospy.loginfo(f"State: {self.current_state.name}, # Red Pixels: {red_pixel_count}")
        msg.data = self.current_state.value

        self.pub_intersection.publish(msg)

        # Publish intersection status every third frame
        if self.frame_counter % 2 != 0:
            return

        # Publish the debug image if requested
        if self.pub_debug_red.get_num_connections() > 0:
            # Create a visual overlay: Highlight detected red pixels in bright green
            debug_img = cropped_img.copy()
            debug_img[red_mask > 0] = [0, 255, 0]  # BGR format: 0 Blue, 255 Green, 0 Red

            # --- NEW: Add the pixel count text to the image ---
            text = f"Red Pixels: {red_pixel_count}"
            position = (10, 50)  # (x, y) coordinates from the top-left corner
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            color = (255, 255, 255)  # White text in BGR
            thickness = 1
            
            cv2.putText(debug_img, text, position, font, font_scale, color, thickness, cv2.LINE_AA)
            # --------------------------------------------------

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
