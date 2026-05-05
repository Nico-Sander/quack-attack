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
from std_msgs.msg import Bool


class DetectIntersectionNode:
    """Node for detecting red intersection lines."""
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        self.frame_counter = 0
        self.red_pixel_threshold = 2000     #TODO: Needs to be fine tuned
        
        # HSV thresholds initialized with defaults
        self.hue_red_l = 160
        self.hue_red_h = 179
        self.saturation_red_l = 100
        self.saturation_red_h = 255
        self.lightness_red_l = 100
        self.lightness_red_h = 255

        # Load parameters from the existing detect_lane_node.json file
        util.init_parameters("detect_lane_node", self.cbUpdateParameters)

        # Topics
        camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        intersection_topic = f"/{self._vehicle_name}/detect/intersection"
        debug_topic = f"/{self._vehicle_name}/debug/lane_red"

        # Subsrcibers and Publishers
        self.sub_image = rospy.Subscriber(
            camera_topic, CompressedImage, self.cbProcessImage, queue_size=1
        )
        self.pub_intersection = rospy.Publisher(
            intersection_topic, Bool, queue_size=1
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
        if self.frame_counter % 3 != 0:
            return

        # Decode compressed image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if img is None:
            rospy.logwarn("Failed to decode image.")

        # Crop the bottom 1/3 of the image
        height = img.shape[0]
        cropped_img = img[int(height * 2 / 3) : height, :]

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

        # Publishe intersection status
        msg = Bool()
        #rospy.loginfo(f"N red pixels: {red_pixel_count}")
        if red_pixel_count > self.red_pixel_threshold:
            msg.data = True
        else:
            msg.data = False
        self.pub_intersection.publish(msg)

        # Publish the debug image if requestead
        if self.pub_debug_red.get_num_connections() > 0:
            # Create a visual overlay: Highlight detected red pixels in bright green
            debug_img = cropped_img.copy()
            debug_img[red_mask > 0] = [
                0,
                255,
                0,
            ]  # BGR format: 0 Blue, 255 Green, 0 Red

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
