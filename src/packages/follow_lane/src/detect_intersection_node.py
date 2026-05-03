#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rospy
import util
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class DetectIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Load parameters from the existing detect_lane_node.json file
        util.init_parameters("detect_lane_node", self.cbUpdateParameters)

        # Subscriptions and Publications
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbProcessImage, queue_size=1
        )
        self.pub_intersection = rospy.Publisher(
            f"/{self._vehicle_name}/detect/intersection", Bool, queue_size=1
        )

        # Debug Publisher for the GUI
        self.pub_debug_red = rospy.Publisher(
            f"/{self._vehicle_name}/debug/lane_red", CompressedImage, queue_size=1
        )

        self.red_pixel_threshold = 2000
        self.frame_counter = 0

    def cbUpdateParameters(self, parameters):
        # Update dynamically from the GUI sliders
        self.hue_red_l = parameters["red"]["hl"]["default"]
        self.hue_red_h = parameters["red"]["hh"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l = parameters["red"]["vl"]["default"]
        self.lightness_red_h = parameters["red"]["vh"]["default"]

    def cbProcessImage(self, image_msg):
        self.frame_counter += 1
        if self.frame_counter % 3 != 0:
            return

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height, width = img.shape[:2]
        # Crop to the bottom 1/3 of the image
        cropped_img = img[int(height * 2 / 3) : height, :]

        hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Upper Red Mask (Tuned via GUI) - Explicitly set dtype to uint8
        lower_red_upper = np.array(
            [self.hue_red_l, self.saturation_red_l, self.lightness_red_l],
            dtype=np.uint8,
        )
        upper_red_upper = np.array(
            [self.hue_red_h, self.saturation_red_h, self.lightness_red_h],
            dtype=np.uint8,
        )
        mask2 = cv2.inRange(hsv, lower_red_upper, upper_red_upper)

        # Lower Red Mask (Hardcoded wraparound for 0-10)
        lower_red_lower = np.array(
            [0, self.saturation_red_l, self.lightness_red_l], dtype=np.uint8
        )
        upper_red_lower = np.array(
            [10, self.saturation_red_h, self.lightness_red_h], dtype=np.uint8
        )
        mask1 = cv2.inRange(hsv, lower_red_lower, upper_red_lower)

        # Combine masks
        red_mask = cv2.bitwise_or(mask1, mask2)
        red_pixel_count = cv2.countNonZero(red_mask)

        msg = Bool()
        if red_pixel_count > self.red_pixel_threshold:
            msg.data = True
        else:
            msg.data = False

        self.pub_intersection.publish(msg)

        # Publish the debug image to the Tkinter GUI if someone is looking at it
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
    node = DetectIntersectionNode("detect_intersection_node")
    rospy.spin()
