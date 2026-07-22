#!/usr/bin/env python3

"""
Debug dashboard: the segmentation masks next to the raw camera feed.

Purely for looking at, and off by default. Its main use is judging whether the
red mask is clean -- speckle there is what turns into a phantom stop line on
open road, and it shows up here long before it stops the robot.
"""

import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


class DashboardNode:
    """Stitches the debug image feeds into one window."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.top_size = (300, 220)
        self.pad_size = 10
        self.total_width = (self.top_size[0] * 4) + (self.pad_size * 3)

        # Placeholders, so the concatenation below cannot crash before the
        # first frame of each feed has arrived.
        self.img_lane = self._blank_top()
        self.img_white = self._blank_top()
        self.img_yellow = self._blank_top()
        self.img_red = self._blank_top()
        self.img_camera = np.zeros((480, 640, 3), dtype=np.uint8)

        self.pad_vertical = np.full(
            (self.top_size[1], self.pad_size, 3), 50, dtype=np.uint8
        )
        self.pad_horizontal = np.full(
            (self.pad_size, self.total_width, 3), 50, dtype=np.uint8
        )

        base_topic = f"/{self._vehicle_name}"

        rospy.Subscriber(f"{base_topic}/debug/lane_croped", CompressedImage,
                         self.cb_lane, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_white", CompressedImage,
                         self.cb_white, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_yellow", CompressedImage,
                         self.cb_yellow, queue_size=1)
        rospy.Subscriber(f"{base_topic}/debug/lane_red", CompressedImage,
                         self.cb_red, queue_size=1)
        rospy.Subscriber(f"{base_topic}/camera_node/image/compressed",
                         CompressedImage, self.cb_camera, queue_size=1)

    def _blank_top(self):
        """An empty panel of the top row's size."""
        return np.zeros(
            (self.top_size[1], self.top_size[0], 3), dtype=np.uint8
        )

    def decode_image(self, msg):
        """Converts a compressed ROS image message to an OpenCV BGR array."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        decoded_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if decoded_img is None:
            rospy.logwarn_throttle(1.0, "Failed to decode incoming image.")
            return self._blank_top()

        return decoded_img

    def cb_lane(self, msg):
        """Stores the annotated lane overlay."""
        self.img_lane = self.decode_image(msg)

    def cb_white(self, msg):
        """Stores the white line mask."""
        self.img_white = self.decode_image(msg)

    def cb_yellow(self, msg):
        """Stores the yellow line mask."""
        self.img_yellow = self.decode_image(msg)

    def cb_red(self, msg):
        """Stores the red line mask."""
        self.img_red = self.decode_image(msg)

    def cb_camera(self, msg):
        """Stores the raw camera frame."""
        self.img_camera = self.decode_image(msg)

    def _apply_semantic_overlay(self, camera_img, mask_white, mask_yellow,
                                mask_red):
        """Blends the segmentation masks over the raw camera feed."""
        if camera_img is None or camera_img.size == 0:
            return camera_img

        img = camera_img.copy()
        height, width = img.shape[:2]

        # The U-Net only ever sees the bottom third of the image.
        crop_height = height // 3
        start_y = height - crop_height

        roi = img[start_y:height, 0:width]
        overlay = roi.copy()

        # The masks are square; the region they describe is not.
        masks = []
        for mask in (mask_white, mask_yellow, mask_red):
            resized = cv2.resize(mask, (width, crop_height))

            if len(resized.shape) == 3:
                resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            masks.append(resized)

        overlay[masks[0] > 127] = [255, 255, 255]
        overlay[masks[1] > 127] = [0, 255, 255]
        overlay[masks[2] > 127] = [0, 0, 255]

        alpha = 0.45
        img[start_y:height, 0:width] = cv2.addWeighted(
            overlay, alpha, roi, 1 - alpha, 0
        )

        # The line above which the network sees nothing at all.
        cv2.line(img, (0, start_y), (width, start_y), (255, 0, 255), 2)

        return img

    def _get_safe_top_image(self, img):
        """Forces a top-row panel to the exact expected size."""
        if img.shape[:2] != (self.top_size[1], self.top_size[0]):
            img = cv2.resize(img, self.top_size)

        return img.copy()

    def _get_scaled_bottom_image(self, img):
        """Scales the camera image to fill the dashboard width."""
        max_bottom_height = 560

        if img is None or img.size == 0:
            return np.zeros(
                (max_bottom_height, self.total_width, 3), dtype=np.uint8
            )

        height, width = img.shape[:2]
        new_width = self.total_width
        new_height = int(height * (self.total_width / width))

        if new_height > max_bottom_height:
            new_height = max_bottom_height
            new_width = int(width * (new_height / height))

        resized = cv2.resize(img.copy(), (new_width, new_height))

        canvas = np.zeros(
            (max_bottom_height, self.total_width, 3), dtype=np.uint8
        )
        x_offset = (self.total_width - new_width) // 2
        canvas[:new_height, x_offset:x_offset + new_width] = resized

        return canvas

    def run(self):
        """Stitches and shows the dashboard, well below the camera rate."""
        rate = rospy.Rate(10)
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.namedWindow("Debug Dashboard", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Debug Dashboard", 1220, 800)

        while not rospy.is_shutdown():
            safe_lane = self._get_safe_top_image(self.img_lane)
            safe_white = self._get_safe_top_image(self.img_white)
            safe_yellow = self._get_safe_top_image(self.img_yellow)
            safe_red = self._get_safe_top_image(self.img_red)

            camera_with_overlay = self._apply_semantic_overlay(
                self.img_camera, self.img_white, self.img_yellow, self.img_red
            )
            safe_main = self._get_scaled_bottom_image(camera_with_overlay)

            cv2.putText(safe_lane, "AI Search Window", (10, 30), font, 0.7,
                        (0, 255, 0), 2)
            cv2.putText(safe_white, "White Mask", (10, 30), font, 0.7,
                        (255, 255, 255), 2)
            cv2.putText(safe_yellow, "Yellow Mask", (10, 30), font, 0.7,
                        (0, 255, 255), 2)
            cv2.putText(safe_red, "Red Mask", (10, 30), font, 0.7,
                        (0, 0, 255), 2)
            cv2.putText(safe_main, "Raw Feed + Semantic Overlay", (10, 30),
                        font, 0.7, (0, 255, 0), 2)

            top_row = cv2.hconcat([
                safe_lane, self.pad_vertical,
                safe_white, self.pad_vertical,
                safe_yellow, self.pad_vertical,
                safe_red,
            ])
            dashboard = cv2.vconcat([
                top_row, self.pad_horizontal, safe_main
            ])

            cv2.imshow("Debug Dashboard", dashboard)
            cv2.waitKey(1)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = DashboardNode("dashboard_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
