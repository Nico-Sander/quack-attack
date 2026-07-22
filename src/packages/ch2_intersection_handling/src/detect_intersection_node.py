#!/usr/bin/env python3

"""
Intersection state from the stop line geometry.

A lean logic node with no image processing of its own: it reads the geometry
detect_lane publishes and derives one of three states from it -- no
intersection, one in the distance, or one reached. The boundary between the
last two is how far down the image the line lies.
"""

import json
import os

import rospy
from std_msgs.msg import Int32, String

from custom_enums import IntersectionState


class DetectIntersectionNode:
    """Turns the segmented stop line into an intersection state."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        self.current_state = IntersectionState.NO_INTERSECTION

        self.config = self._load_config()
        self.stop_y_threshold = self.config["stop_y_threshold"]

        base_topic = f"/{self._vehicle_name}"

        self.sub_borders = rospy.Subscriber(
            f"{base_topic}/detect/lane_borders",
            String,
            self._cb_process_geometry,
            queue_size=1,
        )
        self.pub_intersection = rospy.Publisher(
            f"{base_topic}/detect/intersection", Int32, queue_size=1
        )

        rospy.loginfo("detect_intersection ready: stop line at y >= %.2f",
                      self.stop_y_threshold)

    def _load_config(self):
        """Reads the detect_intersection section of the package config."""
        config_path = os.path.join(
            os.path.dirname(__file__), "../config/config.json"
        )

        try:
            with open(config_path, "r") as config_file:
                return json.load(config_file)["detect_intersection"]
        except (IOError, OSError, KeyError, ValueError) as error:
            rospy.logwarn(
                f"Using default detect_intersection config due to: {error}"
            )
            return {"stop_y_threshold": 0.95}

    def _cb_process_geometry(self, msg):
        """Classifies the stop line distance and publishes the state."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            rospy.logwarn_throttle(5.0, "Failed to decode lane_borders JSON.")
            return

        red_detected = bool(data.get("red_detected", False))
        red_distance_y = float(data.get("red_distance_y", 0.0))

        if not red_detected:
            self.current_state = IntersectionState.NO_INTERSECTION
        elif red_distance_y >= self.stop_y_threshold:
            # red_distance_y grows towards the bottom of the image, so a line
            # near the bottom edge is one the robot has driven up to.
            self.current_state = IntersectionState.AT_INTERSECTION
        else:
            self.current_state = IntersectionState.APPROACHING_INTERSECTION

        self.pub_intersection.publish(Int32(data=self.current_state.value))


if __name__ == "__main__":
    try:
        node = DetectIntersectionNode("detect_intersection_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
