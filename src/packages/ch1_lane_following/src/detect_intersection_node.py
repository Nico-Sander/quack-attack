#!/usr/bin/env python3

"""
Red stop line detection.

The node makes no driving decision, it only answers two questions about the
geometry detect_lane publishes:

  detect/intersection      -- is the robot at a stop line, close enough to
                              stop for it?
  detect/red_line_visible  -- is there any red line in the image at all?

The first starts a stop, the second is how switch_control knows the line it
just drove over has left the image and a new one may be acted on again.
"""

import json
import os

import rospy
from std_msgs.msg import Bool, String


class DetectIntersectionNode:
    """Turns the segmented red line into two plain yes/no signals."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

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
            f"{base_topic}/detect/intersection", Bool, queue_size=1
        )
        self.pub_red_line_visible = rospy.Publisher(
            f"{base_topic}/detect/red_line_visible", Bool, queue_size=1
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
        """Thresholds the red line distance and publishes both signals."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            rospy.logwarn_throttle(
                5.0, "Failed to decode lane_borders JSON."
            )
            return

        red_detected = bool(data.get("red_detected", False))
        red_distance_y = float(data.get("red_distance_y", 0.0))

        # red_distance_y grows towards the bottom of the image, so a line that
        # is nearly at the bottom edge is one the robot has driven up to. A
        # line further up the road is still ahead and must not stop it early.
        stop_line_reached = (red_detected
                             and red_distance_y >= self.stop_y_threshold)

        self.pub_intersection.publish(Bool(data=stop_line_reached))
        self.pub_red_line_visible.publish(Bool(data=red_detected))


if __name__ == "__main__":
    try:
        node = DetectIntersectionNode("detect_intersection_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
