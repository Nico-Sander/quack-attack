#!/usr/bin/env python3

"""
ROS node for inferring intersection state.
Listens to the semantic geometry output from the deep learning lane detector.
"""

import os
import json
import rospy
from std_msgs.msg import Int32, String
from custom_enums import IntersectionState

class DetectIntersectionNode:
    """Lightweight logic node for intersection state."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        self.current_state = IntersectionState.NO_INTERSECTION

        self.config = self._load_config()
        self.stop_y_threshold = self.config["stop_y_threshold"]
        
        # Topics
        lane_borders_topic = f"/{self._vehicle_name}/detect/lane_borders"
        intersection_topic = f"/{self._vehicle_name}/detect/intersection"

        # Subscribers and Publishers
        self.sub_borders = rospy.Subscriber(
            lane_borders_topic, String, self._cb_process_geometry, queue_size=1
        )
        self.pub_intersection = rospy.Publisher(
            intersection_topic, Int32, queue_size=1
        )

        rospy.loginfo(f"[{node_name}] Initialized. Listening to semantic geometry.")

    def _load_config(self):
        """Loads parameters from the central config.json file."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["detect_intersection"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logwarn(f"Using default intersection config due to: {e}")
            return {
                "stop_y_threshold": 0.90 # Fallback default
            }

    def _cb_process_geometry(self, msg):
        """Parses the JSON geometry and determines state."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn("Failed to decode lane_borders JSON.")
            return

        red_detected = data.get("red_detected", False)
        red_distance_y = data.get("red_distance_y", 0.0)

        # State Machine based purely on geometry
        if not red_detected:
            self.current_state = IntersectionState.NO_INTERSECTION
        else:
            if red_distance_y >= self.stop_y_threshold:
                self.current_state = IntersectionState.AT_INTERSECTION
            else:
                self.current_state = IntersectionState.APPROACHING_INTERSECTION

        # Publish state
        state_msg = Int32(data=self.current_state.value)
        self.pub_intersection.publish(state_msg)


if __name__ == "__main__":
    try:
        node = DetectIntersectionNode("detect_intersection_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass