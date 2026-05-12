#!/usr/bin/env python3

"""
ROS node responsible for high-level decision making.
Subscribes to perception nodes and commands the driver node via DriveMode.
"""

import os
import json
import rospy
from std_msgs.msg import Float64, Int32
from detect_intersection import IntersectionState
from custom_enums import DriveMode, TurnDirection

class SwitchControlNode:
    """Central logic node managing behavior states."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_robot')
        self.current_intersection_state = IntersectionState.NO_INTERSECTION

        self.config = self._load_config()

        # Subscribers (Perception)
        self.sub_intersection = rospy.Subscriber(
            f"/{self._vehicle_name}/detect/intersection", Int32, self._cb_intersection, queue_size=1
        )
        self.sub_sign = rospy.Subscriber(
            f"/{self._vehicle_name}/detect/sign", Int32, self._cb_sign, queue_size=1
        )

        # Publishers (Commands)
        self.pub_mode = rospy.Publisher(f"/{self._vehicle_name}/switch/mode", Int32, queue_size=1)
        self.pub_turn = rospy.Publisher(f"/{self._vehicle_name}/switch/turn_direction", Int32, queue_size=1)

        # State Variables
        self.mode = DriveMode.LANE_FOLLOWING
        self.turn_direction = TurnDirection.RIGHT
        
        # Timers
        self.state_timer = 0.0
        self.ignore_red_line_until = 0.0
        self.turn_duration = 0.0

    def _load_config(self):
        """Loads configuration parameters from JSON."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["switch_control"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logwarn(f"Using default logic config due to: {e}")
            return {
                "timers": {
                    "stop_duration": 3.0, 
                    "red_line_ignore_duration": 4.0, 
                    "turn_durations": {"LEFT": 2.0, "RIGHT": 1.2, "STRAIGHT": 2.0}
                }
            }

    def _cb_intersection(self, msg):
        try:
            self.current_intersection_state = IntersectionState(msg.data)
        except ValueError:
            rospy.logwarn(f"Invalid intersection state: {msg.data}")

    def _cb_sign(self, msg):
        # TODO: Implement sign logic. Defaulting to left turn for testing.
        self.turn_direction = TurnDirection.LEFT

    def run(self):
        """Main control loop."""
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # --- State Machine Transitions ---
            if self.mode == DriveMode.LANE_FOLLOWING:
                # Obey the cooldown timer before looking for new lines
                if current_time >= self.ignore_red_line_until:
                    if self.current_intersection_state == IntersectionState.APPROACHING_INTERSECTION:
                        self.mode = DriveMode.APPROACHING_STOP_LINE
                    elif self.current_intersection_state == IntersectionState.AT_INTERSECTION:
                        self.mode = DriveMode.STOPPED
                        self.state_timer = current_time

            elif self.mode == DriveMode.APPROACHING_STOP_LINE:
                if self.current_intersection_state == IntersectionState.AT_INTERSECTION:
                    self.mode = DriveMode.STOPPED
                    self.state_timer = current_time

            elif self.mode == DriveMode.STOPPED:
                if (current_time - self.state_timer) >= self.config["timers"]["stop_duration"]:
                    self.mode = DriveMode.CROSSING_INTERSECTION
                    self.state_timer = current_time
                    
                    # Look up duration dynamically based on direction enum name
                    durations = self.config["timers"]["turn_durations"]
                    self.turn_duration = durations.get(self.turn_direction.name, 2.0)

            elif self.mode == DriveMode.CROSSING_INTERSECTION:
                if (current_time - self.state_timer) >= self.turn_duration:
                    self.mode = DriveMode.LANE_FOLLOWING
                    self.ignore_red_line_until = current_time + self.config["timers"]["red_line_ignore_duration"]

            # --- Action Publishing ---
            self.pub_mode.publish(Int32(data=self.mode.value))
            self.pub_turn.publish(Int32(data=self.turn_direction.value))
            
            rate.sleep()
            
if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()