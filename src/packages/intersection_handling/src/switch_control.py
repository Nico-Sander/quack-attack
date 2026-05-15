#!/usr/bin/env python3

"""
ROS node responsible for high-level decision making.
Subscribes to perception nodes and commands the driver node via DriveMode.
"""

import os
import json
import yaml
import random
import rospy
from std_msgs.msg import Float64, Int32, String
from detect_intersection import IntersectionState
from custom_enums import DriveMode, TurnDirection

class SwitchControlNode:
    """Central logic node managing behavior states."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_robot')
        self.current_intersection_state = IntersectionState.NO_INTERSECTION

        self.config = self._load_config()

        # Load AprilTag DB to translate IDs into Sign Types
        node_dir = os.path.dirname(os.path.abspath(__file__))
        self.db_path = rospy.get_param("~apriltags_db_path", os.path.join(node_dir, "apriltagsDB.yaml"))
        self.sign_db = self._load_sign_db(self.db_path)

        # Dictionary mapping sign types to allowed TurnDirections
        self.allowed_directions_map = {
            "left-T-intersect": [TurnDirection.LEFT, TurnDirection.STRAIGHT],
            "right-T-intersect": [TurnDirection.RIGHT, TurnDirection.STRAIGHT],
            "T-intersection": [TurnDirection.LEFT, TurnDirection.RIGHT],
            "4-way-intersect": [TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.STRAIGHT],
            # If it's a generic stop/yield but we are at an intersection, default to all ways
            "stop": [TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.STRAIGHT],
            "yield": [TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.STRAIGHT]
        }

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

        # Turn Direction Tracking
        self.planned_turn_direction = None
        self.turn_direction = TurnDirection.STRAIGHT        

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

    def _load_sign_db(self, path):
        """Extracts just the tag_id to traffic_sign_type mapping."""
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            # Create a dictionary of {tag_id: traffic_sign_type}
            return {int(e["tag_id"]): e.get("traffic_sign_type", "") for e in data}
        except Exception as e:
            rospy.logwarn(f"Could not load AprilTags DB in switch_control: {e}")
            return {}

    def _cb_sign(self, msg):
        """
        Looks up the received ID, determines allowed directions, 
        and locks in a random choice if we are approaching an intersection.
        """
        tag_id = msg.data
        sign_type = self.sign_db.get(tag_id, "")

        # Only process if this sign actually dictates intersection routing
        if sign_type in self.allowed_directions_map:
            # Only update our plan if we are currently driving/approaching. 
            # This prevents us from reading a sign while crossing and messing up the NEXT intersection.
            if self.mode in [DriveMode.LANE_FOLLOWING, DriveMode.APPROACHING_STOP_LINE]:
                # Only plan a turn if none has been locked in yet
                if self.planned_turn_direction is None:
                    allowed = self.allowed_directions_map[sign_type]
                    self.planned_turn_direction = random.choice(allowed)
                    
                    rospy.loginfo(f"Locked in {sign_type} (ID {tag_id}). Planned turn: {self.planned_turn_direction.name}")

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

                    # Apply planned turn direction
                    if self.planned_turn_direction is not None:
                        self.turn_direction = self.planned_turn_direction
                    else:
                        # Fallback if the sign was missed
                        self.turn_direction = TurnDirection.STRAIGHT
                        rospy.logwarn("Missed intersection sign! Defaulting to straight")
                    
                    # Look up duration dynamically based on direction enum name
                    durations = self.config["timers"]["turn_durations"]
                    self.turn_duration = durations.get(self.turn_direction.name, 2.0)

            elif self.mode == DriveMode.CROSSING_INTERSECTION:
                if (current_time - self.state_timer) >= self.turn_duration:
                    self.mode = DriveMode.LANE_FOLLOWING
                    self.ignore_red_line_until = current_time + self.config["timers"]["red_line_ignore_duration"]

                    # Reset planned durn 
                    self.planned_turn_direction = None

            # --- Action Publishing ---
            self.pub_mode.publish(Int32(data=self.mode.value))
            self.pub_turn.publish(Int32(data=self.turn_direction.value))
            
            rate.sleep()
            
if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()