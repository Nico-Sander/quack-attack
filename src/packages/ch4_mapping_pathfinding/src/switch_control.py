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
from std_msgs.msg import Int32, String

import crossing
from custom_enums import (
    TURN_COMMAND_HALT, TURN_COMMAND_NONE, DriveMode, IntersectionState,
    TurnDirection
)

class SwitchControlNode:
    """Central logic node managing behavior states."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_robot')
        self.current_intersection_state = IntersectionState.NO_INTERSECTION

        self.config = self._load_config()

        # Load AprilTag DB to translate IDs into Sign Types
        node_dir = os.path.dirname(os.path.abspath(__file__))
        self.db_path = rospy.get_param("~apriltags_db_path", os.path.join(node_dir, "52DB.yaml"))
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
        # The planner's turn command. It is authoritative when fresh; the
        # sign-based random choice below is only a fallback for bring-up and
        # for the case where the mapping node is not running or has lost track
        # of where the bot is.
        self.sub_turn_command = rospy.Subscriber(
            f"/{self._vehicle_name}/plan/turn_command", Int32,
            self._cb_turn_command, queue_size=1
        )
        # Set before subscribing: the topic is latched, so a message can arrive
        # the instant the subscription is made.
        self.last_resume_seq = None
        # Bumped when the bot has been physically repositioned; see _cb_resume.
        self.sub_resume = rospy.Subscriber(
            f"/{self._vehicle_name}/plan/resume_driving", Int32,
            self._cb_resume, queue_size=1
        )
        # Lane geometry, used to notice that the intersection has been left.
        self.sub_borders = rospy.Subscriber(
            f"/{self._vehicle_name}/detect/lane_borders", String,
            self._cb_borders, queue_size=1
        )

        # Publishers (Commands)
        self.pub_mode = rospy.Publisher(f"/{self._vehicle_name}/switch/mode", Int32, queue_size=1)
        self.pub_turn = rospy.Publisher(f"/{self._vehicle_name}/switch/turn_direction", Int32, queue_size=1)

        # State Variables
        self.mode = DriveMode.LANE_FOLLOWING

        # Turn Direction Tracking
        self.planned_turn_direction = None
        self.turn_direction = TurnDirection.STRAIGHT

        # Planner command tracking
        self.commanded_turn = None
        self.commanded_halt = False
        self.commanded_turn_time = 0.0
        # How long a planner command stays trustworthy. The mapping node
        # republishes at 10 Hz, so anything older than this means it stopped
        # talking and we should drive on our own again.
        self.command_timeout = float(rospy.get_param("~command_timeout", 1.0))

        # Lane re-acquisition, the signal that ends a crossing
        self.white_detected = False
        self.yellow_detected = False

        timers = self.config["timers"]
        self.min_blind_duration = float(
            timers.get("min_blind_duration", crossing.DEFAULT_MIN_BLIND_DURATION)
        )
        self.exit_confidence_mode = self.config.get(
            "exit_confidence_mode", crossing.CONFIDENCE_BOTH
        )

        # Timers
        self.state_timer = 0.0
        self.ignore_red_line_until = 0.0
        self.turn_duration = 0.0

    def _cb_resume(self, msg):
        """
        The bot has been picked up and put down at an intersection exit.

        Whatever state the machine was parked in belongs to where the bot used
        to be. In particular it is normally STOPPED at a red line when this
        arrives, and crossing from there would drive a turn manoeuvre with no
        intersection in front of it. Start again from lane following.

        The counter is only acted on when it *changes*, and the first value seen
        is just recorded -- so a restart of this node mid-run does not trigger a
        spurious reset off the latched topic.
        """
        if self.last_resume_seq is None:
            self.last_resume_seq = msg.data
            return

        if msg.data == self.last_resume_seq:
            return

        self.last_resume_seq = msg.data
        self.mode = DriveMode.LANE_FOLLOWING
        self.planned_turn_direction = None
        # The stop line it was parked at is behind the bot now; give perception
        # a moment before it is allowed to trigger a new one.
        self.ignore_red_line_until = (
            rospy.Time.now().to_sec()
            + self.config["timers"]["red_line_ignore_duration"]
        )
        rospy.loginfo("Repositioned -- resuming in LANE_FOLLOWING")

    def _cb_borders(self, msg):
        """Tracks which lane markings are currently visible."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return

        self.white_detected = bool(data.get("white_detected", False))
        self.yellow_detected = bool(data.get("yellow_detected", False))

    def _cb_turn_command(self, msg):
        """Stores the planner's command, or clears it when it has no opinion."""
        if msg.data == TURN_COMMAND_HALT:
            # An explicit "hold position", not an absence of instruction: the
            # bot stops at the red line and stays there. Used when mapping
            # finishes, so it can be picked up and placed for the gate run.
            self.commanded_turn = None
            self.commanded_halt = True
            self.commanded_turn_time = rospy.Time.now().to_sec()
            return

        self.commanded_halt = False

        if msg.data == TURN_COMMAND_NONE:
            self.commanded_turn = None
            return

        try:
            self.commanded_turn = TurnDirection(msg.data)
            self.commanded_turn_time = rospy.Time.now().to_sec()
        except ValueError:
            rospy.logwarn(f"Invalid planner turn command: {msg.data}")
            self.commanded_turn = None

    def _command_is_fresh(self, current_time):
        return (current_time - self.commanded_turn_time) <= self.command_timeout

    def _planner_command(self, current_time):
        """The planner's command if it is still fresh, else None."""
        if self.commanded_turn is None:
            return None

        if not self._command_is_fresh(current_time):
            return None

        return self.commanded_turn

    def _halt_requested(self, current_time):
        """
        Whether the planner is actively asking the bot to hold at the line.

        Freshness matters as much as for a turn: a planner that has died must
        not leave the bot parked forever any more than it should leave a stale
        turn standing.
        """
        return self.commanded_halt and self._command_is_fresh(current_time)

    def _load_config(self):
        """Loads configuration parameters from JSON."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["switch_control"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logwarn(f"Using default logic config due to: {e}")
            return {
                "exit_confidence_mode": crossing.CONFIDENCE_BOTH,
                "timers": {
                    "stop_duration": 3.0,
                    "min_blind_duration": crossing.DEFAULT_MIN_BLIND_DURATION,
                    "red_line_ignore_duration": 2.2,
                    "turn_durations": {"LEFT": 2.5, "RIGHT": 1.4, "STRAIGHT": 2.5}
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

                    # Debug only: this is the *fallback* choice, made at every
                    # intersection and then almost always discarded in favour of
                    # the planner's command. Announcing it on the console
                    # suggests a decision that has not been taken. The turn that
                    # is actually driven is logged when the crossing starts, and
                    # a fallback that really is used logs a warning there.
                    rospy.logdebug(
                        f"Fallback turn if the planner goes quiet: "
                        f"{self.planned_turn_direction.name} ({sign_type}, ID {tag_id})"
                    )

    def run(self):
        """
        Main control loop.

        Faster than 10 Hz so state changes are not quantised so coarsely: the
        crossing now ends on seeing a lane, and at 10 Hz that decision could be
        acted on up to 100 ms late, which is real extra distance travelled
        mid-turn.
        """
        rate = rospy.Rate(self.config.get("loop_rate", 30))
        
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
                if self._halt_requested(current_time):
                    # Hold at the line indefinitely. The stop timer is reset so
                    # that when the halt is lifted the bot still waits the full
                    # stop duration before crossing, exactly as it would have.
                    self.state_timer = current_time
                    # The hold lasts as long as it takes to walk over and start
                    # the gate run, so this is a heartbeat, not news.
                    rospy.loginfo_throttle(
                        30.0, "Holding at the red line (planner)"
                    )

                elif (current_time - self.state_timer) >= self.config["timers"]["stop_duration"]:
                    self.mode = DriveMode.CROSSING_INTERSECTION
                    self.state_timer = current_time

                    # Decide the turn, in order of authority:
                    # the planner, then the sign-based random choice, then
                    # straight ahead as a last resort.
                    commanded = self._planner_command(current_time)

                    if commanded is not None:
                        self.turn_direction = commanded
                        rospy.loginfo(f"Turning {commanded.name} (planner)")
                    elif self.planned_turn_direction is not None:
                        self.turn_direction = self.planned_turn_direction
                        rospy.logwarn(
                            f"No planner command; falling back to random "
                            f"{self.planned_turn_direction.name}"
                        )
                    else:
                        self.turn_direction = TurnDirection.STRAIGHT
                        rospy.logwarn("No planner command and no sign! Defaulting to straight")


                    # Look up duration dynamically based on direction enum name
                    durations = self.config["timers"]["turn_durations"]
                    self.turn_duration = durations.get(self.turn_direction.name, 2.0)

            elif self.mode == DriveMode.CROSSING_INTERSECTION:
                # Closed loop: the crossing ends when the bot can see a lane
                # again, not when the open-loop arc has run its course. The
                # configured duration is only a timeout, so a turn that comes
                # out slightly wide still finishes in the right place.
                time_in_state = current_time - self.state_timer

                if crossing.should_exit_crossing(
                    time_in_state, self.turn_duration,
                    self.white_detected, self.yellow_detected,
                    min_blind_duration=self.min_blind_duration,
                    mode=self.exit_confidence_mode,
                ):
                    reason = crossing.exit_reason(
                        time_in_state, self.turn_duration,
                        self.white_detected, self.yellow_detected,
                        min_blind_duration=self.min_blind_duration,
                        mode=self.exit_confidence_mode,
                    )
                    # Always timing out means the exit detection is not working
                    # and the crossing is silently open loop again.
                    if reason == "timeout":
                        rospy.logwarn(
                            f"Crossing timed out after {time_in_state:.2f}s "
                            f"without re-acquiring a lane"
                        )
                    else:
                        rospy.loginfo(
                            f"Crossing done after {time_in_state:.2f}s ({reason})"
                        )

                    self.mode = DriveMode.LANE_FOLLOWING
                    self.ignore_red_line_until = current_time + self.config["timers"]["red_line_ignore_duration"]

                    # Reset planned turn
                    self.planned_turn_direction = None

            # --- Action Publishing ---
            self.pub_mode.publish(Int32(data=self.mode.value))
            self.pub_turn.publish(Int32(data=self.turn_direction.value))
            
            rate.sleep()
            
if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()