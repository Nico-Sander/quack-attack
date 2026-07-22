#!/usr/bin/env python3

"""
The state machine of the package, and the choice of turn.

It translates the tag ID into a sign type via the database, looks up which
directions that kind of intersection allows, and picks one of them at random.
The choice is locked in as soon as the robot approaches an intersection and
held until the intersection has been passed, so a second sign further down the
road cannot change it after the fact. With no sign, straight ahead is the
fallback.
"""

import json
import os
import random

import rospy
import yaml
from std_msgs.msg import Int32, String

import crossing
from custom_enums import DriveMode, IntersectionState, TurnDirection

# Which directions each kind of intersection allows. The sign says what the
# intersection looks like; this is what that means for where the robot may go.
ALLOWED_DIRECTIONS = {
    "left-T-intersect": [TurnDirection.LEFT, TurnDirection.STRAIGHT],
    "right-T-intersect": [TurnDirection.RIGHT, TurnDirection.STRAIGHT],
    "T-intersection": [TurnDirection.LEFT, TurnDirection.RIGHT],
    "4-way-intersect": [TurnDirection.LEFT, TurnDirection.RIGHT,
                        TurnDirection.STRAIGHT],
    # Stop and yield say nothing about the layout, so at an intersection they
    # rule nothing out.
    "stop": [TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.STRAIGHT],
    "yield": [TurnDirection.LEFT, TurnDirection.RIGHT, TurnDirection.STRAIGHT],
}

# Modes in which reading a sign is still about the intersection ahead. Once
# stopped or crossing, a newly visible sign belongs to the next intersection
# and must not overwrite the choice being driven right now.
PLANNING_MODES = (DriveMode.LANE_FOLLOWING, DriveMode.APPROACHING_STOP_LINE)


class SwitchControlNode:
    """Decides the drive mode and the turn direction, and publishes both."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.config = self._load_config()

        node_dir = os.path.dirname(os.path.abspath(__file__))
        self.db_path = rospy.get_param(
            "~apriltags_db_path", os.path.join(node_dir, "52DB.yaml")
        )
        self.sign_db = self._load_sign_db(self.db_path)

        # Forces every turn, bypassing the sign. For checking the manoeuvre and
        # tuning its durations without hunting for the right intersection.
        self.force_turn = self._load_force_turn()

        self.mode = DriveMode.LANE_FOLLOWING
        self.current_intersection_state = IntersectionState.NO_INTERSECTION

        # The direction chosen for the intersection ahead, held until it has
        # been crossed. None means nothing has been decided yet.
        self.planned_turn_direction = None
        self.turn_direction = TurnDirection.STRAIGHT
        self.turn_duration = 0.0

        # Lane re-acquisition: the signal that ends a crossing.
        self.white_detected = False
        self.yellow_detected = False

        timers = self.config["timers"]
        self.stop_duration = float(timers["stop_duration"])
        self.red_line_ignore_duration = float(
            timers["red_line_ignore_duration"]
        )
        self.min_blind_duration = float(
            timers.get("min_blind_duration",
                       crossing.DEFAULT_MIN_BLIND_DURATION)
        )
        self.exit_confidence_mode = self.config.get(
            "exit_confidence_mode", crossing.CONFIDENCE_BOTH
        )

        self.state_timer = 0.0
        self.ignore_red_line_until = 0.0

        base_topic = f"/{self._vehicle_name}"

        self.sub_intersection = rospy.Subscriber(
            f"{base_topic}/detect/intersection", Int32,
            self._cb_intersection, queue_size=1
        )
        self.sub_sign = rospy.Subscriber(
            f"{base_topic}/detect/sign", Int32, self._cb_sign, queue_size=1
        )
        self.sub_borders = rospy.Subscriber(
            f"{base_topic}/detect/lane_borders", String,
            self._cb_borders, queue_size=1
        )

        self.pub_mode = rospy.Publisher(
            f"{base_topic}/switch/mode", Int32, queue_size=1
        )
        self.pub_turn = rospy.Publisher(
            f"{base_topic}/switch/turn_direction", Int32, queue_size=1
        )

        rospy.loginfo("switch_control ready: %d signs known, stop %.1fs%s",
                      len(self.sign_db), self.stop_duration,
                      f", forcing {self.force_turn.name}"
                      if self.force_turn else "")

    def _load_config(self):
        """Reads the switch_control section of the package config."""
        config_path = os.path.join(
            os.path.dirname(__file__), "../config/config.json"
        )

        try:
            with open(config_path, "r") as config_file:
                return json.load(config_file)["switch_control"]
        except (IOError, OSError, KeyError, ValueError) as error:
            rospy.logwarn(f"Using default switch_control config due to: "
                          f"{error}")
            return {
                "exit_confidence_mode": crossing.CONFIDENCE_BOTH,
                "timers": {
                    "stop_duration": 3.0,
                    "min_blind_duration": crossing.DEFAULT_MIN_BLIND_DURATION,
                    "red_line_ignore_duration": 2.2,
                    "turn_durations": {"LEFT": 2.5, "RIGHT": 1.4,
                                       "STRAIGHT": 2.5},
                },
                "loop_rate": 30,
            }

    def _load_sign_db(self, path):
        """Reads the tag ID to sign type mapping."""
        try:
            with open(path, "r") as db_file:
                data = yaml.safe_load(db_file)

            return {int(entry["tag_id"]):
                    entry.get("traffic_sign_type") or ""
                    for entry in data}
        except (IOError, OSError, ValueError, KeyError, TypeError) as error:
            # Not fatal, but every intersection will now fall back to straight
            # ahead, so it needs to be loud.
            rospy.logerr(f"Could not load the AprilTag database: {error}")
            return {}

    def _load_force_turn(self):
        """Reads the forced turn direction, or None for normal operation."""
        name = str(rospy.get_param("~force_turn", "NONE")).upper()

        if name in ("", "NONE"):
            return None

        try:
            return TurnDirection[name]
        except KeyError:
            rospy.logwarn(f"Ignoring unknown force_turn value: {name}")
            return None

    def _cb_intersection(self, msg):
        """Tracks what perception makes of the stop line."""
        try:
            self.current_intersection_state = IntersectionState(msg.data)
        except ValueError:
            rospy.logwarn_throttle(
                5.0, f"Invalid intersection state: {msg.data}"
            )

    def _cb_borders(self, msg):
        """Tracks which lane markings are currently visible."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return

        self.white_detected = bool(data.get("white_detected", False))
        self.yellow_detected = bool(data.get("yellow_detected", False))

    def _cb_sign(self, msg):
        """
        Turns a sign into a locked-in choice of direction.

        Only while driving towards an intersection, and only once: the first
        sign read decides, and the choice stands until the intersection has
        been crossed. Without that lock a second sign coming into view -- one
        belonging to the intersection after this one -- would change the turn
        the robot is already committed to.
        """
        if self.mode not in PLANNING_MODES:
            return

        if self.planned_turn_direction is not None:
            return

        sign_type = self.sign_db.get(msg.data, "")
        allowed = ALLOWED_DIRECTIONS.get(sign_type)

        if not allowed:
            return

        self.planned_turn_direction = random.choice(allowed)
        rospy.loginfo("Sign %d (%s) -> turning %s", msg.data, sign_type,
                      self.planned_turn_direction.name)

    def _decide_turn(self):
        """
        The direction to drive, in order of authority.

        A forced direction wins for debugging, then the choice made from the
        sign, then straight ahead when no sign was read at all.
        """
        if self.force_turn is not None:
            return self.force_turn, "forced"

        if self.planned_turn_direction is not None:
            return self.planned_turn_direction, "sign"

        return TurnDirection.STRAIGHT, "no sign"

    def _step(self, current_time):
        """One pass over the state machine transitions."""
        if self.mode is DriveMode.LANE_FOLLOWING:
            # The cooldown is what stops the intersection just crossed from
            # triggering again while its line is still in view behind.
            if current_time < self.ignore_red_line_until:
                return

            if self.current_intersection_state is \
                    IntersectionState.APPROACHING_INTERSECTION:
                self._enter(DriveMode.APPROACHING_STOP_LINE, current_time)
            elif self.current_intersection_state is \
                    IntersectionState.AT_INTERSECTION:
                self._enter(DriveMode.STOPPED, current_time)

        elif self.mode is DriveMode.APPROACHING_STOP_LINE:
            if self.current_intersection_state is \
                    IntersectionState.AT_INTERSECTION:
                self._enter(DriveMode.STOPPED, current_time)

        elif self.mode is DriveMode.STOPPED:
            if (current_time - self.state_timer) >= self.stop_duration:
                self._start_crossing(current_time)

        elif self.mode is DriveMode.CROSSING_INTERSECTION:
            self._maybe_finish_crossing(current_time)

    def _enter(self, mode, current_time):
        """Switches state, restarts the state clock, and says so once."""
        self.mode = mode
        self.state_timer = current_time
        rospy.loginfo(f"-> {mode.name}")

    def _start_crossing(self, current_time):
        """Commits to a direction and begins the turn."""
        self.turn_direction, reason = self._decide_turn()

        if reason == "no sign":
            rospy.logwarn("No sign was read; going straight ahead")
        else:
            rospy.loginfo("Crossing %s (%s)", self.turn_direction.name,
                          reason)

        durations = self.config["timers"]["turn_durations"]
        self.turn_duration = float(
            durations.get(self.turn_direction.name, 2.0)
        )

        self._enter(DriveMode.CROSSING_INTERSECTION, current_time)

    def _maybe_finish_crossing(self, current_time):
        """
        Ends the crossing once the robot is back in a lane.

        Closed loop on purpose: the manoeuvre itself is open loop, so it is
        only as accurate as the pose the robot happened to have at the stop
        line. Ending on seeing a lane again rather than on the clock means a
        turn that comes out slightly wide still finishes in the right place;
        the configured duration is only the timeout for when it never does.
        """
        time_in_state = current_time - self.state_timer

        if not crossing.should_exit_crossing(
                time_in_state, self.turn_duration,
                self.white_detected, self.yellow_detected,
                min_blind_duration=self.min_blind_duration,
                mode=self.exit_confidence_mode):
            return

        reason = crossing.exit_reason(
            time_in_state, self.turn_duration,
            self.white_detected, self.yellow_detected,
            min_blind_duration=self.min_blind_duration,
            mode=self.exit_confidence_mode,
        )

        if reason == "timeout":
            # Always timing out means the exit detection is not working and
            # the crossing is silently open loop again.
            rospy.logwarn("Crossing timed out after %.2fs without "
                          "re-acquiring a lane", time_in_state)
        else:
            rospy.loginfo("Crossing done after %.2fs (%s)", time_in_state,
                          reason)

        self._enter(DriveMode.LANE_FOLLOWING, current_time)
        self.ignore_red_line_until = (
            current_time + self.red_line_ignore_duration
        )

        # Free the choice for the next intersection.
        self.planned_turn_direction = None

    def run(self):
        """
        Main loop.

        Faster than 10 Hz so state changes are not quantised so coarsely: the
        crossing ends on seeing a lane, and at 10 Hz that decision could be
        acted on up to 100 ms late, which is real extra distance travelled
        mid-turn.
        """
        rate = rospy.Rate(self.config.get("loop_rate", 30))

        while not rospy.is_shutdown():
            self._step(rospy.Time.now().to_sec())

            self.pub_mode.publish(Int32(data=self.mode.value))
            self.pub_turn.publish(Int32(data=self.turn_direction.value))

            rate.sleep()


if __name__ == "__main__":
    try:
        node = SwitchControlNode("switch_control_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
