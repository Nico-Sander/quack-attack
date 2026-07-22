#!/usr/bin/env python3

"""
The state machine of the package.

It separates perception from control: "a red line is visible" arrives here and
leaves as the decision to stop, to cross and to carry on. Exactly one command
goes to the controller, so it is always unambiguous what the robot is meant to
be doing.

All transitions sit in a single loop and are driven by two signals -- whether a
red line is there, and how long the current state has lasted.
"""

import json
import os

import rospy
from std_msgs.msg import Bool, Int32

from custom_enums import DriveMode


class SwitchControlNode:
    """Decides the drive mode and publishes it."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.config = self._load_config()
        timers = self.config["timers"]
        self.stop_duration = float(timers["stop_duration"])
        self.crossing_duration = float(timers["crossing_duration"])
        self.min_clearing_duration = float(timers["min_clearing_duration"])
        self.max_clearing_duration = float(timers["max_clearing_duration"])

        self.mode = DriveMode.LANE_FOLLOWING
        self.state_timer = 0.0

        # Perception state, as of the last message from detect_intersection.
        self.stop_line_reached = False
        self.red_line_visible = False

        base_topic = f"/{self._vehicle_name}"

        self.sub_intersection = rospy.Subscriber(
            f"{base_topic}/detect/intersection", Bool,
            self._cb_intersection, queue_size=1
        )
        self.sub_red_line_visible = rospy.Subscriber(
            f"{base_topic}/detect/red_line_visible", Bool,
            self._cb_red_line_visible, queue_size=1
        )
        self.pub_mode = rospy.Publisher(
            f"{base_topic}/switch/mode", Int32, queue_size=1
        )

        rospy.loginfo("switch_control ready: stop %.1fs, cross %.1fs blind",
                      self.stop_duration, self.crossing_duration)

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
                "timers": {
                    "stop_duration": 3.0,
                    "crossing_duration": 1.2,
                    "min_clearing_duration": 0.5,
                    "max_clearing_duration": 4.0,
                },
                "loop_rate": 30,
            }

    def _cb_intersection(self, msg):
        """The robot has arrived at a stop line."""
        self.stop_line_reached = bool(msg.data)

    def _cb_red_line_visible(self, msg):
        """A red line is somewhere in the image, however far away."""
        self.red_line_visible = bool(msg.data)

    def _enter(self, mode, current_time):
        """Switches state, restarts the state clock, and says so once."""
        self.mode = mode
        self.state_timer = current_time
        rospy.loginfo(f"-> {mode.name}")

    def _step(self, current_time):
        """
        One pass over the transitions.

        Only one transition can fire per tick, which is what keeps the robot
        from skipping a state when several conditions happen to hold at once.
        """
        time_in_state = current_time - self.state_timer

        if self.mode is DriveMode.LANE_FOLLOWING:
            # Only a line the robot has actually driven up to stops it; one
            # further up the road is still something to steer towards.
            #
            # Both flags, not just the first: they describe one observation but
            # arrive as two messages, so for one tick the loop can see the new
            # "no red line" next to the old "at the line". Right after clearing
            # a line that is exactly the pair that would stop the robot again
            # on the line it just left. A line it is standing at is always also
            # visible, so requiring both costs nothing real.
            if self.stop_line_reached and self.red_line_visible:
                self._enter(DriveMode.AT_STOP_LINE, current_time)

        elif self.mode is DriveMode.AT_STOP_LINE:
            if time_in_state >= self.stop_duration:
                self._enter(DriveMode.CROSSING, current_time)

        elif self.mode is DriveMode.CROSSING:
            # Open loop on purpose: right on the line the lane markings are
            # partly hidden by it, so there is nothing worth steering on for
            # the short stretch it takes to get across.
            if time_in_state >= self.crossing_duration:
                self._enter(DriveMode.CROSSING_CLEARING, current_time)

        elif self.mode is DriveMode.CROSSING_CLEARING:
            # The line only counts as passed once it is out of the image.
            # min_clearing_duration covers the moment right after the
            # crossing, when the last message from perception can still be
            # describing the line the robot was standing at.
            if (time_in_state >= self.min_clearing_duration
                    and not self.red_line_visible):
                self._enter(DriveMode.LANE_FOLLOWING, current_time)

            elif time_in_state >= self.max_clearing_duration:
                # Something is reporting red that the robot cannot drive away
                # from. Carrying on is better than standing in this state
                # forever, but it means the next real stop line may be missed,
                # so it is worth a warning.
                rospy.logwarn(
                    f"Red line still visible {time_in_state:.1f}s after "
                    f"crossing -- clearing anyway"
                )
                self._enter(DriveMode.LANE_FOLLOWING, current_time)

    def run(self):
        """
        Main loop.

        Faster than the camera on purpose: at 10 Hz a state change could be
        acted on up to 100 ms late, which on a 1.2 s blind crossing is real
        extra distance travelled.
        """
        rate = rospy.Rate(self.config.get("loop_rate", 30))

        while not rospy.is_shutdown():
            self._step(rospy.Time.now().to_sec())

            # Republished every tick, not just on a change: the controller and
            # the lane detector both subscribe with queue_size 1, so a single
            # dropped message would otherwise leave them on the wrong mode.
            self.pub_mode.publish(Int32(data=self.mode.value))

            rate.sleep()


if __name__ == "__main__":
    try:
        node = SwitchControlNode("switch_control_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
