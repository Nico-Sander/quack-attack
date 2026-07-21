#!/usr/bin/env python3

"""
Fake driver: exercises the mapping node over real ROS, without the robot.

It stands in for switch_control, walking the drive-mode state machine
(LANE_FOLLOWING -> APPROACHING -> STOPPED -> CROSSING -> LANE_FOLLOWING) at
whatever speed you ask for, and obeys /plan/turn_command exactly the way
switch_control does. It can also fake gate detections: tell it which street
carries which tag and it publishes on /detect/sign_detections whenever the
mapping node reports the bot is on that street.

That closes the whole loop -- planner commands a turn, the "bot" executes it,
the position advances, the route is replanned -- so a full mapping run plus
gate run can be watched in the visualization window in about a minute.

IMPORTANT: do not run this while switch_control is running; both publish
/switch/mode and they will fight. Launch with driving:=false and stop
switch_control, or start the mapping node and visualization on their own.

Usage:
    rosrun mapping_pathfinding simulate_drive.py \
        _gates:=A1__B1:5,B3__C4:6,A3__C1:7 _step_time:=0.4 _moves:=30
"""

import json
import os
import random
import sys

import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Int32, String

# The package's modules live in src/; this node sits in tests/.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"
))

import gate_detection  # noqa: E402
from custom_enums import (  # noqa: E402
    TURN_COMMAND_HALT, TURN_COMMAND_NONE, DriveMode, TurnDirection
)

# A comfortably-accepted tag area, and enough repeats to satisfy the mapping
# node's confirmation streak with room to spare.
FAKE_GATE_AREA = 4 * gate_detection.DEFAULT_MIN_AREA
FAKE_GATE_SIGHTINGS = gate_detection.DEFAULT_CONFIRM_FRAMES + 2


class SimulatedDriver:
    def __init__(self):
        rospy.init_node("simulated_driver")

        vehicle = os.environ.get("VEHICLE_NAME", "default_robot")
        base = f"/{vehicle}"

        # Seconds per state. The default is much faster than reality; the point
        # is to exercise logic, not to imitate timing.
        self.step_time = float(rospy.get_param("~step_time", 0.4))
        self.moves = int(rospy.get_param("~moves", 30))
        self.obey_planner = bool(rospy.get_param("~obey_planner", True))
        # How long to wait at a commanded halt before giving up. Long enough to
        # stand in for a person walking over and starting the gate run.
        self.halt_timeout = float(rospy.get_param("~halt_timeout", 8.0))

        self.gates = self._parse_gates(rospy.get_param("~gates", ""))

        self.commanded_turn = None
        self.commanded_halt = False
        self.current_edge_key = None
        self.phase = None
        # The street whose gate has already been announced for the traversal
        # currently under way. Reset by moving to another street, so driving a
        # street again announces its gate again -- which is what a real camera
        # does, and what the gate run relies on to credit a gate by sighting
        # rather than by the end-of-street fallback.
        self.announced_on_edge = None

        self.pub_mode = rospy.Publisher(f"{base}/switch/mode", Int32, queue_size=1)
        # The mapping node starts the first street's clock on the first wheel
        # command that actually moves the bot, so the fake driver has to produce
        # one or the first street is never timed.
        self.pub_car_cmd = rospy.Publisher(
            f"{base}/lane_controller_node/car_cmd", Twist2DStamped, queue_size=1
        )
        self.pub_turn = rospy.Publisher(f"{base}/switch/turn_direction", Int32,
                                        queue_size=1)
        self.pub_sign = rospy.Publisher(f"{base}/detect/sign_detections",
                                        String, queue_size=1)

        self.resume_seq = None
        self.repositioned = False
        rospy.Subscriber(f"{base}/plan/resume_driving", Int32,
                         self._cb_resume, queue_size=1)
        rospy.Subscriber(f"{base}/plan/turn_command", Int32,
                         self._cb_turn_command, queue_size=1)
        rospy.Subscriber(f"{base}/mapping/state", String,
                         self._cb_state, queue_size=1)

        rospy.loginfo("simulated_driver ready: %d moves, %.2fs per state, gates=%s",
                      self.moves, self.step_time, self.gates or "none")

    def _parse_gates(self, raw):
        """'A1__B1:5,B3__C4:6' -> {'A1__B1': 5, 'B3__C4': 6}"""
        gates = {}

        for chunk in str(raw).split(","):
            chunk = chunk.strip()

            if not chunk:
                continue

            try:
                key, tag = chunk.rsplit(":", 1)
                gates[key.strip()] = int(tag)
            except ValueError:
                rospy.logwarn("Ignoring malformed gate spec %r", chunk)

        return gates

    def _cb_resume(self, msg):
        """
        The bot has been "picked up" and placed at an intersection exit.

        Same rule switch_control follows: the next thing to do is drive the
        street, not cross an intersection that is no longer in front of it.
        """
        if self.resume_seq is None:
            self.resume_seq = msg.data
            return

        if msg.data != self.resume_seq:
            self.resume_seq = msg.data
            self.repositioned = True

    def _cb_turn_command(self, msg):
        self.commanded_halt = msg.data == TURN_COMMAND_HALT

        if msg.data in (TURN_COMMAND_NONE, TURN_COMMAND_HALT):
            self.commanded_turn = None
            return

        try:
            self.commanded_turn = TurnDirection(msg.data)
        except ValueError:
            self.commanded_turn = None

    def _cb_state(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return

        self.current_edge_key = state.get("graph", {}).get("current_edge_key")
        self.phase = state.get("mission", {}).get("phase")

    def _hold(self, mode, seconds):
        """Publishes a drive mode steadily, the way switch_control does."""
        deadline = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(20)
        moving = mode in (DriveMode.LANE_FOLLOWING,
                          DriveMode.APPROACHING_STOP_LINE)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.pub_mode.publish(Int32(data=mode.value))
            self.pub_car_cmd.publish(
                Twist2DStamped(v=0.19 if moving else 0.0, omega=0.0)
            )
            rate.sleep()

    def _street_seconds(self, edge_key):
        """
        A deterministic, per-street drive time.

        Varied on purpose: a simulation where every street takes the same time
        cannot show whether the measurements are being attributed to the right
        street. Kept above run_timing.MIN_PLAUSIBLE_STREET_TIME so the samples
        are not rejected as implausible.
        """
        spread = (sum(ord(char) for char in (edge_key or "")) % 7) * 0.15
        return max(self.step_time * 2.0, 0.7) + spread

    def _hold_until_released(self):
        """
        Stays stopped at the line while the planner is asking for a halt.

        This is what a real run looks like between the mapping phase and the
        gate run: the bot waits to be picked up and placed, and for the run to
        be started.
        """
        rospy.loginfo("[sim] halt commanded -- holding at the red line")
        deadline = rospy.Time.now() + rospy.Duration(self.halt_timeout)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self.commanded_halt:
                rospy.loginfo("[sim] halt released, carrying on")
                return True

            self.pub_mode.publish(Int32(data=DriveMode.STOPPED.value))
            self.pub_car_cmd.publish(Twist2DStamped(v=0.0, omega=0.0))
            rate.sleep()

        rospy.loginfo("[sim] still halted after %.0fs -- ending the run",
                      self.halt_timeout)
        return False

    def _maybe_announce_gate(self):
        """Publishes the gate tag for the street the bot is currently on."""
        tag = self.gates.get(self.current_edge_key)

        if tag is None:
            return

        if self.announced_on_edge == self.current_edge_key:
            return

        self.announced_on_edge = self.current_edge_key

        # Repeated because the mapping node only believes a gate it has seen
        # several frames in a row -- one sighting is deliberately not enough.
        payload = json.dumps({
            "stamp": rospy.Time.now().to_sec(),
            "min_area": gate_detection.DEFAULT_MIN_AREA,
            "detections": [{
                "tag_id": tag,
                "area": FAKE_GATE_AREA,
                "accepted": True,
                "corners": [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
                "center": [5.0, 5.0],
            }],
        })

        for _ in range(FAKE_GATE_SIGHTINGS):
            self.pub_sign.publish(String(data=payload))
            rospy.sleep(0.05)

        rospy.loginfo("[sim] gate %d visible on %s", tag, self.current_edge_key)

    def run(self):
        # Give the mapping node a moment to publish its initial state.
        rospy.sleep(1.0)

        for move in range(self.moves):
            if rospy.is_shutdown():
                break

            # Drive the street. Split across the two driving modes so the
            # measured street time is the whole of it, as on the real bot.
            street_seconds = self._street_seconds(self.current_edge_key)
            self._hold(DriveMode.LANE_FOLLOWING, street_seconds * 0.6)
            self._maybe_announce_gate()
            self._hold(DriveMode.APPROACHING_STOP_LINE, street_seconds * 0.4)
            self._hold(DriveMode.STOPPED, self.step_time)

            # Checked *here*, after the street has been driven and the bot has
            # stopped -- not right after the last gate is passed. The last gate
            # counts as passed when its street is entered, but the real bot then
            # drives that street and stops at the red line beyond it, and that
            # arrival is what closes the run's stopwatch.
            if self.phase == "DONE":
                rospy.loginfo("[sim] mission reported DONE")
                break

            # The planner holds the bot here when mapping finishes, so it can be
            # picked up and placed for the gate run.
            if self.commanded_halt and not self._hold_until_released():
                break

            # Repositioned while halted: there is no intersection in front of
            # the bot any more, so skip the crossing and drive the street.
            if self.repositioned:
                self.repositioned = False
                rospy.loginfo("[sim] repositioned onto %s -- driving off "
                              "without a crossing", self.current_edge_key)
                continue

            # Same decision switch_control makes: planner first, else random.
            if self.obey_planner and self.commanded_turn is not None:
                turn = self.commanded_turn
                source = "planner"
            else:
                turn = random.choice(list(TurnDirection))
                source = "random"

            rospy.loginfo("[sim] move %d/%d: turning %s (%s), phase=%s, on %s",
                          move + 1, self.moves, turn.name, source,
                          self.phase, self.current_edge_key)

            self.pub_turn.publish(Int32(data=turn.value))
            self._hold(DriveMode.CROSSING_INTERSECTION, self.step_time)

            # The falling edge CROSSING -> LANE_FOLLOWING is what makes the
            # mapping node advance its position.
            self.pub_mode.publish(Int32(data=DriveMode.LANE_FOLLOWING.value))
            rospy.sleep(0.2)

            self._maybe_announce_gate()

        rospy.loginfo("[sim] finished")


if __name__ == "__main__":
    try:
        SimulatedDriver().run()
    except rospy.ROSInterruptException:
        pass
