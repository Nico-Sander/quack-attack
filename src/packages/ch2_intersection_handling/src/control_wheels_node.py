#!/usr/bin/env python3

"""
Executes the driving commands.

Lane following works like the controller from challenge 1: a PID turns the
lane error into a turn rate. Approaching the stop line it runs the same
controller with softer gains and a lower speed, so the robot arrives slowly.

The crossing itself is driven open loop, over fixed times and turn rates per
direction, because inside the intersection there are no lane markings a
controller could work from.
"""

import json
import os

import rospy
from duckietown_msgs.msg import FSMState, Twist2DStamped
from std_msgs.msg import Bool, Float64, Int32

from custom_enums import DriveMode, IntersectionPhase, TurnDirection

# The turn rate the summed controller output is clamped to, in rad/s. Anything
# beyond this is a misdetection, not a curve.
MAX_OMEGA = 5.0

# The integral is clamped to this before being scaled by the gain, so a long
# stretch of one-sided error cannot wind up into a lasting steering offset.
MAX_INTEGRAL = 1.0

# How much of the speed is given up at full error. Slowing down when far off
# centre buys the controller time to steer back before the next curve.
SPEED_ERROR_PENALTY = 0.7

# Speed floor, so the robot keeps creeping towards the lane instead of
# stalling in place when the error is large.
MIN_VEL = 0.04


class ControlWheelsNode:
    """PID lane controller, turn manoeuvre, and the wheel command publisher."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.config = self._load_config()

        self.active_mode = DriveMode.LANE_FOLLOWING
        self.turn_direction = TurnDirection.STRAIGHT
        self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
        self.intersection_phase_start_time = 0.0

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.d_filtered = 0.0
        self.v = 0.0
        self.omega = 0.0

        # Nothing moves until detect_signs says it is detecting.
        #
        # Motion is implicitly gated on detect_lane already -- v stays 0 until
        # the first lane message, which cannot arrive before that node has
        # loaded its network. Sign detection has no such gate and is the
        # slower of the two to come up, because building the 52h13 decode
        # table alone takes about five seconds. Without this the robot pulls
        # away roughly a second before any tag can be read, and a sign already
        # in frame at the start is missed -- which in this challenge means
        # turning the wrong way.
        #
        # The flag is latched, so subscribing late is safe.
        self.wait_for_signs = bool(rospy.get_param("~wait_for_signs", True))
        self.signs_ready = not self.wait_for_signs

        base_topic = f"/{self._vehicle_name}"

        self.pub_cmd_vel = rospy.Publisher(
            f"{base_topic}/lane_controller_node/car_cmd",
            Twist2DStamped, queue_size=1
        )
        self.pub_fsm = rospy.Publisher(
            f"{base_topic}/fsm_node/mode", FSMState, queue_size=1, latch=True
        )

        # Publish this immediately: until the Duckiebot's own state machine is
        # in LANE_FOLLOWING it ignores our car_cmd entirely and the robot does
        # not move at all.
        fsm_msg = FSMState()
        fsm_msg.state = "LANE_FOLLOWING"
        self.pub_fsm.publish(fsm_msg)

        rospy.loginfo("control_wheels ready: %d Hz, max %.2f m/s, "
                      "Duckiebot FSM forced to LANE_FOLLOWING",
                      self.config.get("publish_rate", 30),
                      self.config["pid"]["max_vel"])

        self.sub_lane = rospy.Subscriber(
            f"{base_topic}/detect/lane", Float64, self._cb_lane, queue_size=1
        )
        self.sub_mode = rospy.Subscriber(
            f"{base_topic}/switch/mode", Int32, self._cb_mode, queue_size=1
        )
        self.sub_turn = rospy.Subscriber(
            f"{base_topic}/switch/turn_direction", Int32,
            self._cb_direction, queue_size=1
        )
        self.sub_signs_ready = rospy.Subscriber(
            f"{base_topic}/detect/sign_ready", Bool,
            self._cb_signs_ready, queue_size=1
        )

        rospy.on_shutdown(self._fn_shutdown)

    def _load_config(self):
        """Reads the control_wheels section of the package config."""
        config_path = os.path.join(
            os.path.dirname(__file__), "../config/config.json"
        )

        try:
            with open(config_path, "r") as config_file:
                return json.load(config_file)["control_wheels"]
        except (IOError, OSError, KeyError, ValueError) as error:
            # No defaults here, unlike the perception nodes: guessing gains
            # would drive the robot off the track. Better to refuse to start.
            #
            # Raising rather than returning None matters. Shutting down alone
            # does not stop __init__, so the next line would fail on a None
            # config and bury this message under an unrelated TypeError.
            rospy.logerr(f"Missing config.json parameters: {error}")
            rospy.signal_shutdown("Missing required config.")
            raise

    def _cb_signs_ready(self, msg):
        """
        Releases the wheels once sign detection is live.

        One way only: the flag says "the pipeline has come up", and a node that
        later falls silent is a different problem. Freezing the robot
        mid-crossing on a dropped message would be worse than driving on.
        """
        if not msg.data or self.signs_ready:
            return

        self.signs_ready = True
        rospy.loginfo("Sign detection is live -- releasing the wheels")

    def _cb_direction(self, msg):
        """Follows the turn direction switch_control decided on."""
        try:
            self.turn_direction = TurnDirection(msg.data)
        except ValueError:
            self.turn_direction = TurnDirection.STRAIGHT

    def _cb_mode(self, msg):
        """Follows the mode switch_control commands."""
        try:
            new_mode = DriveMode(msg.data)
        except ValueError:
            return

        # Start the crossing phases only on the way *into* the state. The mode
        # is republished every tick, so keying off the message alone would
        # restart the manoeuvre from its first phase thirty times a second.
        if (new_mode is DriveMode.CROSSING_INTERSECTION
                and self.active_mode is not DriveMode.CROSSING_INTERSECTION):
            self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
            self.intersection_phase_start_time = rospy.Time.now().to_sec()

        self.active_mode = new_mode

    def _cb_lane(self, msg):
        """
        Runs one PID step per lane message.

        The controller is clocked by perception rather than by the publishing
        loop, so the gains do not change when the frame rate does.
        """
        error = msg.data

        if not self.signs_ready:
            # Lane messages usually start about a second before the wheels are
            # released. Running the PID over that second would wind the
            # integrator up and stamp last_time with a moment the robot spent
            # standing still, so it starts from rest instead.
            self.v, self.omega = 0.0, 0.0
            self.integral = 0.0
            self.last_time = None
            return

        if self.active_mode in (DriveMode.STOPPED,
                                DriveMode.CROSSING_INTERSECTION):
            # Not steering in these states, so the controller is reset rather
            # than left to integrate an error it is not acting on.
            self.v, self.omega = 0.0, 0.0
            self.integral = 0.0
            self.last_time = None
            return

        current_time = rospy.Time.now().to_sec()
        max_vel, kp, kd = self._gains_for_mode()

        if self.last_time is None:
            # First sample after a reset: there is no dt to differentiate
            # over, so just start moving and steer from the next one.
            self.last_time = current_time
            self.last_error = error
            self.v = max_vel
            return

        dt = current_time - self.last_time

        if dt > 0.0:
            self.omega = self._compute_omega(error, dt, kp, kd)
            self.v = max(
                max_vel * (1.0 - (abs(error) * SPEED_ERROR_PENALTY)), MIN_VEL
            )

        self.last_error = error
        self.last_time = current_time

    def _gains_for_mode(self):
        """
        Speed and gains for the current mode.

        Approaching the line the robot drives slower and steers more gently,
        so it arrives at the line settled rather than still correcting.
        """
        pid = self.config["pid"]
        max_vel = pid["max_vel"]

        if self.active_mode is DriveMode.APPROACHING_STOP_LINE:
            return (max_vel * self.config["approach_speed_multiplier"],
                    pid["p_slow"], pid["d_slow"])

        return max_vel, pid["p"], pid["d"]

    def _compute_omega(self, error, dt, kp, kd):
        """
        The three controller terms, summed and clamped.

        Pure lane centering, in every mode. There used to be a "red-line
        squaring" blend here that steered towards being perpendicular to the
        stop line as it got close. It was dropped: squaring up only pays off
        if the robot also arrives at a repeatable distance from the line,
        which it does not, and the crossing no longer depends on a perfect
        starting pose now that it ends on seeing a lane again rather than on a
        timer. Fighting the lane controller near the line made stops less
        consistent, not more.
        """
        pid = self.config["pid"]

        p_term = kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, MAX_INTEGRAL), -MAX_INTEGRAL)
        i_term = pid["i"] * self.integral

        # Low-pass filtered derivative. The error is a segmentation median, so
        # it is quantised and noisy; differentiating it amplifies that noise,
        # and the faster the loop runs the smaller and noisier each step
        # becomes. The filter keeps the D term usable regardless of the control
        # rate, which is what lets frame_skip be changed without the steering
        # blowing up. tau is a time constant in seconds: larger = smoother but
        # laggier. alpha is derived from the real dt so the filter behaves the
        # same whether the loop runs at 10 or 30 Hz.
        raw_derivative = (error - self.last_error) / dt
        tau = pid.get("d_filter_tau", 0.08)
        alpha = dt / (tau + dt) if (tau + dt) > 0.0 else 1.0
        self.d_filtered += alpha * (raw_derivative - self.d_filtered)
        d_term = kd * self.d_filtered

        omega = p_term + i_term + d_term

        return max(min(omega, MAX_OMEGA), -MAX_OMEGA)

    def _execute_intersection_crossing(self, twist):
        """
        Drives the turn, open loop, one phase at a time.

        First roll straight far enough to get the wheels into the
        intersection, then turn at a fixed rate. Turning immediately would
        clip the corner of the lane being left.
        """
        current_time = rospy.Time.now().to_sec()
        cross_cfg = self.config["intersection"]

        # logdebug, not loginfo: switch_control already reports each crossing
        # and its outcome, so this low-level phase trace is noise on the
        # console by default. Runs every publish tick, hence also throttled.
        rospy.logdebug_throttle(
            0.5, f"Crossing phase={self.intersection_phase.name}, "
                 f"direction={self.turn_direction.name}"
        )

        if self.intersection_phase is IntersectionPhase.STRAIGHT_BEFORE_TURN:
            twist.v = cross_cfg["straight_before_turn"]["v"]
            twist.omega = cross_cfg["straight_before_turn"]["omega"]

            duration = cross_cfg["durations"].get(
                self.turn_direction.name, 0.0
            )

            if (current_time
                    - self.intersection_phase_start_time) >= duration:
                self.intersection_phase = IntersectionPhase.INITIAL_TURNING
                self.intersection_phase_start_time = current_time

        elif self.intersection_phase is IntersectionPhase.INITIAL_TURNING:
            turn_cfg = cross_cfg["initial_turn"].get(
                self.turn_direction.name, cross_cfg["initial_turn"]["STRAIGHT"]
            )
            twist.v = turn_cfg["v"]
            twist.omega = turn_cfg["omega"]

    def _compute_twist(self):
        """The wheel command for this tick."""
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()

        # Before anything else, including the crossing manoeuvre: no
        # perception, no movement. At startup the mode is LANE_FOLLOWING, so
        # in practice this is what holds the robot at its placement until the
        # whole pipeline is up.
        if not self.signs_ready:
            rospy.logwarn_throttle(
                1.0, "Holding still: waiting for detect_signs to come up "
                     "(set wait_for_signs:=false to drive anyway)"
            )
            return twist

        if self.active_mode in (DriveMode.LANE_FOLLOWING,
                                DriveMode.APPROACHING_STOP_LINE):
            twist.v = self.v
            twist.omega = self.omega

        elif self.active_mode is DriveMode.CROSSING_INTERSECTION:
            self._execute_intersection_crossing(twist)

        # STOPPED falls through with the zeros the message starts with.

        return twist

    def _fn_shutdown(self):
        """Stops the robot when the node goes away."""
        rospy.loginfo("Shutting down control_wheels. Stopping robot.")
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        """
        Publishing loop.

        Runs faster than the lane detector on purpose. The PID itself is still
        computed once per lane message, so the gains are unaffected -- this
        only decides how promptly the newest command reaches the wheels, and
        how finely the timed phases of a crossing are resolved. At 10 Hz a
        phase boundary could land up to 100 ms late, which on a 0.46 s segment
        is a 22% error.
        """
        rate = rospy.Rate(self.config.get("publish_rate", 30))

        while not rospy.is_shutdown():
            self.pub_cmd_vel.publish(self._compute_twist())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlWheelsNode("control_wheels_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
