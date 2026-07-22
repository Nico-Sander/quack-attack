#!/usr/bin/env python3

"""
Turns the lane error into a wheel command.

A PID controller produces the turn rate from the error while the speed is
prescribed; the result goes to the wheels as a Twist2DStamped. The integral
term is limited and the turn rate hard clamped, so single outliers in
perception cannot become a hard steering kick.

What the controller does depends on the drive mode switch_control commands:
lane following and clearing steer, standing still at the line publishes zeros,
and crossing drives a fixed blind command.
"""

import json
import os

import rospy
from duckietown_msgs.msg import FSMState, Twist2DStamped
from std_msgs.msg import Float64, Int32

from custom_enums import DriveMode

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


class ControlLaneNode:
    """PID lane controller and the wheel command publisher."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.config = self._load_config()
        self.crossing_cmd = self.config["crossing"]

        self.active_mode = DriveMode.LANE_FOLLOWING

        # Controller state. v stays 0 until the first lane message arrives,
        # which cannot happen before detect_lane has loaded its network -- so
        # the robot holds its placement until perception is actually up,
        # without needing a separate gate.
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.d_filtered = 0.0
        self.v = 0.0
        self.omega = 0.0

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

        rospy.loginfo("control_lane ready: %d Hz, max %.2f m/s, "
                      "Duckiebot FSM forced to LANE_FOLLOWING",
                      self.config.get("publish_rate", 30),
                      self.config["pid"]["max_vel"])

        self.sub_lane = rospy.Subscriber(
            f"{base_topic}/detect/lane", Float64, self._cb_lane, queue_size=1
        )
        self.sub_mode = rospy.Subscriber(
            f"{base_topic}/switch/mode", Int32, self._cb_mode, queue_size=1
        )

        rospy.on_shutdown(self._fn_shutdown)

    def _load_config(self):
        """Reads the control_lane section of the package config."""
        config_path = os.path.join(
            os.path.dirname(__file__), "../config/config.json"
        )

        try:
            with open(config_path, "r") as config_file:
                return json.load(config_file)["control_lane"]
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

    def _cb_mode(self, msg):
        """Follows the mode switch_control commands."""
        try:
            self.active_mode = DriveMode(msg.data)
        except ValueError:
            rospy.logwarn_throttle(5.0, f"Invalid drive mode: {msg.data}")

    def _cb_lane(self, msg):
        """
        Runs one PID step per lane message.

        The controller is clocked by perception rather than by the publishing
        loop, so the gains do not change when the frame rate does.
        """
        error = msg.data

        if self.active_mode in (DriveMode.AT_STOP_LINE, DriveMode.CROSSING):
            # Not steering in these states, so the controller is reset rather
            # than left to integrate an error it is not acting on.
            self.v, self.omega = 0.0, 0.0
            self.integral = 0.0
            self.last_time = None
            return

        current_time = rospy.Time.now().to_sec()

        if self.last_time is None:
            # First sample after a reset: there is no dt to differentiate
            # over, so just start moving and steer from the next one.
            self.last_time = current_time
            self.last_error = error
            self.v = self.config["pid"]["max_vel"]
            return

        dt = current_time - self.last_time

        if dt > 0.0:
            self.omega = self._compute_omega(error, dt)
            self.v = max(
                self.config["pid"]["max_vel"]
                * (1.0 - (abs(error) * SPEED_ERROR_PENALTY)),
                MIN_VEL,
            )

        self.last_error = error
        self.last_time = current_time

    def _compute_omega(self, error, dt):
        """The three controller terms, summed and clamped."""
        pid = self.config["pid"]

        p_term = pid["p"] * error

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
        d_term = pid["d"] * self.d_filtered

        omega = p_term + i_term + d_term

        return max(min(omega, MAX_OMEGA), -MAX_OMEGA)

    def _compute_twist(self):
        """The wheel command for this tick."""
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()

        if self.active_mode in (DriveMode.LANE_FOLLOWING,
                                DriveMode.CROSSING_CLEARING):
            twist.v = self.v
            twist.omega = self.omega

        elif self.active_mode is DriveMode.CROSSING:
            # Blind, because the stop line hides the markings the controller
            # would otherwise steer on.
            twist.v = self.crossing_cmd["v"]
            twist.omega = self.crossing_cmd["omega"]

        # AT_STOP_LINE falls through with the zeros the message starts with.

        return twist

    def _fn_shutdown(self):
        """Stops the robot when the node goes away."""
        rospy.loginfo("Shutting down control_lane. Stopping robot.")
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        """
        Publishing loop.

        Runs faster than the lane detector on purpose. The PID itself is still
        computed once per lane message, so the gains are unaffected -- this
        only decides how promptly the newest command reaches the wheels, and
        how finely the timed crossing is resolved.
        """
        rate = rospy.Rate(self.config.get("publish_rate", 30))

        while not rospy.is_shutdown():
            self.pub_cmd_vel.publish(self._compute_twist())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlLaneNode("control_lane_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
