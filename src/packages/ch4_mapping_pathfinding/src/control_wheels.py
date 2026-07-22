#!/usr/bin/env python3

"""
ROS node for controlling physical wheel commands based on DriveMode.
"""

import os
import json
import rospy
from std_msgs.msg import Bool, Float64, Int32
from duckietown_msgs.msg import Twist2DStamped, FSMState
from custom_enums import DriveMode, IntersectionPhase, TurnDirection

class ControlWheelsNode:
    """Action node translating logic commands into Twist messages."""

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_robot')
        
        self.config = self._load_config()
        
        # State & Movement variables
        self.active_mode = DriveMode.LANE_FOLLOWING
        self.turn_direction = TurnDirection.STRAIGHT
        self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
        self.intersection_phase_start_time = 0.0

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        # Filtered derivative. See the low-pass in _cb_lane.
        self.d_filtered = 0.0
        
        self.v = 0.0
        self.omega = 0.0

        # Nothing moves until detect_signs says it is detecting.
        #
        # Motion has always been gated on detect_lane implicitly -- v stays 0
        # until the first /detect/lane message, which cannot arrive before that
        # node has loaded its network. Gate detection had no such gate, and it
        # is the *slower* of the two to come up (~5.5s against ~4.5s: building
        # the tagStandard52h13 decode table alone takes five seconds). So the
        # bot pulled away roughly a second before any tag could be detected and
        # drove ~0.2m of its first street blind -- enough to miss a gate that
        # was already in frame at the start, which is exactly what happened.
        #
        # The flag is latched, so subscribing late is safe.
        self.wait_for_signs = bool(rospy.get_param("~wait_for_signs", True))
        self.signs_ready = not self.wait_for_signs

        # Topics
        base = f"/{self._vehicle_name}"
        self.pub_cmd_vel = rospy.Publisher(f"{base}/lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_fsm = rospy.Publisher(f"{base}/fsm_node/mode", FSMState, queue_size=1, latch=True)

        # Publish the state immediately so the Duckiebot listens to your lane controller
        fsm_msg = FSMState()
        fsm_msg.state = "LANE_FOLLOWING"
        self.pub_fsm.publish(fsm_msg)
        rospy.loginfo("control_wheels ready: %d Hz, max %.2f m/s, "
                      "Duckiebot FSM forced to LANE_FOLLOWING",
                      self.config.get("publish_rate", 30),
                      self.config["pid"]["max_vel"])
        
        self.sub_lane = rospy.Subscriber(f"{base}/detect/lane", Float64, self._cb_lane, queue_size=1)

        self.sub_mode = rospy.Subscriber(f"{base}/switch/mode", Int32, self._cb_mode, queue_size=1)
        self.sub_turn = rospy.Subscriber(f"{base}/switch/turn_direction", Int32, self._cb_direction, queue_size=1)
        self.sub_signs_ready = rospy.Subscriber(
            f"{base}/detect/sign_ready", Bool, self._cb_signs_ready, queue_size=1
        )

        rospy.on_shutdown(self._fn_shutdown)

    def _load_config(self):
        """Loads control parameters from JSON."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["control_wheels"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logerr(f"Missing config.json parameters: {e}")
            rospy.signal_shutdown("Missing required config.")

    def _cb_signs_ready(self, msg):
        """
        Releases the wheels once sign detection is live.

        One way only: the flag says "the pipeline has come up", and a node that
        later falls silent is a different problem. Freezing the bot mid-crossing
        on a dropped message would be worse than driving on.
        """
        if not msg.data or self.signs_ready:
            return

        self.signs_ready = True
        rospy.loginfo("Sign detection is live -- releasing the wheels")

    def _cb_direction(self, msg):
        try:
            self.turn_direction = TurnDirection(msg.data)
        except ValueError:
            self.turn_direction = TurnDirection.STRAIGHT

    def _cb_mode(self, msg):
        """Updates internal state based on commanded mode, ensuring idempotent crossing initialization."""
        try:
            new_mode = DriveMode(msg.data)
        except ValueError:
            return

        # Initialize crossing timers ONLY if transitioning into CROSSING_INTERSECTION
        if new_mode == DriveMode.CROSSING_INTERSECTION and self.active_mode != DriveMode.CROSSING_INTERSECTION:
            self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
            self.intersection_phase_start_time = rospy.Time.now().to_sec()

        self.active_mode = new_mode

    def _cb_lane(self, msg):
        """Calculates PID if the current mode requires lane following."""
        error = msg.data

        if not self.signs_ready:
            # Lane messages usually start about a second before the wheels are
            # released. Running the PID over that second would wind the
            # integrator up and stamp last_time with a moment the bot spent
            # standing still, so it starts from rest instead.
            self.v, self.omega = 0.0, 0.0
            self.integral = 0.0
            self.last_time = None
            return

        if self.active_mode in (DriveMode.STOPPED, DriveMode.CROSSING_INTERSECTION):
            self.v, self.omega = 0.0, 0.0
            return

        current_time = rospy.Time.now().to_sec()
        
        max_vel = self.config["pid"]["max_vel"]
        if self.active_mode == DriveMode.APPROACHING_STOP_LINE:
            max_vel *= self.config["approach_speed_multiplier"]
            kp = self.config["pid"]["p_slow"]
            kd = self.config["pid"]["d_slow"]
        else:
            kp = self.config["pid"]["p"]
            kd = self.config["pid"]["d"]

        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            self.v = max_vel
            return
            
        dt = current_time - self.last_time
        
        if dt > 0.0:
            # Standard Lane Centering PID
            p_term = kp * error
            self.integral += error * dt
            self.integral = max(min(self.integral, 1.0), -1.0)
            i_term = self.config["pid"]["i"] * self.integral

            # Low-pass filtered derivative. The error is a segmentation median,
            # so it is quantised and noisy; differentiating it amplifies that
            # noise, and the faster the loop runs the smaller and noisier each
            # step becomes. The filter keeps the D term usable regardless of the
            # control rate, which is what lets frame_skip be changed without the
            # steering blowing up. tau is a time constant in seconds: larger =
            # smoother but laggier. The alpha is derived from the real dt so the
            # filter behaves the same whether the loop runs at 10 or 30 Hz.
            raw_derivative = (error - self.last_error) / dt
            tau = self.config["pid"].get("d_filter_tau", 0.08)
            alpha = dt / (tau + dt) if (tau + dt) > 0.0 else 1.0
            self.d_filtered += alpha * (raw_derivative - self.d_filtered)
            d_term = kd * self.d_filtered

            # Steering is lane centering, in every mode. There used to be a
            # "red-line squaring" blend here that steered towards being
            # perpendicular to the stop line as it got close. It was dropped:
            # squaring up only pays off if the bot also arrives at a repeatable
            # distance from the line, which it does not, and the crossing no
            # longer depends on a perfect starting pose now that it ends on
            # seeing a lane again rather than on a timer. Fighting the lane
            # controller near the line made stops less consistent, not more.
            omega_lane = p_term + i_term + d_term
            self.omega = max(min(omega_lane, 5.0), -5.0)

            # Linear Velocity Calculation
            self.v = max(max_vel * (1.0 - (abs(error) * 0.7)), 0.04)

        self.last_error = error
        self.last_time = current_time

    def _execute_intersection_crossing(self, twist):
        """Open-loop kinematic execution based on current crossing phase."""
        current_time = rospy.Time.now().to_sec()
        cross_cfg = self.config["intersection"]
        # logdebug, not loginfo: switch_control already reports each crossing
        # and its outcome, so this low-level phase trace is noise on the console
        # by default. Runs every publish tick, hence also throttled.
        rospy.logdebug_throttle(
            0.5,
            f"Crossing phase={self.intersection_phase.name}, "
            f"direction={self.turn_direction.name}"
        )

        if self.intersection_phase == IntersectionPhase.STRAIGHT_BEFORE_TURN:
            twist.v = cross_cfg["straight_before_turn"]["v"]
            twist.omega = cross_cfg["straight_before_turn"]["omega"]

            duration = cross_cfg["durations"].get(self.turn_direction.name, 0.0)
            
            if (current_time - self.intersection_phase_start_time) >= duration:
                self.intersection_phase = IntersectionPhase.INITIAL_TURNING
                self.intersection_phase_start_time = current_time

        elif self.intersection_phase == IntersectionPhase.INITIAL_TURNING:
            turn_cfg = cross_cfg["initial_turn"].get(
                self.turn_direction.name, 
                cross_cfg["initial_turn"]["STRAIGHT"]
            )
            twist.v = turn_cfg["v"]
            twist.omega = turn_cfg["omega"]

    def _fn_shutdown(self):
        rospy.loginfo("Shutting down control_wheels. Stopping robot.")
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    def _compute_twist(self):
        """
        The wheel command for this tick.

        Split out from run() so the startup gate below can be tested without a
        ROS master; see tests/test_startup_gating.py.
        """
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()

        # Before anything else, including the open-loop crossing manoeuvre: no
        # perception, no movement. At startup the mode is LANE_FOLLOWING, so in
        # practice this is what holds the bot at its placement until the whole
        # pipeline is up.
        if not self.signs_ready:
            rospy.logwarn_throttle(
                1.0, "Holding still: waiting for detect_signs to come up "
                     "(set wait_for_signs:=false to drive anyway)"
            )
            return twist

        if self.active_mode in (DriveMode.LANE_FOLLOWING, DriveMode.APPROACHING_STOP_LINE):
            twist.v = self.v
            twist.omega = self.omega

        elif self.active_mode == DriveMode.STOPPED:
            twist.v = 0.0
            twist.omega = 0.0

        elif self.active_mode == DriveMode.CROSSING_INTERSECTION:
            self._execute_intersection_crossing(twist)

        return twist

    def run(self):
        """
        Publishing loop.

        Runs faster than the lane detector on purpose. The PID itself is still
        computed once per lane message (~10 Hz), so the gains are unaffected --
        this only decides how promptly the newest command reaches the wheels,
        and how finely the timed phases of a crossing are resolved. At 10 Hz a
        crossing phase boundary could land up to 100 ms late, which on a 0.42 s
        segment is a 24% error.
        """
        rate = rospy.Rate(self.config.get("publish_rate", 30))

        while not rospy.is_shutdown():
            self.pub_cmd_vel.publish(self._compute_twist())
            rate.sleep()
            
if __name__ == '__main__':
    try:
        node = ControlWheelsNode('control_wheels_node')
        node.run()
    except rospy.ROSInterruptException:
        pass