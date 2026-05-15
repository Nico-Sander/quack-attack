#!/usr/bin/env python3

"""
ROS node for controlling physical wheel commands based on DriveMode.
"""

import os
import json
import rospy
from std_msgs.msg import Float64, Int32, String
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
        
        self.v = 0.0
        self.omega = 0.0

        self.red_detected = False
        self.red_distance_y = 0.0
        self.red_angle = 0.0

        # Topics
        base = f"/{self._vehicle_name}"
        self.pub_cmd_vel = rospy.Publisher(f"{base}/lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_fsm = rospy.Publisher(f"{base}/fsm_node/mode", FSMState, queue_size=1, latch=True)

        # Publish the state immediately so the Duckiebot listens to your lane controller
        fsm_msg = FSMState()
        fsm_msg.state = "LANE_FOLLOWING"
        self.pub_fsm.publish(fsm_msg)
        rospy.loginfo("Forced Duckiebot FSM to LANE_FOLLOWING mode.")
        
        self.sub_lane = rospy.Subscriber(f"{base}/detect/lane", Float64, self._cb_lane, queue_size=1)
        self.sub_borders = rospy.Subscriber(f"{base}/detect/lane_borders", String, self._cb_borders, queue_size=1)

        self.sub_mode = rospy.Subscriber(f"{base}/switch/mode", Int32, self._cb_mode, queue_size=1)
        self.sub_turn = rospy.Subscriber(f"{base}/switch/turn_direction", Int32, self._cb_direction, queue_size=1)

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

    def _cb_borders(self, msg):
        """Extracts geometric features from the semantic masks"""
        try:
            data = json.loads(msg.data)
            self.red_detected = data.get("red_detected", False)
            self.red_distance_y = data.get("red_distance_y", 0.0)
            self.red_angle = data.get("red_angle", 0.0)
        except json.JSONDecodeError:
            pass

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
            d_term = kd * ((error - self.last_error) / dt)

            omega_lane = max(min(p_term + i_term + d_term, 5.0), -5.0)

            # Dynamic Red-Line Squaring
            if self.active_mode == DriveMode.APPROACHING_STOP_LINE and self.red_detected:
                # Calculate blending weight (0.0 when far, 1.0 when at the line)
                start_blend_y = 0.40 # Start caring about angle when line is 40% down image
                stop_y = 0.92        # Match this roughly to your intersection threshold
                
                progress = (self.red_distance_y - start_blend_y) / (stop_y - start_blend_y)
                blend_weight = max(0.0, min(1.0, progress))

                # Calculate Angle Error
                # If line slopes down-right (+ angle), robot is facing too far left. Turn Right (- omega).
                kp_angle = self.config["pid"].get("p_angle", 2.5)
                omega_angle = -1.0 * kp_angle * self.red_angle

                # Blend the two steering commands
                omega_final = (omega_lane * (1.0 - blend_weight)) + (omega_angle * blend_weight)
            else:
                omega_final = omega_lane

            self.omega = max(min(omega_final, 5.0), -5.0)

            # Linear Velocity Calculation
            self.v = max(max_vel * (1.0 - (abs(error) * 0.7)), 0.04)

        self.last_error = error
        self.last_time = current_time

    def _execute_intersection_crossing(self, twist):
        """Open-loop kinematic execution based on current crossing phase."""
        current_time = rospy.Time.now().to_sec()
        cross_cfg = self.config["intersection"]
        rospy.loginfo(f"Drive Intersection, Phase: {self.intersection_phase.name}, Direction: {self.turn_direction.name}")

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

    def run(self):
        """Publishing loop running at 10 Hz."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            twist.header.stamp = rospy.Time.now()

            if self.active_mode in (DriveMode.LANE_FOLLOWING, DriveMode.APPROACHING_STOP_LINE):
                twist.v = self.v
                twist.omega = self.omega

            elif self.active_mode == DriveMode.STOPPED:
                twist.v = 0.0
                twist.omega = 0.0

            elif self.active_mode == DriveMode.CROSSING_INTERSECTION:
                self._execute_intersection_crossing(twist)

            self.pub_cmd_vel.publish(twist)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        node = ControlWheelsNode('control_wheels_node')
        node.run()
    except rospy.ROSInterruptException:
        pass