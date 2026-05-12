#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String

from duckietown_msgs.msg import Twist2DStamped
import os
from custom_enums import ControlType, IntersectionPhase, TurnDirection
import yaml
import util

class ControlLaneNode:
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.enable = True
        self.stopping = False
        self.drive_intersection = False

        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size = 1)

        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.sub_control = rospy.Subscriber(control_change_topic, Int32, self.cbControl , queue_size = 1)

        direction_topic = f"/{self._vehicle_name}/switch/turn_direction"
        self.sub_turn_direction = rospy.Subscriber(direction_topic, Int32, self.cbDirection, queue_size = 1)
 
        self.lastError = 0
        self.v = 0
        self.a = 0

        self.integral = 0.0           
        self.last_time = None

        self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
        self.intersection_phase_start_time = rospy.Time.now().to_sec()
        self.turn_direction = TurnDirection.STRAIGHT

        # Intersection driving values
        self.straight_before_turn_v = 0.15
        self.straight_before_turn_omega = 0.0
        self.straight_before_turn_duration_left = 0.0
        self.straight_before_turn_duration_right = 0.25
        self.straight_before_turn_duration_straight = 0.0

        self.initial_turn_left_v = 0.3
        self.initial_turn_left_omega = 1.25

        self.initial_turn_right_v = 0.25
        self.initial_turn_right_omega = -3.5

        self.initial_turn_straight_v = 0.25
        self.initial_turn_straight_omega = 0.0

        rospy.on_shutdown(self.fnShutDown)
    
    def cbUpdateParameters(self, parameters):
        self.kp = parameters["pid"]["p"]["default"]
        self.ki = parameters["pid"]["i"]["default"]
        self.kd = parameters["pid"]["d"]["default"]
        
        # Save the default velocity separately so we can restore it
        self.base_max_vel = parameters["pid"]["max_vel"]["default"]
        self.MAX_VEL = self.base_max_vel

    def cbDirection(self, msg):
        try:
            self.turn_direction = TurnDirection(msg.data)
        except ValueError:
            rospy.logwarn(f"Unknown turn direction: {msg.data}")
            self.turn_direction = TurnDirection.STRAIGHT

    def cbControl(self, msg):
        if msg.data == ControlType.LANE_FOLLOWING.value:
            self.enable = True
            self.stopping = False
            self.drive_intersection = False
            self.MAX_VEL = self.base_max_vel # Normal speed

        elif msg.data == ControlType.FIND_STOP_LINE.value:
            self.enable = True
            self.stopping = False
            self.drive_intersection = False
            self.MAX_VEL = self.base_max_vel * 0.3 # Slow down for approach

        elif msg.data == ControlType.STOP.value:
            self.enable = False
            self.stopping = True
            self.drive_intersection = False
            self.v = 0.0
            self.a = 0.0

        elif msg.data == ControlType.DRIVE_INTERSECTION.value:
            # ONLY initialize the timers and phase if we are entering 
            # this mode for the first time
            if not self.drive_intersection:
                self.intersection_phase = IntersectionPhase.STRAIGHT_BEFORE_TURN
                self.intersection_phase_start_time = rospy.Time.now().to_sec()

            self.enable = False
            self.stopping = False
            self.drive_intersection = True
            self.v = 0.0
            self.a = 0.0

    # error between 1 and -1
    def cbFollowLane(self, error):
        error = error.data

        if not self.enable:
            self.v = 0.0
            self.a = 0.0
            return

        current_time = rospy.Time.now().to_sec()
        
        if self.last_time is None:
            self.last_time = current_time
            self.lastError = error
            # Set an initial velocity so it starts moving immediately
            self.v = self.MAX_VEL 
            return
            
        dt = current_time - self.last_time
        
        # If dt is too small, skip the PID calculation to avoid division by zero,
        # but DO NOT return out of the function. Just keep the old self.v and self.a
        if dt > 0.0:
            # ==========================================
            # PID BERECHNUNG
            # ==========================================
            p_term = self.kp * error

            self.integral += error * dt
            max_integral = 1.0  
            self.integral = max(min(self.integral, max_integral), -max_integral)
            
            i_term = self.ki * self.integral
            d_term = self.kd * ((error - self.lastError) / dt)

            omega = p_term + i_term + d_term
            omega = max(min(omega, 5.0), -5.0)

            # ==========================================
            # GESCHWINDIGKEIT (LINEAR VELOCITY)
            # ==========================================
            # Use your dynamic velocity logic
            velocity = self.MAX_VEL * (1.0 - (abs(error) * 0.7))
            velocity = self.MAX_VEL
            velocity = max(velocity, 0.04) 
            # rospy.loginfo(f"Err: {error}, Vel: {velocity}")

            # Update the class variables
            self.v = velocity
            self.a = omega

        # ==========================================
        # STATE UPDATE (MUST HAPPEN EVERY FRAME)
        # ==========================================
        self.lastError = error
        self.last_time = current_time

    def driveIntersection(self, twist):

        current_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"Intersection: {self.turn_direction}, Intersection Phase: {self.intersection_phase}")

        # =========================================================
        # STRAIGHT BEFORE TURN
        # =========================================================
        if self.intersection_phase == IntersectionPhase.STRAIGHT_BEFORE_TURN:

            twist.v = self.straight_before_turn_v
            twist.omega = self.straight_before_turn_omega

            rospy.loginfo(f"{current_time} - {self.intersection_phase_start_time} = {current_time - self.intersection_phase_start_time} >= {self.getStraightBeforeTurnDuration()} => {(current_time - self.intersection_phase_start_time) >= self.getStraightBeforeTurnDuration()}")
            if (current_time - self.intersection_phase_start_time) >= self.getStraightBeforeTurnDuration():
                self.intersection_phase = IntersectionPhase.INITIAL_TURNING
                self.intersection_phase_start_time = current_time

        # =========================================================
        # INITIAL TURNING
        # =========================================================
        elif self.intersection_phase == IntersectionPhase.INITIAL_TURNING:

            if self.turn_direction == TurnDirection.LEFT:
                twist.v = self.initial_turn_left_v
                twist.omega = self.initial_turn_left_omega

            elif self.turn_direction == TurnDirection.RIGHT:
                twist.v = self.initial_turn_right_v
                twist.omega = self.initial_turn_right_omega

            else:
                twist.v = self.initial_turn_straight_v
                twist.omega = self.initial_turn_straight_omega
        
    def getStraightBeforeTurnDuration(self):
        if self.turn_direction == TurnDirection.LEFT:
            return self.straight_before_turn_duration_left

        elif self.turn_direction == TurnDirection.RIGHT:
            return self.straight_before_turn_duration_right

        else:
            return self.straight_before_turn_duration_straight

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            twist = Twist2DStamped()
            twist.header.stamp = rospy.Time.now()

            # =====================================================
            # LANE FOLLOWING
            # =====================================================
            if self.enable:

                twist.v = self.v
                twist.omega = self.a

                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("Following Lane")

            # =====================================================
            # STOP
            # =====================================================
            elif self.stopping:

                twist.v = 0.0
                twist.omega = 0.0

                self.pub_cmd_vel.publish(twist)
                rospy.loginfo("Stopped")

            # =====================================================
            # DRIVE INTERSECTION
            # =====================================================
            elif self.drive_intersection:

                self.driveIntersection(twist)

                self.pub_cmd_vel.publish(twist)

            rate.sleep()
            
if __name__ == '__main__':
    try:
        # create the node
        node = ControlLaneNode('control_lane_node')
        node.run()
    except rospy.ROSInterruptException:
        # Catch Ctrl+C silently
        pass
    