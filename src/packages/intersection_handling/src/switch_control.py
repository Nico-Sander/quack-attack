#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool, String
from enum import Enum
from detect_intersection import IntersectionState

import os

# External Commands: What the motor controllers should do
class ControlType(Enum):
    LANE_FOLLOWING = 1
    FIND_STOP_LINE = 2
    STOP = 3
    DRIVE_INTERSECTION = 4

# Internal States: The phases of the brains decision making
class State(Enum):
    LANE_FOLLOWING = 1
    APPROACHING = 2
    STOPPED = 3
    CROSSING = 4
    CLEARING = 5

# 
class TurnDirection(Enum):
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3

class SwitchControlNode:
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.current_intersection_state = IntersectionState.NO_INTERSECTION

        ## Subscirbers
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Float64, self.cbDuckieDetected, queue_size = 1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size = 1)
        self.sub_intersection = rospy.Subscriber(f"/{self._vehicle_name}/detect/intersection", Int32, self.cbIntersectionDetected, queue_size = 1)
        self.sub_sign = rospy.Subscriber(f"/{self._vehicle_name}/detect/sign", Int32, self.cbSignDetected, queue_size=1
)

        ## Publishers
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        self.pub_turn_direction = rospy.Publisher(f"/{self._vehicle_name}/switch/turn_direction", String, queue_size=1)

        
        ## State Machine Initialization
        self.state = State.LANE_FOLLOWING

        ## Internal Beliefs (Updated by callbacks)
        # self.red_line_visible = False

        ## Timer variable for the non-blocking wait
        self.state_timer = 0.0
        self.stop_duration = 3.0

        ## Timer variavle for crossing phases
        self.ignore_red_line_until = 0.0
        self.red_line_ignore_duration = 4.0     # This coolddown is crucial

        self.left_turn_duration = 2.0
        self.right_turn_duration = 1.2
        self.straight_duration = 2.0

        # TODO for testing purposes, always turn right at the intersection
        self.turn_direction = TurnDirection.LEFT
        

    # ========================================================================
    # Sensor Callbacks (no logic allowed here)
    # ========================================================================

    def cbIntersectionDetected(self, msg):
        try:
            # This looks up the Enum member by its value (0, 1, or 2)
            self.current_intersection_state = IntersectionState(msg.data)
            rospy.loginfo(f"State updated to: {self.current_intersection_state.name}")
        except ValueError:
            rospy.logwarn(f"Received invalid intersection state value: {msg.data}")

    def cbSignDetected(self, msg):
        # TODO: Implement sign detection and turn direction logic
        # Input: INT Identifier of the detected sign
        # Decision / random generation of TurnDirection

        # for testing purposes, always turn right at the intersection
        self.turn_direction = TurnDirection.LEFT
        pass

    def cbDuckieDetected(self, msg):
        pass

    def cbLaneDetected(self, msg):
        pass

    # ========================================================================
    # Logic 
    # ========================================================================
    def publishTurnDirection(self):
        msg_dir = String()

        if self.turn_direction == TurnDirection.LEFT:
            msg_dir.data = "left"
        elif self.turn_direction == TurnDirection.STRAIGHT:
            msg_dir.data = "straight"
        else:
            msg_dir.data = "right"

        self.pub_turn_direction.publish(msg_dir)
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # --- 1. State Transitions ---
            if self.state == State.LANE_FOLLOWING:
                # Only listen to vision if our "clearing" cooldown has expired
                if current_time >= self.ignore_red_line_until:
                    if self.current_intersection_state == IntersectionState.APPROACHING_INTERSECTION:
                        rospy.loginfo("Vision: Approaching -> State: APPROACHING")
                        self.state = State.APPROACHING

                    elif self.current_intersection_state == IntersectionState.AT_INTERSECTION:
                        # Edge case: If the camera skipped approaching and went straight to the line
                        rospy.loginfo("Vision: At Line -> State: STOPPED")
                        self.state = State.STOPPED
                        self.state_timer = current_time
                

            elif self.state == State.APPROACHING:
                if self.current_intersection_state == IntersectionState.AT_INTERSECTION:
                    rospy.loginfo("Vision: At Line -> State: STOPPED")
                    self.state = State.STOPPED
                    self.state_timer = current_time

            elif self.state == State.STOPPED:
                # Wait for the stop_duration (e.g., 2 seconds)
                if (current_time - self.state_timer) >= self.stop_duration:
                    rospy.loginfo("Timer finished -> State: CROSSING")
                    self.state = State.CROSSING
                    self.state_timer = current_time
                    
                    # Set the duration for the open-loop turn
                    if self.turn_direction == TurnDirection.LEFT:
                        self.turn_duration = self.left_turn_duration
                    elif self.turn_direction == TurnDirection.RIGHT:
                        self.turn_duration = self.right_turn_duration
                    else:
                        self.turn_duration = self.straight_duration                 

            elif self.state == State.CROSSING:
                # Wait for the turn to complete
                if (current_time - self.state_timer) >= self.turn_duration:
                    rospy.loginfo("Turn finished -> State: CLEARING")
                    self.state = State.CLEARING
                    # Set a cooldown timer so we don't immediately see the red line we just crossed
                    self.ignore_red_line_until = current_time + self.red_line_ignore_duration

            elif self.state == State.CLEARING:
                # We are driving via Lane Following, but actively ignoring red lines
                if current_time >= self.ignore_red_line_until:
                    rospy.loginfo("Cooldown finished -> State: LANE_FOLLOWING")
                    self.state = State.LANE_FOLLOWING

            # --- 2. Action mapping ---
            # Map internal states to external commands

            msg_control = Int32()

            if self.state in (State.LANE_FOLLOWING, State.CLEARING):
                msg_control.data = ControlType.LANE_FOLLOWING.value
            
            elif self.state == State.APPROACHING:
                msg_control.data = ControlType.FIND_STOP_LINE.value
                
            elif self.state == State.STOPPED:
                msg_control.data = ControlType.STOP.value

            elif self.state == State.CROSSING:
                msg_control.data = ControlType.DRIVE_INTERSECTION.value

            self.pub_control.publish(msg_control) 
            self.publishTurnDirection()
            rate.sleep()
            
if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()