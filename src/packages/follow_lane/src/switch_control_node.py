#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum

import os

# External Commands: What the motor controllers should do
class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3

# Internal States: The phases of the brains decision making
class State(Enum):
    LANE_FOLLOWING = 1
    AT_STOP_LINE = 2
    CROSSING_BLIND = 3
    CROSSING_CLEARING = 4


class SwitchControlNode:
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        ## Subscirbers
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Float64, self.cbDuckieDetected, queue_size = 1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size = 1)
        self.sub_intersection = rospy.Subscriber(f"/{self._vehicle_name}/detect/intersection", Bool, self.cbIntersectionDetected, queue_size = 1)

        ## Publishers
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        
        ## State Machine Initialization
        self.state = State.LANE_FOLLOWING

        ## Internal Beliefs (Updated by callbacks)
        self.red_line_visible = False

        ## Timer variable for the non-blocking wait
        self.state_timer = 0.0
        self.stop_duration = 2.0
        self.blind_duration = 1.0

    # ========================================================================
    # Sensor Callbacks (no logic allowed here)
    # ========================================================================

    def cbIntersectionDetected(self, msg):
       self.red_line_visible = msg.data 

    def cbDuckieDetected(self, msg):
        pass

    def cbLaneDetected(self, msg):
        pass

    # ========================================================================
    # Logic 
    # ========================================================================
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # --- 1. State Transitions ---

            if self.state == State.LANE_FOLLOWING:
                # Event: Red Line observed
                if self.red_line_visible:
                    rospy.loginfo("State -> AT_STOP_LINE")
                    self.state = State.AT_STOP_LINE
                    self.state_timer = current_time

            elif self.state == State.AT_STOP_LINE:
                # Event: 2 seconds have passed
                if (current_time - self.state_timer) >= self.stop_duration:
                    rospy.loginfo("State -> CROSSING_BLIND")
                    self.state = State.CROSSING_BLIND
                    self.state_timer = current_time

            elif self.state == State.CROSSING_BLIND:
                # Event: 1 second has passed (acceleration blur settled)
                if (current_time - self.state_timer) >= self.blind_duration:
                    rospy.loginfo("State -> CROSSING_CLEARING")
                    self.state = State.CROSSING_CLEARING

            elif self.state == State.CROSSING_CLEARING:
                # Event: Red line is no longer visible (Falling Edge)
                if not self.red_line_visible:
                    rospy.loginfo("State -> LANE_FOLLOWING (Intersection cleared)")
                    self.state = State.LANE_FOLLOWING

            # --- 2. Action mapping ---
            # Map internal states to external commands

            msg_control = Int32()

            if self.state == State.AT_STOP_LINE:
                msg_control.data = ControlType.Stop.value

            else:
                # For LANE_FOLLOWING, CROSSING_BLIND and CROSSING_CLEARING, duckie should drive
                msg_control.data = ControlType.Lane.value

            self.pub_control.publish(msg_control) 
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()
