#!/usr/bin/env python3

import rospy
import os
from enum import Enum
from std_msgs.msg import Int32, String
from duckietown_msgs.msg import Twist2DStamped


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Stop = 3
    Crossing = 4


class TurnDirection(Enum):
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3


class CrossingPhase(Enum):
    STRAIGHT_BEFORE_TURN = 1
    INITIAL_TURNING = 2


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self.active = False
        self.turn_direction = TurnDirection.RIGHT

        self.phase = CrossingPhase.STRAIGHT_BEFORE_TURN
        self.phase_start_time = 0.0

        # Straight (before turn)
        self.straight_before_turn_v = 0.15
        self.straight_before_turn_omega = 0.0

        self.straight_before_turn_duration_left = 0.0
        self.straight_before_turn_duration_right = 0.8
        self.straight_before_turn_duration_straight = 0.0

        # Initial turn movements
        self.initial_turn_left_v = 0.3
        self.initial_turn_left_omega = 1.25

        self.initial_turn_right_v = 0.25
        self.initial_turn_right_omega = -3.0

        self.initial_turn_straight_v = 0.25
        self.initial_turn_straight_omega = 0.0

        self.pub_cmd_vel = rospy.Publisher(
            f"/{self._vehicle_name}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )

        self.sub_control = rospy.Subscriber(
            f"/{self._vehicle_name}/switch/control",
            Int32,
            self.cbControl,
            queue_size=1
        )

        self.sub_direction = rospy.Subscriber(
            f"/{self._vehicle_name}/switch/turn_direction",
            String,
            self.cbDirection,
            queue_size=1
        )

        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self, msg):
        if msg.data == ControlType.Crossing.value:
            if not self.active:
                self.phase = CrossingPhase.STRAIGHT_BEFORE_TURN
                self.phase_start_time = rospy.Time.now().to_sec()

            self.active = True
        else:
            self.active = False

    def cbDirection(self, msg):
        if msg.data == "left":
            self.turn_direction = TurnDirection.LEFT
        elif msg.data == "straight":
            self.turn_direction = TurnDirection.STRAIGHT
        elif msg.data == "right":
            self.turn_direction = TurnDirection.RIGHT

    def getPreTurnDuration(self):
        if self.turn_direction == TurnDirection.LEFT:
            return self.straight_before_turn_duration_left
        elif self.turn_direction == TurnDirection.RIGHT:
            return self.pre_turn_duration_right
        else:
            return self.pre_turn_duration_straight

    def fnShutDown(self):
        rospy.loginfo("Shutting down intersection controller.")
        twist = Twist2DStamped()
        twist.v = 0.0
        twist.omega = 0.0
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.active:
                current_time = rospy.Time.now().to_sec()

                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()

                if self.phase == CrossingPhase.STRAIGHT_BEFORE_TURN:
                    twist.v = self.straight_before_turn_v
                    twist.omega = self.straight_before_turn_omega

                    if (current_time - self.phase_start_time) >= self.getPreTurnDuration():
                        self.phase = CrossingPhase.INITIAL_TURNING
                        self.phase_start_time = current_time

                elif self.phase == CrossingPhase.INITIAL_TURNING:
                    if self.turn_direction == TurnDirection.LEFT:
                        twist.v = self.initial_turn_left_v
                        twist.omega = self.initial_turn_left_omega

                    elif self.turn_direction == TurnDirection.RIGHT:
                        twist.v = self.initial_turn_right_v
                        twist.omega = self.initial_turn_right_omega

                    else:
                        twist.v = self.initial_turn_straight_v
                        twist.omega = self.initial_turn_straight_omega

                self.pub_cmd_vel.publish(twist)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlIntersectionNode("control_intersection_node")
        node.run()
    except rospy.ROSInterruptException:
        pass