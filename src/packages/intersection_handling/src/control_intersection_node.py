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


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ["VEHICLE_NAME"]

        self.active = False
        self.turn_direction = TurnDirection.RIGHT

        # Startwerte, später kalibrieren
        self.v_left = 0.15
        self.omega_left = 3.0

        self.v_right = 0.15
        self.omega_right = -3.0

        self.v_straight = 0.18
        self.omega_straight = 0.0

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
        self.active = msg.data == ControlType.Crossing.value

    def cbDirection(self, msg):
        if msg.data == "left":
            self.turn_direction = TurnDirection.LEFT
        elif msg.data == "straight":
            self.turn_direction = TurnDirection.STRAIGHT
        elif msg.data == "right":
            self.turn_direction = TurnDirection.RIGHT

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
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()

                if self.turn_direction == TurnDirection.LEFT:
                    twist.v = self.v_left
                    twist.omega = self.omega_left

                elif self.turn_direction == TurnDirection.RIGHT:
                    twist.v = self.v_right
                    twist.omega = self.omega_right

                else:
                    twist.v = self.v_straight
                    twist.omega = self.omega_straight

                self.pub_cmd_vel.publish(twist)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlIntersectionNode("control_intersection_node")
        node.run()
    except rospy.ROSInterruptException:
        pass