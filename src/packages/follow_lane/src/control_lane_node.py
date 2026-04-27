#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String

from duckietown_msgs.msg import Twist2DStamped
import os
from switch_control_node import ControlType
import yaml
import util


class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.enable = True

        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(
            twist_topic,
            Twist2DStamped,
            queue_size=1
        )

        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(
            detect_lane_topic,
            Float64,
            self.cbFollowLane,
            queue_size=1
        )

        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.sub_control = rospy.Subscriber(
            control_change_topic,
            Int32,
            self.cbControl,
            queue_size=1
        )

        self.lastError = 0
        self.v = 0.0
        self.a = 0.0

        self.integral = 0.0
        self.last_time = None

        self.is_stopping = False
        self.stop_until = None

        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self, msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
            self.is_stopping = False
            self.stop_until = None

        elif msg.data == ControlType.Stop.value:
            if not self.is_stopping:
                rospy.loginfo("Stop line detected. Stopping for 2 seconds.")

                self.enable = False
                self.is_stopping = True
                self.stop_until = rospy.Time.now().to_sec() + 2.0

                self.v = 0.0
                self.a = 0.0

                self.integral = 0.0
                self.last_time = None

        else:
            self.enable = False
            self.is_stopping = False
            self.stop_until = None

            self.v = 0.0
            self.a = 0.0

    def cbUpdateParameters(self, parameters):
        self.kp = parameters["pid"]["p"]["default"]
        self.ki = parameters["pid"]["i"]["default"]
        self.kd = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]

    def cbFollowLane(self, error):
        error = error.data

        if not self.enable or self.is_stopping:
            self.v = 0.0
            self.a = 0.0
            return

        current_time = rospy.Time.now().to_sec()

        if self.last_time is None:
            self.last_time = current_time
            self.lastError = error
            return

        dt = current_time - self.last_time

        if dt <= 0.0:
            return

        p_term = self.kp * error

        self.integral += error * dt

        max_integral = 1.0
        self.integral = max(
            min(self.integral, max_integral),
            -max_integral
        )

        i_term = self.ki * self.integral

        d_term = self.kd * ((error - self.lastError) / dt)

        omega = p_term + i_term + d_term

        velocity = self.MAX_VEL * (1.0 - (abs(error) * 0.8))
        velocity = max(velocity, 0.04)

        self.v = velocity
        self.a = omega

        self.lastError = error
        self.last_time = current_time

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped()
        twist.v = 0.0
        twist.omega = 0.0
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            twist.header.stamp = rospy.Time.now()

            if self.is_stopping:
                twist.v = 0.0
                twist.omega = 0.0
                self.pub_cmd_vel.publish(twist)

                if rospy.Time.now().to_sec() >= self.stop_until:
                    rospy.loginfo("Stop finished. Continue lane following.")

                    self.is_stopping = False
                    self.enable = True
                    self.stop_until = None

                    self.integral = 0.0
                    self.last_time = None

                rate.sleep()
                continue

            if self.enable:
                twist.v = self.v
                twist.omega = self.a
            else:
                twist.v = 0.0
                twist.omega = 0.0

            self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()