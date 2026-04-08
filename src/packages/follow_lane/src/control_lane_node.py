#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, String

from duckietown_msgs.msg import Twist2DStamped
import os
from switch_control_node import ControlType
from dynamic_reconfigure.server import Server
from follow_lane.cfg import ControlLaneConfig
import yaml

class ControlLaneNode:
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.enable = True

        # Setup dynamic reconfigure server
        self.srv = Server(ControlLaneConfig, self.reconfigure_callback)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size = 1)

        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.sub_control = rospy.Subscriber(control_change_topic, Int32, self.cbControl , queue_size = 1)
 
        self.lastError = 0
        self.v = 0
        self.a = 0
        rospy.on_shutdown(self.fnShutDown)

    def reconfigure_callback(self, config, level):
        """Dynamic reconfigure callback"""
        print(f"Reconfigure Request: {config}")
        # Update white line parameters
        self.kp = config.p
        self.ki = config.i
        self.kd = config.d
        self.MAX_VEL = config.max_vel
        return config


    def cbControl(self,msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
        
        else:
            self.enable = False

    # error between 1 and -1
    def cbFollowLane(self, error):

        print(f'received message. enabled : {self.enable}')
        if not self.enable:
            return        
        error = error.data

        #Todo Write own code for PID controller here
        self.v = 0
        self.a = 0                
        

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            print(f'{rospy.Time.now()} \n\n {rospy.get_time()}')
            twist.header.stamp = rospy.Time.now()
            
            twist.v = self.v
            twist.omega = self.a

            self.pub_cmd_vel.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ControlLaneNode('control_lane_node')
    node.run()