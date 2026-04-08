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
        
        #self.load_conf('/home/ubuntu/DuckieRace_2026/src/packages/follow_lane/config/detect_lane.yaml')
        #self.sub_config = rospy.Subscriber(f"/{self._vehicle_name}/conf", String, self.cbUpdateConf, queue_size = 1)
        
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

    def cbFollowLane(self, desired_center):

        print(f'received message. enabled : {self.enable}')

        if not self.enable:
            return        
        
        center = desired_center.data
        self.followLane(center)

    
    # error between 1 and -1
    def followLane(self, error):
        error = -error

        Kp = self.kp
        Kd = self.kd

        print(f'error {error}')

        a = -(Kp * error + Kd * (error - self.lastError) )
        self.lastError = error
        
        # twist.linear.x = 0.05        
        v = min(self.MAX_VEL*100 * ((1 - abs(error)) ** 2), self.MAX_VEL)
        self.v = v
        self.a = a                
        

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

    def load_conf(self,path):
        with open(path,'r') as f:
            conf_yml = f.read()
        
        self.update_conf(conf_yml)
    def cbUpdateConf(self,conf_msg):
        self.update_conf(conf_msg.data)
    
    def update_conf(self,conf_yml):
        self.conf = yaml.safe_load(conf_yml)
        self.MAX_VEL = self.conf['pid_regler']['max_vel'] /100
        self.kp = self.conf['pid_regler']['p'] /10
        self.kd = self.conf['pid_regler']['d'] /10
        self.ki = self.conf['pid_regler']['i'] /10

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            print(f'{rospy.Time.now()} \n\n {rospy.get_time()}')
            twist.header.stamp = rospy.Time.now()
            
            twist.v = self.v
            twist.omega = self.a

            #print(f'type: {type(self.pub_cmd_vel)}')
            #print(f'topic: {self.pub_cmd_vel.name}')
            #print(f'message type: {self.pub_cmd_vel.data_class} actualy type{type(twist)}')
            #print(f'max Vel: {self.MAX_VEL}')
            #print(f'pid {self.kp}, {self.ki}, {self.kd}')
            #print(f'message was: {twist}')
            self.pub_cmd_vel.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ControlLaneNode('control_lane_node')
    node.run()