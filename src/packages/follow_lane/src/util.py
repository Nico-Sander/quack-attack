#!/usr/bin/env python3

import json
import os
import rospy
from std_msgs.msg import String
#import rospy

def init_parameters(node_name, node):
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)

    node.node_name = node_name
    node.parameters = config['parameters']
    node.cb_update_parameters = cb_update_parameters.__get__(node)  # Bind the method to the node instance
    rospy.Subscriber(f'/{node._vehicle_name}/update_parameters', String, node.cb_update_parameters, queue_size=1)

def cb_update_parameters(self,parameters_string):
    msg = json.loads(parameters_string.data)
    if msg['node'] == self.node_name:
        self.parameters = msg['parameters']
        print(f"Received new parameters for {self.node_name}: {self.parameters}")
    
def load_parameters(node_name):
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)
    return config['parameters']
    

def get_image_topics(node_name):
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)
    return config['debug_image_topics']
