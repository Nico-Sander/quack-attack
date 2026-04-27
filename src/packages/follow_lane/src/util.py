#!/usr/bin/env python3

import json
import os
import rospy
from std_msgs.msg import String

def init_parameters(node_name, callback_update_parameters):
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)

    def callback_wrapper(msg):
        msg = json.loads(msg.data)
        if msg['node'] == node_name:
            parameters = msg['parameters']
            print(f"Received new parameters for {node_name}: {parameters}")
            callback_update_parameters(parameters) 
    
    callback_update_parameters(config['parameters'])
    vehicle_name = os.environ['VEHICLE_NAME']
    rospy.Subscriber(f'/{vehicle_name}/update_parameters', String, callback_wrapper, queue_size=1)   
    
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
