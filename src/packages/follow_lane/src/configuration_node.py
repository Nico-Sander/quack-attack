#!/usr/bin/env python3

import json
import os
import tkinter as tk
import cv2
import rospy
import util
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage

class ConfigurationNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        
        self.config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))
        self.update_topic = f'/{self._vehicle_name}/update_parameters'
        self.publisher = rospy.Publisher(self.update_topic, String, queue_size=1)

        self.image_subscriber = None
        self.image = None

        self.available_nodes = []
        for file_name in sorted(os.listdir(self.config_dir)):
            if file_name.endswith('.json'):
                node_name = os.path.splitext(file_name)[0].replace('.json', '')
                self.available_nodes.append(node_name)

        self.root = tk.Tk()
        self.root.title('Duckie Configuration')
        self.root.geometry('860x720')
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.selected_node = tk.StringVar(self.root, value=self.available_nodes[0])
        self.selected_group = tk.StringVar(self.root)

        tk.Label(self.root, text='Node').pack(anchor='w', padx=10, pady=(10, 0))
        tk.OptionMenu(self.root, self.selected_node, *self.available_nodes, command=self.change_node).pack(fill='x', padx=10)
        tk.Label(self.root, text='Group').pack(anchor='w', padx=10, pady=(10, 0))
        self.group_dropdown = tk.OptionMenu(self.root, self.selected_group, '')
        self.group_dropdown.pack(fill='x', padx=10)

        self.image_var = tk.StringVar()
        tk.Label(self.root, text='Debug Image').pack(anchor='w', padx=10, pady=(10, 0))
        self.image_dropown = tk.OptionMenu(self.root, self.image_var, '')
        self.image_dropown.pack(fill='x', padx=10, pady=(10, 0))

        self.slider_frame = tk.Frame(self.root)
        self.slider_frame.pack(fill='both', expand=True, padx=10, pady=10)
        self.change_node(self.selected_node.get())

    def select_group(self, group_name):
        self.selected_group.set(group_name)
        self.change_group(group_name)

    def select_image_topic(self, topic_name):
        print(f'changing image topic to {topic_name}')
        self.image_var.set(topic_name)
        if self.image_subscriber:
            self.image_subscriber.unregister()
        topic = topic_name if topic_name.startswith(f'/{self._vehicle_name}/') else f'/{self._vehicle_name}{topic_name}'
        
        print(f'changing image topic to {topic}')
        self.image_subscriber = rospy.Subscriber(topic, CompressedImage, self.update_image, queue_size=1)
        

    def rebuild_group_menu(self):
        groups = list(self.parameters.keys())
        menu = self.group_dropdown['menu']
        menu.delete(0, 'end')
        for group in groups:
            menu.add_command(label=group, command=lambda value=group: self.select_group(value))

        image_menu = self.image_dropown['menu']
        image_menu.delete(0, 'end')
        for topic in util.get_image_topics(self.selected_node.get()):
            image_menu.add_command(label=topic, command=lambda value=topic: self.select_image_topic(value))

        self.select_group(groups[0] if groups else '')
        self.rebuild_sliders()

    def rebuild_sliders(self):
        for widget in self.slider_frame.winfo_children():
            widget.destroy()
        self.sliders = {}
        for name, values in self.parameters.get(self.selected_group.get(), {}).items():
            is_float = isinstance(values['min'], float)


            slider = tk.Scale(self.slider_frame, from_=values['min'], to=values['max'], orient='horizontal', label=name, command=lambda value, param=name: self.update_parameter(param, value), resolution=0.01 if is_float else 1)
            slider.set(values['default'])
            slider.pack(fill='x', pady=4)
            self.sliders[name] = slider

    def change_node(self, *_):
        self.parameters = util.load_parameters(self.selected_node.get())
        self.rebuild_group_menu()

    def change_group(self, *_):
        self.rebuild_sliders()
   
    def update_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow('Debug Image', cv_image)
        cv2.waitKey(1)

    def update_parameter(self, param, value):
        is_float = isinstance(self.parameters[self.selected_group.get()][param]['min'], float)
        self.parameters[self.selected_group.get()][param]['default'] = float(value) if is_float else int(float(value))

        print(f"Updated {param} to {value} in group {self.selected_group.get()} for node {self.selected_node.get()}")
        payload = {'node': self.selected_node.get(), 'parameters': self.parameters}
        self.publisher.publish(String(data=json.dumps(payload)))

    def run(self):
        self.root.mainloop()

    def shutdown(self):
        if self.image_subscriber:
            self.image_subscriber.unregister()
        cv2.destroyAllWindows()
        self.root.destroy()
        #rospy.signal_shutdown('User ended program')


if __name__ == '__main__':
    node = ConfigurationNode('configuration_node')
    node.run()
    #rospy.spin()