#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class DataCollectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        
        # Directory to save images
        self.save_dir = os.path.expanduser("/workspace/data/images/004_complex_track_lighted")
        os.makedirs(self.save_dir, exist_ok=True)
        # Make the directory readable/writable/executable by everyone
        # so the host user can manage files inside it.
        os.chmod(self.save_dir, 0o777)
        
        self.image_count = 0
        self.latest_msg = None
        
        # Subscribe to the camera
        self.sub_cam = rospy.Subscriber(
            f"/{self.vehicle_name}/camera_node/image/compressed", 
            CompressedImage, 
            self.cbImage, 
            queue_size=1
        )
        
        rospy.loginfo(f"[{node_name}] Initialized. Saving 2 frames per second to {self.save_dir}")

    def cbImage(self, msg):
        self.latest_msg = msg

    def run(self):
        # 2 Hz = Save an image every 0.5 seconds
        rate = rospy.Rate(1) 
        
        while not rospy.is_shutdown():
            if self.latest_msg is not None:
                # Decode image
                np_arr = np.frombuffer(self.latest_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Save full frame (Matches the raw dataset format)
                filename = os.path.join(self.save_dir, f"duckie_custom_{self.image_count:04d}.jpg")
                cv2.imwrite(filename, cv_image)
                rospy.loginfo(f"Saved: {filename}")
                
                self.image_count += 1
                self.latest_msg = None # Wait for the next fresh frame
                
            rate.sleep()

if __name__ == "__main__":
    node = DataCollectionNode("data_collection_node")
    node.run()