#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

def image_callback(msg):
    try:
        # Convert the byte array into a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Decode the numpy array into an OpenCV image
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Display the image in a cv2 window
        cv2.imshow('Duckiebot Camera View', image_np)
        
        # Required for cv2 to update the window and process GUI events
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr(f"Failed to process image: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('image_visualizer', anonymous=True)
    
    # Subscribe to the specific compressed image topic
    rospy.Subscriber(
        '/trick/camera_node/image/compressed', 
        CompressedImage, 
        image_callback,
        queue_size=1
    )
    
    rospy.loginfo("Image visualizer node started. Waiting for images...")
    
    # Keep python from exiting until this node is stopped
    rospy.spin()
    
    # Cleanup windows gracefully on shutdown
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()