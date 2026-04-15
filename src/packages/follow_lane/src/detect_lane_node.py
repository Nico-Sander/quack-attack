#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
import util

#from duckietown.dtros import DTROS, NodeType

class DetectLaneNode:
    def __init__(self, node_name):
        # initialize the ROS node
        rospy.init_node(node_name)
        
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name,self)
                
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)
        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        self._crop_im_size = 400
        self.is_running = False
        self.counter = 0

        # init debug channels 
        self.pub_debug_lane = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_croped',CompressedImage,queue_size=1)
        self.pub_debug_white = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_white',CompressedImage,queue_size=1)
        self.pub_debug_yellow = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_yellow',CompressedImage,queue_size=1)


    def update_parameters(self):
        # Update white line parameters
        self.hue_white_l = self.parameters["white"]["hl"]["default"]
        self.hue_white_h = self.parameters["white"]["hh"]["default"]
        self.saturation_white_l = self.parameters["white"]["sl"]["default"]
        self.saturation_white_h = self.parameters["white"]["sh"]["default"]
        self.lightness_white_l = self.parameters["white"]["vl"]["default"]
        self.lightness_white_h = self.parameters["white"]["vh"]["default"]
        
        # Update yellow line parameters
        self.hue_yellow_l = self.parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h = self.parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = self.parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = self.parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l = self.parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h = self.parameters["yellow"]["vh"]["default"]
        
        # Update perspective transform points
        self.top_left_x = self.parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y = self.parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x = self.parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y = self.parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x = self.parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y = self.parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = self.parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = self.parameters["crop_image"]["bottom_right_y"]["default"]
  
    def crop_img(self,img):
        img = img.copy()
        
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))


    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        _,th1 = cv2.threshold(grad,127,255,cv2.THRESH_BINARY)

        a = []
        for row in range(distance-50, distance+50):
            if np.where(th1[row] == 255)[0].size == 0:
                continue
            else:
                if left_line:
                    a.append(np.where(th1[row] == 255)[0][-1])
                else:
                    a.append(np.where(th1[row] == 255)[0][0])

        if len(a) > 10:
            return np.median(a)
        else:
            return no_lane_value
        

    def cbFindLane(self, image_msg):
        
        if self.counter  <= 3:
            self.counter += 1   
            return

        if self.is_running:
            return
        
        self.is_running = True
        self.conunter = 0
        self.update_parameters()


        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)
        
        white_alternative = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)

        center_white = self.get_x_for_driving(mask_white,int(len(img)*0.75), white_alternative, left_line=True)
        center_yellow = self.get_x_for_driving(mask_yellow,int(len(img)*0.75), yellow_alternative, left_line=False)


        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        
        lane_center = (center_white + center_yellow) / 2

        msg_error = Float64()
        msg_error.data = 1-(lane_center / len(img) * 2)

        self.pub_lane.publish(msg_error)
        print(f"Lane error: {msg_error.data} range [-1,1]")


        # saving for debug
        self.img = img
        self.lane_center = lane_center
        self.white_alternative = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white = center_white
        self.center_yellow = center_yellow

        self.debug_img_white = mask_white
        self.debug_img_yellow = mask_yellow

        
        image = cv2.circle(img,(int(lane_center),int(len(img) / 2)),3,(255,0,0))
        image = cv2.line(image, (white_alternative , 0), (white_alternative , self._crop_im_size) ,color=(255,255,255)) 
        image = cv2.line(image, (yellow_alternative , 0), (yellow_alternative , self._crop_im_size) ,color=(255,255,0))
        image = cv2.line(image, (0,int(len(img) * 0.75) + 100) , (len(img[0]),int(len(img) * 0.75) + 100), color=(255,255,255) )
        image = cv2.line(image, (0,int(len(img) * 0.75) - 100) , (len(img[0]),int(len(img) * 0.75) - 100), color=(255,255,255))

        image = cv2.line(image,(int(len(img[0])/2),0),(int(len(img[0])/2),len(image)),(0,255,0))
        image = cv2.circle(image, (int(center_white), int(len(img) * 0.75)),  5,(255,255,255))
        image = cv2.circle(image, (int(center_yellow), int(len(img) * 0.75)), 5,(0,255,255))


        
        cv2.imshow('lane detection', image)
        self.is_running = False
        
        #cv2.imshow('white', mask_white)
        #cv2.imshow('yellow', mask_yellow)
        cv2.waitKey(1)
            
    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # add debug info to lane image
            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()
                debug_img = cv2.circle(debug_img,(int(self.lane_center),int(len(debug_img) / 2)),3,(255,0,0))

                debug_img = cv2.line(debug_img, (self.white_alternative , 0), (self.white_alternative , 1000) ,color=(255,255,255)) 
                debug_img = cv2.line(debug_img, (self.yellow_alternative , 0), (self.yellow_alternative , 1000) ,color=(255,255,0))
                debug_img = cv2.line(debug_img, (0,int(len(debug_img) * 0.75) + 100) , (len(debug_img[0]),int(len(debug_img) * 0.75) + 100), color=(255,255,255) )
                debug_img = cv2.line(debug_img, (0,int(len(debug_img) * 0.75) - 100) , (len(debug_img[0]),int(len(debug_img) * 0.75) - 100), color=(255,255,255))

                debug_img = cv2.line(debug_img,(int(len(debug_img[0])/2),0),(int(len(debug_img[0])/2),len(debug_img)),(0,255,0))
                debug_img = cv2.circle(debug_img, (int(self.center_white), int(len(debug_img) * 0.75)),  5,(255,255,255))
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), int(len(debug_img) * 0.75)), 5,(0,255,255))

                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
                self.pub_debug_lane.publish(debug_msg)

            if self.pub_debug_white.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_white)[1]).tobytes()
                self.pub_debug_white.publish(debug_msg)

            if self.pub_debug_yellow.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_yellow)[1]).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

            rate.sleep()
        
if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()