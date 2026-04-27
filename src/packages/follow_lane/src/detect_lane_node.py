#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String, Int32
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
import util


class DetectLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic,
            CompressedImage,
            self.cbFindLane,
            queue_size=1
        )

        # Publish -1;1 (FLOAT64): Lane error
        self.pub_lane = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane',
            Float64,
            queue_size=1
        )

        # Publish 3 (INT): Stop line detection
        self.last_stop_publish_time = 0.0
        self.stop_cooldown = 5.0
        control_change_topic = f"/{self._vehicle_name}/switch/control"
        self.pub_control = rospy.Publisher(
            control_change_topic,
            Int32,
            queue_size=1
        )

        self._crop_im_size = 400
        self.is_running = False
        self.counter = 0

        self.stop_line_detected = False

        self.pub_debug_lane = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_croped',
            CompressedImage,
            queue_size=1
        )
        self.pub_debug_white = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_white',
            CompressedImage,
            queue_size=1
        )
        self.pub_debug_yellow = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_yellow',
            CompressedImage,
            queue_size=1
        )
        self.pub_debug_red = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_red',
            CompressedImage,
            queue_size=1
        )

    def cbUpdateParameters(self, parameters):
        self.hue_white_l = parameters["white"]["hl"]["default"]
        self.hue_white_h = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l = parameters["white"]["vl"]["default"]
        self.lightness_white_h = parameters["white"]["vh"]["default"]

        self.hue_yellow_l = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h = parameters["yellow"]["vh"]["default"]

        self.top_left_x = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

    def crop_img(self, img):
        img = img.copy()

        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],
        ])

        pts2 = np.float32([
            [0, 0],
            [self._crop_im_size, 0],
            [0, self._crop_im_size],
            [self._crop_im_size, self._crop_im_size]
        ])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(
            img,
            M,
            (self._crop_im_size, self._crop_im_size)
        )

    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        grad = cv2.Sobel(
            mask,
            cv2.CV_16S,
            1,
            0,
            ksize=3,
            scale=1,
            delta=0,
            borderType=cv2.BORDER_DEFAULT
        )

        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []

        for row in range(distance - 50, distance + 50):
            if row < 0 or row >= mask.shape[0]:
                continue

            xs = np.where(th1[row] == 255)[0]

            if xs.size == 0:
                continue

            if left_line:
                a.append(xs[-1])
            else:
                a.append(xs[0])

        if len(a) > 10:
            return np.median(a)
        else:
            return no_lane_value

    def detect_stop_line(self, mask_red, center_yellow, center_white, img):
        if center_white <= center_yellow:
            return False, None

        x_min = int(min(center_yellow, center_white))
        x_max = int(max(center_yellow, center_white))

        y_center = int(len(img) * 0.75)

        y_min = max(0, y_center - 80)
        y_max = min(img.shape[0], y_center + 80)

        red_roi = mask_red[y_min:y_max, x_min:x_max]

        red_pixels = cv2.countNonZero(red_roi)

        stop_line_detected = red_pixels > 200

        roi_data = {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "red_pixels": red_pixels
        }

        return stop_line_detected, roi_data

    def cbFindLane(self, image_msg):
        if self.counter <= 3:
            self.counter += 1
            return

        if self.is_running:
            return

        self.is_running = True
        self.counter = 0

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(
            hsv,
            (
                self.hue_yellow_l,
                self.saturation_yellow_l,
                self.lightness_yellow_l
            ),
            (
                self.hue_yellow_h,
                self.saturation_yellow_h,
                self.lightness_yellow_h
            )
        )

        mask_white = cv2.inRange(
            hsv,
            (
                self.hue_white_l,
                self.saturation_white_l,
                self.lightness_white_l
            ),
            (
                self.hue_white_h,
                self.saturation_white_h,
                self.lightness_white_h
            )
        )

        # Rot-Erkennung in HSV.
        # Rot liegt am Anfang UND am Ende des Hue-Bereichs.
        mask_red_1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        mask_red_2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

        white_alternative = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)

        y_check = int(len(img) * 0.75)

        center_white = self.get_x_for_driving(
            mask_white,
            y_check,
            white_alternative,
            left_line=True
        )

        center_yellow = self.get_x_for_driving(
            mask_yellow,
            y_check,
            yellow_alternative,
            left_line=False
        )

        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        lane_center = (center_white + center_yellow) / 2

        msg_error = Float64()
        msg_error.data = 1 - (lane_center / len(img) * 2)

        self.pub_lane.publish(msg_error)

        stop_line_detected, roi_data = self.detect_stop_line(
            mask_red,
            center_yellow,
            center_white,
            img
        )

        self.stop_line_detected = stop_line_detected
        self.stop_line_roi = roi_data

        current_time = rospy.Time.now().to_sec()

        if stop_line_detected and current_time - self.last_stop_publish_time > self.stop_cooldown:
            msg_control = Int32()
            msg_control.data = 3
            self.pub_control.publish(msg_control)

            self.last_stop_publish_time = current_time
            print("Stop line detected -> published 3 on /switch/control")

        print(
            f"Lane error: {msg_error.data} range [-1,1], "
            f"Stop line: {stop_line_detected}"
        )

        self.img = img
        self.lane_center = lane_center
        self.white_alternative = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white = center_white
        self.center_yellow = center_yellow

        self.debug_img_white = mask_white
        self.debug_img_yellow = mask_yellow
        self.debug_img_red = mask_red

        image = img.copy()

        image = cv2.circle(
            image,
            (int(lane_center), int(len(img) / 2)),
            3,
            (255, 0, 0),
            -1
        )

        image = cv2.line(
            image,
            (white_alternative, 0),
            (white_alternative, self._crop_im_size),
            color=(255, 255, 255)
        )

        image = cv2.line(
            image,
            (yellow_alternative, 0),
            (yellow_alternative, self._crop_im_size),
            color=(255, 255, 0)
        )

        image = cv2.line(
            image,
            (0, int(len(img) * 0.75) + 100),
            (len(img[0]), int(len(img) * 0.75) + 100),
            color=(255, 255, 255)
        )

        image = cv2.line(
            image,
            (0, int(len(img) * 0.75) - 100),
            (len(img[0]), int(len(img) * 0.75) - 100),
            color=(255, 255, 255)
        )

        image = cv2.line(
            image,
            (int(len(img[0]) / 2), 0),
            (int(len(img[0]) / 2), len(image)),
            (0, 255, 0)
        )

        image = cv2.circle(
            image,
            (int(center_white), y_check),
            5,
            (255, 255, 255),
            -1
        )

        image = cv2.circle(
            image,
            (int(center_yellow), y_check),
            5,
            (0, 255, 255),
            -1
        )

        if roi_data is not None:
            image = cv2.rectangle(
                image,
                (roi_data["x_min"], roi_data["y_min"]),
                (roi_data["x_max"], roi_data["y_max"]),
                (0, 0, 255),
                2
            )

            cv2.putText(
                image,
                f"red: {roi_data['red_pixels']}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if stop_line_detected:
            cv2.putText(
                image,
                "STOP LINE",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                3
            )

        cv2.imshow('lane detection', image)

        self.is_running = False
        cv2.waitKey(1)

    def run_debug(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if hasattr(self, "img") and self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()

                debug_img = cv2.circle(
                    debug_img,
                    (int(self.lane_center), int(len(debug_img) / 2)),
                    3,
                    (255, 0, 0),
                    -1
                )

                debug_img = cv2.line(
                    debug_img,
                    (self.white_alternative, 0),
                    (self.white_alternative, 1000),
                    color=(255, 255, 255)
                )

                debug_img = cv2.line(
                    debug_img,
                    (self.yellow_alternative, 0),
                    (self.yellow_alternative, 1000),
                    color=(255, 255, 0)
                )

                debug_img = cv2.line(
                    debug_img,
                    (0, int(len(debug_img) * 0.75) + 100),
                    (len(debug_img[0]), int(len(debug_img) * 0.75) + 100),
                    color=(255, 255, 255)
                )

                debug_img = cv2.line(
                    debug_img,
                    (0, int(len(debug_img) * 0.75) - 100),
                    (len(debug_img[0]), int(len(debug_img) * 0.75) - 100),
                    color=(255, 255, 255)
                )

                debug_img = cv2.line(
                    debug_img,
                    (int(len(debug_img[0]) / 2), 0),
                    (int(len(debug_img[0]) / 2), len(debug_img)),
                    (0, 255, 0)
                )

                debug_img = cv2.circle(
                    debug_img,
                    (int(self.center_white), int(len(debug_img) * 0.75)),
                    5,
                    (255, 255, 255),
                    -1
                )

                debug_img = cv2.circle(
                    debug_img,
                    (int(self.center_yellow), int(len(debug_img) * 0.75)),
                    5,
                    (0, 255, 255),
                    -1
                )

                if self.stop_line_roi is not None:
                    debug_img = cv2.rectangle(
                        debug_img,
                        (
                            self.stop_line_roi["x_min"],
                            self.stop_line_roi["y_min"]
                        ),
                        (
                            self.stop_line_roi["x_max"],
                            self.stop_line_roi["y_max"]
                        ),
                        (0, 0, 255),
                        2
                    )

                if self.stop_line_detected:
                    cv2.putText(
                        debug_img,
                        "STOP LINE",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        3
                    )

                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(
                    cv2.imencode('.jpg', debug_img)[1]
                ).tobytes()

                self.pub_debug_lane.publish(debug_msg)

            if hasattr(self, "debug_img_white") and self.pub_debug_white.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(
                    cv2.imencode('.jpg', self.debug_img_white)[1]
                ).tobytes()
                self.pub_debug_white.publish(debug_msg)

            if hasattr(self, "debug_img_yellow") and self.pub_debug_yellow.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(
                    cv2.imencode('.jpg', self.debug_img_yellow)[1]
                ).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

            if hasattr(self, "debug_img_red") and self.pub_debug_red.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(
                    cv2.imencode('.jpg', self.debug_img_red)[1]
                ).tobytes()
                self.pub_debug_red.publish(debug_msg)

            rate.sleep()


if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()