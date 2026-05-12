#!/usr/bin/env python3

import os
import json
import yaml
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class DetectIntersectionSignNode:
    def __init__(self):
        rospy.init_node("detect_intersection_sign_node")

        vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        node_dir = os.path.dirname(os.path.abspath(__file__))

        self.db_path = rospy.get_param("~apriltags_db_path",
                                       os.path.join(node_dir, "apriltagsDB.yaml"))

        self.image_topic = rospy.get_param("~image_topic",
                                           "/{}/camera_node/image/compressed".format(vehicle))

        self.output_topic = rospy.get_param("~output_topic",
                                            "/{}/detected_sign".format(vehicle))

        self.cooldown = float(rospy.get_param("~cooldown", 1.0))
        self.last_seen = {}

        self.db = self.load_db(self.db_path)

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()

        self.pub = rospy.Publisher(self.output_topic, String, queue_size=1)
        self.sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("detect_intersection_sign_node started")
        rospy.loginfo("DB:  %s", self.db_path)
        rospy.loginfo("IN:  %s", self.image_topic)
        rospy.loginfo("OUT: %s", self.output_topic)
        rospy.loginfo("Loaded %d tags", len(self.db))

    def load_db(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        db = {}
        for e in data:
            tag_id = int(e["tag_id"])
            db[tag_id] = {
                "tag_type": e.get("tag_type") or "",
                "traffic_sign_type": e.get("traffic_sign_type") or "",
                "street_name": e.get("street_name") or "",
                "vehicle_name": e.get("vehicle_name") or "",
            }

        return db

    def cb_image(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        gray = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

        if gray is None:
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.params
        )

        if ids is None:
            return

        now = rospy.Time.now().to_sec()

        for tag_id in ids.flatten():
            tag_id = int(tag_id)

            if now - self.last_seen.get(tag_id, 0.0) < self.cooldown:
                continue

            self.last_seen[tag_id] = now

            entry = self.db.get(tag_id)

            if entry:
                out = {
                    "tag_id": tag_id,
                    "known": True,
                    "tag_type": entry["tag_type"],
                    "traffic_sign_type": entry["traffic_sign_type"],
                    "street_name": entry["street_name"],
                    "vehicle_name": entry["vehicle_name"],
                }
            else:
                out = {
                    "tag_id": tag_id,
                    "known": False,
                    "tag_type": "unknown",
                    "traffic_sign_type": "unknown",
                    "street_name": "",
                    "vehicle_name": "",
                }

            msg_out = String()
            msg_out.data = json.dumps(out)
            self.pub.publish(msg_out)

            rospy.loginfo("Detected sign: %s", msg_out.data)


if __name__ == "__main__":
    node = DetectIntersectionSignNode()
    rospy.spin()