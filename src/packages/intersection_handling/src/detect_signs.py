#!/usr/bin/env python3

import os
import yaml
import rospy
import cv2
import numpy as np
from pupil_apriltags import Detector 
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage


class DetectIntersectionSignNode:
    def __init__(self):
        rospy.init_node("detect_intersection_sign_node")

        vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        node_dir = os.path.dirname(os.path.abspath(__file__))

        self.db_path = rospy.get_param("~apriltags_db_path",
                                       os.path.join(node_dir, "apriltagsDB.yaml"))
        
        self.db_52h13_path = rospy.get_param("~apriltags_db_52h13_path",
                                             os.path.join(node_dir, "52DB.yaml"))

        self.image_topic = rospy.get_param("~image_topic",
                                           "/{}/camera_node/image/compressed".format(vehicle))

        self.output_topic = rospy.get_param("~output_topic",
                                            "/{}/detect/sign".format(vehicle))

        self.cooldown = float(rospy.get_param("~cooldown", 0.1))
        self.last_seen = {}

        self.db = self.load_db(self.db_path)
        self.db_52h13 = self.load_db(self.db_52h13_path)

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()


        self.detector_52h13 = Detector(families="tagStandard52h13")

        self.pub = rospy.Publisher(self.output_topic, Int32, queue_size=1)
        
        # Buffer to hold the most recently received image
        self.latest_image = None

        self.sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )
        rospy.loginfo("detect_intersection_sign_node started at 10 Hz (36h11 + 52h13)")
        rospy.loginfo("DB 36h11:  %s", self.db_path)
        rospy.loginfo("DB 52h13: %s (%d tags)", self.db_52h13_path, len(self.db_52h13))
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
        self.latest_image = msg

    def run(self):
        """Main control loop running at 10 Hz."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.latest_image is not None:
                msg = self.latest_image
                self.latest_image = None

                img = np.frombuffer(msg.data, np.uint8)
                gray = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

                if gray is not None:
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        gray,
                        self.dictionary,
                        parameters=self.params
                    )

                    closest_tag_id = None
                    max_area = 0.0
                    closest_family = None

                    # 36h11 detection
                    if ids is not None and len(ids) > 0:
                        for i in range(len(ids)):
                            area = cv2.contourArea(corners[i][0])
                            if area > max_area:
                                max_area = area
                                closest_tag_id = int(ids[i][0])
                                closest_family = "36h11"

                    # 52h13 fallback - only if 36h11 found nothing
                    if closest_tag_id is None:
                        for det in self.detector_52h13.detect(gray):
                            area = cv2.contourArea(det.corners.astype(np.float32))
                            if area > max_area:
                                max_area = area
                                closest_tag_id = int(det.tag_id)
                                closest_family = "52h13"

                    # Publish closest tag (whichever family it came from)
                    if closest_tag_id is not None:
                        now = rospy.Time.now().to_sec()
                        if now - self.last_seen.get(closest_tag_id, 0.0) >= self.cooldown:
                            self.last_seen[closest_tag_id] = now
                            self.pub.publish(Int32(data=closest_tag_id))

                            if closest_family == "36h11":
                                entry = self.db.get(closest_tag_id)
                            else:
                                entry = self.db_52h13.get(closest_tag_id)

                            sign_type = entry.get("traffic_sign_type", "unknown") if entry else "unknown"
                            rospy.loginfo(
                                "Detected closest sign: %d (family=%s, type=%s, Area: %.1f px)",
                                closest_tag_id, closest_family, sign_type, max_area
                            )

            rate.sleep()


if __name__ == "__main__":
    node = DetectIntersectionSignNode()
    node.run()