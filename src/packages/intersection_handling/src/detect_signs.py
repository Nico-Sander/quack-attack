#!/usr/bin/env python3

import os
import yaml
import rospy
import cv2
import numpy as np

from std_msgs.msg import Int32
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
                                            "/{}/detect/sign".format(vehicle))

        self.cooldown = float(rospy.get_param("~cooldown", 0.1))
        self.last_seen = {}

        self.db = self.load_db(self.db_path)

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()

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

        rospy.loginfo("detect_intersection_sign_node started at 10 Hz (Closest Only)")
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

                    # Ensure we actually found at least one tag
                    if ids is not None and len(ids) > 0:
                        closest_tag_id = None
                        max_area = 0.0

                        # Loop through all detected tags to find the largest one
                        for i in range(len(ids)):
                            # corners[i][0] contains the 4 (x,y) coordinates of the tag
                            area = cv2.contourArea(corners[i][0])
                            
                            if area > max_area:
                                max_area = area
                                closest_tag_id = int(ids[i][0])

                        # Process only the closest tag
                        if closest_tag_id is not None:
                            now = rospy.Time.now().to_sec()

                            # Only publish if the closest tag is past its cooldown
                            if now - self.last_seen.get(closest_tag_id, 0.0) >= self.cooldown:
                                self.last_seen[closest_tag_id] = now
                                self.pub.publish(Int32(data=closest_tag_id))
                                
                                # Optional: Added area to the debug log so you can tune things if needed
                                rospy.loginfo("Detected closest sign: %d (Area: %.1f px)", closest_tag_id, max_area)
            
            rate.sleep()


if __name__ == "__main__":
    node = DetectIntersectionSignNode()
    node.run()