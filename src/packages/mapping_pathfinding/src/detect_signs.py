#!/usr/bin/env python3

import json
import os
import yaml
import rospy
import cv2
import numpy as np
from pupil_apriltags import Detector
from std_msgs.msg import Int32, String
from sensor_msgs.msg import CompressedImage

import gate_detection

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../config/config.json"
)


class DetectIntersectionSignNode:
    def __init__(self):
        rospy.init_node("detect_intersection_sign_node")

        vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        node_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.sign_db_path = rospy.get_param("~sign_db_path",
                                             os.path.join(node_dir, "52DB.yaml"))

        self.image_topic = rospy.get_param("~image_topic",
                                           "/{}/camera_node/image/compressed".format(vehicle))

        self.output_topic = rospy.get_param("~output_topic",
                                            "/{}/detect/sign".format(vehicle))

        self.detections_topic = rospy.get_param(
            "~detections_topic", "/{}/detect/sign_detections".format(vehicle)
        )

        # Minimum detected area for a tag to count as being on the street the
        # bot is on rather than somewhere across an intersection. Only the
        # `accepted` flag is derived from it here -- what to do with a rejected
        # detection is the consumer's business, and the dashboard draws both.
        self.gate_min_area = self._load_gate_min_area()

        self.cooldown = float(rospy.get_param("~cooldown", 0.1))
        self.last_seen = {}

        self.sign_db = self.load_db(self.sign_db_path)
        self.detector_52h13 = Detector(families="tagStandard52h13")

        self.pub = rospy.Publisher(self.output_topic, Int32, queue_size=1)

        # Every detection in the frame, with geometry and the accept decision.
        # The mapping node filters gates with it and the dashboard draws it, so
        # the size threshold is applied in exactly one place.
        self.pub_detections = rospy.Publisher(
            self.detections_topic, String, queue_size=1
        )

        # Buffer to hold the most recently received image
        self.latest_image = None

        self.sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )
        rospy.loginfo("detect_signs started (52h13, %d tags, min gate area "
                      "%.0f px): %s -> %s",
                      len(self.sign_db), self.gate_min_area,
                      self.image_topic, self.output_topic)

    def _load_gate_min_area(self):
        """
        The gate size threshold, from config.json unless overridden.

        config.json is where the calibrated value lives, alongside every other
        tuned constant. The ROS param only wins when it is set to something
        positive, so the launch file can leave it at its "not given" sentinel
        and the config keeps control -- otherwise a launch-arg default would
        silently shadow the config for everyone who never passes the arg.
        """
        override = float(rospy.get_param("~gate_min_area", -1.0))

        if override > 0.0:
            rospy.loginfo("gate_min_area overridden to %.0f px", override)
            return override

        try:
            with open(CONFIG_PATH, "r") as handle:
                return float(json.load(handle)["detect_signs"]["gate_min_area"])
        except (IOError, ValueError, KeyError, TypeError) as error:
            rospy.logwarn("Could not read detect_signs.gate_min_area from %s "
                          "(%s); falling back to %.0f px",
                          CONFIG_PATH, error, gate_detection.DEFAULT_MIN_AREA)
            return gate_detection.DEFAULT_MIN_AREA

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

    def _describe(self, det):
        """One detection as a plain dict, in camera pixel coordinates."""
        corners = det.corners.astype(np.float32)
        area = float(cv2.contourArea(corners))

        return {
            "tag_id": int(det.tag_id),
            "area": area,
            "accepted": gate_detection.is_confident(area, self.gate_min_area),
            "corners": [[float(x), float(y)] for x, y in corners],
            "center": [float(det.center[0]), float(det.center[1])],
        }

    def _publish_detections(self, detections, stamp):
        """
        Publishes every detection in the frame, accepted or not.

        Published on every processed frame including empty ones, so a consumer
        can tell "nothing in view" from "node stopped" -- the dashboard relies
        on that to clear its boxes instead of leaving the last ones on screen.
        """
        self.pub_detections.publish(String(data=json.dumps({
            "stamp": stamp,
            "min_area": self.gate_min_area,
            "detections": detections,
        })))

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
                    now = rospy.Time.now().to_sec()
                    detections = [self._describe(det)
                                  for det in self.detector_52h13.detect(gray)]

                    self._publish_detections(detections, now)

                    # /detect/sign carries the closest tag only, unthresholded:
                    # switch_control reads intersection signs from it and must
                    # keep seeing them at whatever size they appear.
                    if detections:
                        closest = max(detections,
                                      key=lambda d: d["area"])
                        closest_tag_id = closest["tag_id"]

                        if now - self.last_seen.get(closest_tag_id, 0.0) >= self.cooldown:
                            self.last_seen[closest_tag_id] = now
                            self.pub.publish(Int32(data=closest_tag_id))

                            entry = self.sign_db.get(closest_tag_id)
                            sign_type = entry.get("traffic_sign_type", "unknown") if entry else "unknown"

                            # Debug only. A tag in view produces one of these
                            # every second for as long as it is visible, and it
                            # duplicates what matters: the mapping node already
                            # reports each gate it records, with the area. Turn
                            # it on when a tag is not being picked up at all --
                            # or better, read it off a rosbag, which has every
                            # detection rather than one per second.
                            rospy.logdebug(
                                "Detected 52h13 sign: %d (type=%s, area=%.0f px, %s)"
                                % (closest_tag_id, sign_type, closest["area"],
                                   "accepted" if closest["accepted"] else "too far")
                            )

            rate.sleep()


if __name__ == "__main__":
    node = DetectIntersectionSignNode()
    node.run()