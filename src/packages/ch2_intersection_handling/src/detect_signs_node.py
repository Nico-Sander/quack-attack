#!/usr/bin/env python3

"""
Intersection sign detection via AprilTags.

The signs use the tagStandard52h13 family. If several tags are in frame the one
with the largest area wins: area falls off with the square of the distance, so
the largest is the nearest and therefore the one that describes the
intersection being approached rather than one across it.

Published is the bare tag ID. Translating it into a sign type is
switch_control's job, which is where the sign database is consulted.
"""

import json
import os
import time

import cv2
import numpy as np
import rospy
import yaml
from pupil_apriltags import Detector
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32, String


class DetectSignsNode:
    """Detects intersection signs and publishes the nearest tag ID."""

    def __init__(self, node_name):
        # Stamped before anything else: what this measures is how long the
        # robot has to stand still waiting for this node, so it has to include
        # building the detector rather than start after it.
        self._started_at = time.time()

        rospy.init_node(node_name)

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        node_dir = os.path.dirname(os.path.abspath(__file__))
        base_topic = f"/{self._vehicle_name}"

        self.sign_db_path = rospy.get_param(
            "~sign_db_path", os.path.join(node_dir, "52DB.yaml")
        )
        self.sign_db = self._load_db(self.sign_db_path)

        # Per-tag rate limit on publishing. A sign sits in view for many
        # frames, and switch_control only needs to be told about it often
        # enough to lock in a direction before the stop line.
        self.cooldown = float(rospy.get_param("~cooldown", 0.1))
        self.last_seen = {}

        # Timed and reported, because it is the slowest thing in the launch and
        # the whole run waits on it: building the quick-decode table for
        # tagStandard52h13 (48714 codes, 2-bit error correction) takes about
        # five seconds, which is longer than detect_lane needs to load and
        # trace its network.
        load_started = time.time()
        self.detector = Detector(families="tagStandard52h13")
        self.detector_load_seconds = time.time() - load_started

        self.pub_sign = rospy.Publisher(
            f"{base_topic}/detect/sign", Int32, queue_size=1
        )

        # Every detection in the frame, for the debug dashboard to draw.
        self.pub_detections = rospy.Publisher(
            f"{base_topic}/detect/sign_detections", String, queue_size=1
        )

        # "Signs can be detected now." control_wheels holds the robot still
        # until it sees this. Latched, so a consumer that connects later still
        # gets it rather than waiting forever for a message sent before it
        # existed.
        self.pub_ready = rospy.Publisher(
            f"{base_topic}/detect/sign_ready", Bool, queue_size=1, latch=True
        )
        self.ready = False

        # Only the newest frame is kept. Detection is slower than the camera,
        # and working through a backlog would mean reading signs from images
        # taken metres ago.
        self.latest_image = None

        self.sub_image = rospy.Subscriber(
            f"{base_topic}/camera_node/image/compressed",
            CompressedImage,
            self._cb_image,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo("detect_signs ready: 52h13, %d tags in the database, "
                      "detector built in %.1fs",
                      len(self.sign_db), self.detector_load_seconds)

    def _load_db(self, path):
        """Reads the tag ID to sign type mapping used for logging."""
        try:
            with open(path, "r") as db_file:
                data = yaml.safe_load(db_file)

            return {int(entry["tag_id"]):
                    entry.get("traffic_sign_type") or ""
                    for entry in data}
        except (IOError, OSError, ValueError, KeyError, TypeError) as error:
            # Not fatal: the ID is what gets published, and switch_control
            # loads the database itself. This copy only labels log lines.
            rospy.logwarn(f"Could not load the sign database: {error}")
            return {}

    def _cb_image(self, msg):
        """Keeps the newest frame for the detection loop to pick up."""
        self.latest_image = msg

    def _describe(self, detection):
        """One detection as a plain dict, in camera pixel coordinates."""
        corners = detection.corners.astype(np.float32)

        return {
            "tag_id": int(detection.tag_id),
            "area": float(cv2.contourArea(corners)),
            "corners": [[float(x), float(y)] for x, y in corners],
            "center": [float(detection.center[0]),
                       float(detection.center[1])],
        }

    def _publish_detections(self, detections, stamp):
        """
        Publishes every detection in the frame, empty frames included.

        Sending the empty ones too is what lets the dashboard tell "nothing in
        view" from "node stopped" and clear its boxes, instead of leaving the
        last ones on screen.
        """
        self.pub_detections.publish(String(data=json.dumps({
            "stamp": stamp,
            "detections": detections,
        })))

    def _announce_ready(self):
        """
        Says "signs can be detected now", once and latched.

        Deliberately not sent when the detector finishes building. Two things
        have to be true before a sign in front of the camera would actually be
        read, and the second implies the first: the detector exists, and a
        frame has been through it, so the camera feed really is arriving.
        """
        if self.ready:
            return

        self.ready = True
        self.pub_ready.publish(Bool(data=True))
        rospy.loginfo("Sign detection is live after %.1fs",
                      time.time() - self._started_at)

    def _publish_nearest(self, detections, now):
        """
        Publishes the nearest tag, subject to its cooldown.

        Unthresholded on purpose: switch_control has to keep seeing the sign
        at whatever size it appears, and the largest tag in frame is already
        the closest one.
        """
        nearest = max(detections, key=lambda d: d["area"])
        tag_id = nearest["tag_id"]

        if now - self.last_seen.get(tag_id, 0.0) < self.cooldown:
            return

        self.last_seen[tag_id] = now
        self.pub_sign.publish(Int32(data=tag_id))

        # Debug only. A sign in view produces one of these per cooldown period
        # for as long as it is visible; switch_control reports the turn it
        # actually decided on, which is the interesting event.
        rospy.logdebug("Sign %d (%s, area %.0f px)", tag_id,
                       self.sign_db.get(tag_id, "unknown"), nearest["area"])

    def run(self):
        """
        Detection loop.

        10 Hz rather than the camera's 30: a sign is in view for a second or
        more as the robot approaches, so this is far more often than needed to
        catch one, and tag detection is not cheap.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            msg = self.latest_image
            self.latest_image = None

            if msg is not None:
                np_arr = np.frombuffer(msg.data, np.uint8)
                gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

                if gray is not None:
                    now = rospy.Time.now().to_sec()
                    detections = [self._describe(detection)
                                  for detection in self.detector.detect(gray)]

                    self._publish_detections(detections, now)
                    self._announce_ready()

                    if detections:
                        self._publish_nearest(detections, now)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = DetectSignsNode("detect_signs_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
