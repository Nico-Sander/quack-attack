#!/usr/bin/env python3

import json
import os
import time
import yaml
import rospy
import cv2
import numpy as np
from pupil_apriltags import Detector
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import CompressedImage

import gate_detection

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../config/config.json"
)


class DetectIntersectionSignNode:
    def __init__(self):
        # Before anything else: what this stamps is how long the bot has to
        # stand still waiting for this node, so it has to include the detector
        # build rather than start after it.
        self._started_at = time.time()

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

        # "This node is now producing detections." control_wheels holds the
        # robot still until it sees this; see the ready flag below.
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/{}/detect/sign_ready".format(vehicle)
        )

        # Minimum detected area for a tag to count as being on the street the
        # bot is on rather than somewhere across an intersection. Only the
        # `accepted` flag is derived from it here -- what to do with a rejected
        # detection is the consumer's business, and the dashboard draws both.
        self.gate_min_area = self._load_gate_min_area()

        self.cooldown = float(rospy.get_param("~cooldown", 0.1))
        self.last_seen = {}

        self.sign_db = self.load_db(self.sign_db_path)

        # Timed and reported, because it is the slowest thing in the launch and
        # the whole run waits on it: building the quick-decode table for
        # tagStandard52h13 (48714 codes, 2-bit error correction) takes about
        # five seconds, which is *longer* than detect_lane needs to load and
        # trace its network. Nothing about that is obvious from the outside, and
        # it used to mean the bot drove the first stretch of its first street
        # with no gate detection running at all.
        load_started = time.time()
        self.detector_52h13 = Detector(families="tagStandard52h13")
        self.detector_load_seconds = time.time() - load_started

        self.pub = rospy.Publisher(self.output_topic, Int32, queue_size=1)

        # Every detection in the frame, with geometry and the accept decision.
        # The mapping node filters gates with it and the dashboard draws it, so
        # the size threshold is applied in exactly one place.
        self.pub_detections = rospy.Publisher(
            self.detections_topic, String, queue_size=1
        )

        # Latched, so a consumer that connects later still gets it rather than
        # waiting forever for a message that was sent once, before it existed.
        self.pub_ready = rospy.Publisher(
            self.ready_topic, Bool, queue_size=1, latch=True
        )
        self.ready = False

        # Buffer to hold the most recently received image
        self.latest_image = None

        self.sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.cb_image,
            queue_size=1,
            buff_size=2**24
        )
        rospy.loginfo("detect_signs started (52h13, %d tags, detector built in "
                      "%.1fs, min gate area %.0f px): %s -> %s",
                      len(self.sign_db), self.detector_load_seconds,
                      self.gate_min_area, self.image_topic, self.output_topic)

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

    def _announce_ready(self):
        """
        Says "gates can be detected now", once and latched.

        Deliberately not sent when the detector finishes building. Three things
        have to be true before a gate in front of the camera would actually be
        recorded, and only the last of them implies the other two:

          1. the detector exists,
          2. a frame has been through it, so the camera feed really is arriving,
          3. someone is connected to the detections topic -- the mapping node
             subscribes at its own pace, and a message published before that
             TCP connection is up is dropped, not queued.

        Called once per processed frame; returns immediately after the first
        time it fires.
        """
        if self.ready:
            return

        if self.pub_detections.get_num_connections() < 1:
            rospy.logwarn_throttle(
                2.0, "Detections are ready but nothing is subscribed to %s yet; "
                     "holding the ready flag back", self.detections_topic
            )
            return

        self.ready = True
        self.pub_ready.publish(Bool(data=True))
        rospy.loginfo("detect_signs ready after %.1fs -- gate detection is live",
                      time.time() - self._started_at)

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
                    self._announce_ready()

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