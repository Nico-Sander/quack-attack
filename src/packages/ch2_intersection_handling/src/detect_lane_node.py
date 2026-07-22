#!/usr/bin/env python3

"""
Semantic lane detection with a U-Net, and the geometry it implies.

The network segments the camera image into background, white line, yellow line
and red line. The lane centre follows from where the white and the yellow line
sit in a narrow horizontal search band; the error is how far that centre is
from the middle of the image.

Two further quantities are derived from the red pixels: how far down the image
the stop line lies, which serves as a measure of distance, and at what angle it
runs, from a straight-line fit. Both go out as JSON on lane_borders, together
with which lane markings are currently visible -- that is what tells the state
machine the intersection has been left behind. The search band is also pushed
below the stop line once one is seen, so the line is not read as a lane
marking.
"""

import json
import os

import cv2
import numpy as np
import rospy
import segmentation_models_pytorch as smp
import torch
import torchvision.transforms.functional as TF
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, Int32, String

from custom_enums import DriveMode

# Class indices of the segmentation model. 0 is background.
CLASS_WHITE = 1
CLASS_YELLOW = 2
CLASS_RED = 3

# Half height of the horizontal band the line positions are read from, in
# pixels of the resized image.
SEARCH_BAND_HALF_HEIGHT = 15

# A line counts as seen only if the band holds more than this many of its
# pixels. Fewer than that is noise, and taking a median of it would put the
# lane centre somewhere arbitrary.
MIN_LINE_PIXELS = 10

# Below this many red pixels in the whole frame there is no stop line, only
# speckle from the segmentation.
MIN_RED_PIXELS = 50


class DetectLaneNode:
    """Segments the lane and publishes the error to its centre."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        self.config = self._load_config()
        self.lane_search_y_ratio = self.config["lane_search_y_ratio"]
        self._target_im_size = self.config["target_im_size"]
        self.current_dynamic_y = (self._target_im_size
                                  * self.lane_search_y_ratio)

        # While stopped at a line or crossing it, look at the very bottom of
        # the image. Anything further up is on the far side of the line and
        # belongs to a piece of road the robot is not on yet.
        self.crossing_search_y_ratio = self.config.get(
            "crossing_search_y_ratio", 0.99
        )

        # Process one frame in N. The camera runs at 30 Hz and one pass costs
        # ~20 ms, so frame_skip=1 gives a 30 Hz control loop with headroom to
        # spare. This is the effective control rate: the PID runs once per
        # frame published here, so lowering this is the only way to steer more
        # often. The controller's derivative term is low-pass filtered
        # (control_lane_node) so it stays stable across rates.
        self.frame_skip = max(1, int(self.config.get("frame_skip", 1)))
        self.report_timing_every = float(
            self.config.get("report_timing_every", 10.0)
        )
        self._last_timing_report = 0.0

        # The segmentation model over-reports red on some surfaces, which shows
        # up as phantom stop lines. Requiring the pixels to also be red in HSV
        # removes those without needing to retrain. Red sits at both ends of
        # the hue circle, hence two windows.
        # uint8 because that is what cv2.inRange expects; a plain list would
        # arrive as int64 and raise.
        self.hsv_red_bounds = [
            (np.array(self.config.get("hsv_red_lower1", [0, 100, 80]),
                      dtype=np.uint8),
             np.array(self.config.get("hsv_red_upper1", [10, 255, 255]),
                      dtype=np.uint8)),
            (np.array(self.config.get("hsv_red_lower2", [165, 100, 80]),
                      dtype=np.uint8),
             np.array(self.config.get("hsv_red_upper2", [180, 255, 255]),
                      dtype=np.uint8)),
        ]

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        # Frame gating
        self.is_running = False
        self.counter = 0

        # Latest results, kept for the debug publishers
        self.img = None
        self.debug_img_white = None
        self.debug_img_yellow = None
        self.debug_img_red = None
        self.lane_center = 0.0
        self.center_white = 0.0
        self.center_yellow = 0.0

        # Where a line is assumed to be when it is not visible: the white line
        # far right, the yellow line far left. Driving on their midpoint keeps
        # the robot roughly in lane even with one line missing.
        self.white_fallback = int(self._target_im_size * 0.95)
        self.yellow_fallback = int(self._target_im_size * 0.05)

        # Loading and tracing the model takes a few seconds and is reported in
        # one line at the end, not step by step.
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        load_started = rospy.Time.now().to_sec()
        self._init_model()
        self._model_load_seconds = rospy.Time.now().to_sec() - load_started

        base_topic = f"/{self._vehicle_name}"

        # buff_size matters here: frames are ~40 KB and rospy's default receive
        # buffer is 64 KB, so under any load the socket backlogs and this node
        # starts steering on stale images.
        self.sub_image = rospy.Subscriber(
            f"{base_topic}/camera_node/image/compressed",
            CompressedImage,
            self.cb_find_lane,
            queue_size=1,
            buff_size=2 ** 24,
        )

        # The drive mode decides how far ahead it is safe to look.
        self.sub_mode = rospy.Subscriber(
            f"{base_topic}/switch/mode", Int32, self._cb_mode, queue_size=1
        )

        self.pub_lane = rospy.Publisher(
            f"{base_topic}/detect/lane", Float64, queue_size=1
        )

        # The raw geometry behind the error. detect_intersection thresholds the
        # red line distance from it, and switch_control watches the two
        # detected flags to tell when the intersection has been left.
        self.pub_lane_borders = rospy.Publisher(
            f"{base_topic}/detect/lane_borders", String, queue_size=1
        )

        self.pub_debug_lane = rospy.Publisher(
            f"{base_topic}/debug/lane_croped", CompressedImage, queue_size=1
        )
        self.pub_debug_white = rospy.Publisher(
            f"{base_topic}/debug/lane_white", CompressedImage, queue_size=1
        )
        self.pub_debug_yellow = rospy.Publisher(
            f"{base_topic}/debug/lane_yellow", CompressedImage, queue_size=1
        )
        self.pub_debug_red = rospy.Publisher(
            f"{base_topic}/debug/lane_red", CompressedImage, queue_size=1
        )

        rospy.loginfo("detect_lane ready: %s, model loaded in %.1fs, "
                      "1-in-%d frames (%d Hz control)",
                      self.device, self._model_load_seconds,
                      self.frame_skip, round(30.0 / self.frame_skip))

    def _load_config(self):
        """Reads the detect_lane section of the package config."""
        config_path = os.path.join(
            os.path.dirname(__file__), "../config/config.json"
        )

        try:
            with open(config_path, "r") as config_file:
                return json.load(config_file)["detect_lane"]
        except (IOError, OSError, KeyError, ValueError) as error:
            rospy.logwarn(f"Using default detect_lane config due to: {error}")
            return {
                "lane_search_y_ratio": 0.6,
                "target_im_size": 192,
            }

    def _init_model(self):
        """Loads the U-Net weights and traces the model for inference."""
        self.model = smp.Unet(
            encoder_name="mobilenet_v2",
            encoder_weights=None,  # Weights are loaded from file
            in_channels=3,
            classes=4,
        )

        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(
            current_dir, "../models/lane_segmentation_002_model.pth"
        )

        if not os.path.exists(model_path):
            rospy.logerr(f"Model weights not found at {model_path}")
            return

        self.model.load_state_dict(
            torch.load(model_path, map_location=self.device)
        )
        self.model.to(self.device)
        self.model.eval()

        # CPU optimisations. The bot has no GPU, so this is the normal path.
        self.model = self.model.to(memory_format=torch.channels_last)
        torch.set_num_threads(4)

        # Trace once with a dummy frame: JIT removes the Python overhead of the
        # forward pass, which is a large share of the per-frame cost on CPU.
        with torch.no_grad():
            dummy_input = torch.randn(
                1, 3, self._target_im_size, self._target_im_size
            ).to(self.device)
            dummy_input = dummy_input.to(memory_format=torch.channels_last)
            self.model = torch.jit.trace(self.model, dummy_input)

    def _cb_mode(self, msg):
        """
        Picks the search band for the current drive mode.

        Standing at the line or crossing the intersection, the only lane
        markings that mean anything are directly in front of the wheels;
        anything further up belongs to a road on the other side of the
        intersection.
        """
        try:
            mode = DriveMode(msg.data)
        except ValueError:
            return

        if mode in (DriveMode.STOPPED, DriveMode.CROSSING_INTERSECTION):
            self.lane_search_y_ratio = self.crossing_search_y_ratio
        else:
            self.lane_search_y_ratio = self.config["lane_search_y_ratio"]

    def get_x_from_mask(self, mask, class_idx, fallback_value,
                        search_y_center):
        """
        Median column of one class inside the horizontal search band.

        Returns fallback_value when the class is not convincingly present, so
        the caller can tell a real detection from an assumption.
        """
        y_start = max(0, int(search_y_center - SEARCH_BAND_HALF_HEIGHT))
        y_end = min(self._target_im_size,
                    int(search_y_center + SEARCH_BAND_HALF_HEIGHT))

        target_pixels = np.where(mask[y_start:y_end, :] == class_idx)[1]

        if len(target_pixels) > MIN_LINE_PIXELS:
            return np.median(target_pixels)

        return fallback_value

    def cb_find_lane(self, image_msg):
        """
        Frame gate: throttles, guards against re-entry, clears the busy flag.

        The try/finally is load-bearing. is_running is what stops a second
        frame being processed while one is in flight, so if a frame ever
        raised, the flag would stay set and lane detection would stop for the
        rest of the run -- silently, with the robot still driving on the last
        error it computed.
        """
        if self.counter < self.frame_skip - 1:
            self.counter += 1
            return

        if self.is_running:
            return

        self.is_running = True
        self.counter = 0

        start_time = rospy.Time.now().to_sec()

        try:
            self._process_frame(image_msg)
        except Exception as error:
            rospy.logerr_throttle(
                5.0, f"Lane detection failed on a frame: {error}"
            )
        finally:
            self._report_timing(start_time, rospy.Time.now().to_sec())
            self.is_running = False

    def _process_frame(self, image_msg):
        """Segments one frame and publishes the lane error and geometry."""
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            return

        # The model was trained on the bottom third of the camera feed, so
        # that is the only part worth showing it.
        height, width, _ = cv_image.shape
        crop_height = height // 3
        cropped_img = cv_image[height - crop_height:height, 0:width]

        rgb_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)
        resized_img = cv2.resize(
            rgb_img, (self._target_im_size, self._target_im_size)
        )

        # to_tensor also converts HWC to CHW and scales 0-255 to 0.0-1.0.
        tensor_img = TF.to_tensor(resized_img)

        # The exact ImageNet statistics the model was trained with.
        tensor_img = TF.normalize(
            tensor_img,
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        )
        tensor_img = tensor_img.unsqueeze(0).to(self.device)
        tensor_img = tensor_img.to(memory_format=torch.channels_last)

        with torch.no_grad():
            output = self.model(tensor_img)
            pred_mask = torch.argmax(output.squeeze(), dim=0).cpu().numpy()

        pred_mask = self._reject_phantom_red(pred_mask, resized_img)

        red_detected, closest_red_y, red_angle = self._measure_red_line(
            pred_mask
        )
        search_y_center = self._pick_search_band(red_detected, closest_red_y)

        center_white, center_yellow = self._find_lane_borders(
            pred_mask, search_y_center
        )

        # A fallback value means the line was not actually seen.
        white_detected = bool(center_white != self.white_fallback)
        yellow_detected = bool(center_yellow != self.yellow_fallback)

        lane_center = (center_white + center_yellow) / 2.0

        # Map the centre to an error in [-1, 1]: negative means the lane is to
        # the right of the robot, so it has to steer right.
        error_msg = Float64()
        error_msg.data = 1.0 - (lane_center / self._target_im_size * 2.0)
        self.pub_lane.publish(error_msg)

        borders_msg = String()
        borders_msg.data = json.dumps({
            "yellow_x": float(center_yellow / self._target_im_size),
            "white_x": float(center_white / self._target_im_size),
            "lane_center_x": float(lane_center / self._target_im_size),
            "valid_lanes": bool(center_white > center_yellow),
            "white_detected": white_detected,
            "yellow_detected": yellow_detected,
            "red_detected": red_detected,
            # Normalised to [0, 1]; 1.0 is the bottom edge of the image, so a
            # larger value means the stop line is closer to the wheels.
            "red_distance_y": float(closest_red_y / self._target_im_size),
            "red_angle": red_angle,
        })
        self.pub_lane_borders.publish(borders_msg)

        self.img = cv2.resize(
            cropped_img, (self._target_im_size, self._target_im_size)
        )
        self.lane_center = lane_center
        self.center_white = center_white
        self.center_yellow = center_yellow
        self.current_dynamic_y = search_y_center

        self.debug_img_white = (
            (pred_mask == CLASS_WHITE).astype(np.uint8) * 255
        )
        self.debug_img_yellow = (
            (pred_mask == CLASS_YELLOW).astype(np.uint8) * 255
        )
        self.debug_img_red = (pred_mask == CLASS_RED).astype(np.uint8) * 255

    def _reject_phantom_red(self, pred_mask, resized_img):
        """
        Drops pixels the model calls red that are not red in HSV.

        These phantom stop lines otherwise trigger a full stop on open road.
        """
        hsv_img = cv2.cvtColor(resized_img, cv2.COLOR_RGB2HSV)
        hsv_red_mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)

        for lower, upper in self.hsv_red_bounds:
            hsv_red_mask = cv2.bitwise_or(
                hsv_red_mask, cv2.inRange(hsv_img, lower, upper)
            )

        pred_mask[(pred_mask == CLASS_RED) & (hsv_red_mask == 0)] = 0

        return pred_mask

    def _measure_red_line(self, pred_mask):
        """
        Locates the stop line: whether it is there, how close, and at what
        angle.

        Returns (red_detected, closest_red_y, red_angle). closest_red_y is
        -1.0 when nothing was found, so a caller that ignores the flag gets a
        distance that cannot be mistaken for a near line.
        """
        red_mask = (pred_mask == CLASS_RED).astype(np.uint8)
        y_coords_red, x_coords_red = np.where(red_mask > 0)

        if len(y_coords_red) <= MIN_RED_PIXELS:
            return False, -1.0, 0.0

        # The largest row index is the part of the line nearest the wheels.
        closest_red_y = float(np.max(y_coords_red))

        # Fit a line through the red pixels; its slope is how skewed the robot
        # is relative to the stop line.
        slope, _ = np.polyfit(x_coords_red, y_coords_red, 1)
        red_angle = float(np.arctan(slope))

        return True, closest_red_y, red_angle

    def _pick_search_band(self, red_detected, closest_red_y):
        """
        Moves the search band below the stop line when one is in the way.

        Reading the lane across a red line picks up the markings on its far
        side, which belong to the next stretch of road and pull the robot off
        centre just as it is trying to stop squarely.
        """
        default_y_center = self._target_im_size * self.lane_search_y_ratio

        if red_detected and closest_red_y > default_y_center:
            return closest_red_y + (
                (self._target_im_size - closest_red_y) / 2.0
            )

        return default_y_center

    def _find_lane_borders(self, pred_mask, search_y_center):
        """
        Column of the white and the yellow line, with a plausibility check.

        The yellow line is always left of the white one. If they come out the
        other way round, one of them is a misdetection: whichever is further
        from where its line should be is replaced by its fallback.
        """
        center_white = self.get_x_from_mask(
            pred_mask, CLASS_WHITE, self.white_fallback, search_y_center
        )
        center_yellow = self.get_x_from_mask(
            pred_mask, CLASS_YELLOW, self.yellow_fallback, search_y_center
        )

        if center_white <= center_yellow:
            if center_white > int(self._target_im_size * 0.4):
                center_yellow = self.yellow_fallback
            else:
                center_white = self.white_fallback

        return center_white, center_yellow

    def _report_timing(self, start_time, end_time):
        """
        Periodically logs how long a frame takes.

        Without this the pipeline can slow down -- a busy CPU, a bigger
        model -- and the only symptom is worse driving, because every decision
        is made on an older image than it looks.
        """
        if self.report_timing_every <= 0.0:
            return

        if end_time - self._last_timing_report < self.report_timing_every:
            return

        self._last_timing_report = end_time
        elapsed_ms = (end_time - start_time) * 1000.0
        budget_ms = 1000.0 * self.frame_skip / 30.0

        message = (f"lane pipeline {elapsed_ms:.0f} ms/frame "
                   f"(budget {budget_ms:.0f} ms at 1-in-{self.frame_skip})")

        if elapsed_ms > budget_ms:
            rospy.logwarn(message + " -- falling behind, images are stale")
        else:
            rospy.loginfo(message)

    def _publish_mask(self, publisher, mask):
        """Publishes a debug image, but only if something is listening."""
        if publisher.get_num_connections() == 0 or mask is None:
            return

        debug_msg = CompressedImage()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.format = "jpeg"
        debug_msg.data = np.array(cv2.imencode(".jpg", mask)[1]).tobytes()
        publisher.publish(debug_msg)

    def _draw_overlay(self):
        """Annotates the last frame with what the node made of it."""
        debug_img = self.img.copy()
        y_search = int(self.current_dynamic_y)
        size = self._target_im_size

        # Lane centre, and the fallback columns the missing lines snap to.
        cv2.circle(debug_img, (int(self.lane_center), int(size / 2)), 3,
                   (255, 0, 0), -1)
        cv2.line(debug_img, (self.white_fallback, 0),
                 (self.white_fallback, size), (255, 255, 255))
        cv2.line(debug_img, (self.yellow_fallback, 0),
                 (self.yellow_fallback, size), (255, 255, 0))

        # Search band and image centre, the line the error is measured against.
        cv2.line(debug_img, (0, y_search + 20), (size, y_search + 20),
                 (255, 255, 255))
        cv2.line(debug_img, (0, y_search - 20), (size, y_search - 20),
                 (255, 255, 255))
        cv2.line(debug_img, (int(size / 2), 0), (int(size / 2), size),
                 (0, 255, 0))

        cv2.circle(debug_img, (int(self.center_white), y_search), 5,
                   (255, 255, 255), -1)
        cv2.circle(debug_img, (int(self.center_yellow), y_search), 5,
                   (0, 255, 255), -1)

        return debug_img

    def run_debug(self):
        """
        Publishes the debug images, well below the detection rate.

        Encoding four JPEGs per frame would eat into the budget the lane
        pipeline needs, and nobody watches a dashboard at 30 Hz.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if (self.pub_debug_lane.get_num_connections() > 0
                    and self.img is not None):
                self._publish_mask(self.pub_debug_lane, self._draw_overlay())

            self._publish_mask(self.pub_debug_white, self.debug_img_white)
            self._publish_mask(self.pub_debug_yellow, self.debug_img_yellow)
            self._publish_mask(self.pub_debug_red, self.debug_img_red)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = DetectLaneNode("detect_lane_node")
        node.run_debug()
    except rospy.ROSInterruptException:
        pass
