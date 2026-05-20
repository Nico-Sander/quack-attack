#!/usr/bin/env python3

"""OpenCV dashboard for lane following and duckie avoidance debug data."""

import json
import os
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_duckie")

        self.top_size = (360, 240)
        self.pad_size = 10
        self.bottom_height = 520
        self.fps = 10
        self.total_width = self.top_size[0] * 3 + self.pad_size * 2

        self.img_lane = self.placeholder(self.top_size[1], self.top_size[0], "Waiting: lane")
        self.img_white = self.placeholder(self.top_size[1], self.top_size[0], "Waiting: white")
        self.img_yellow = self.placeholder(self.top_size[1], self.top_size[0], "Waiting: yellow")
        self.img_obstacle = self.placeholder(480, 640, "Waiting: obstacle")
        self.obstacle_roi = (0, 0, self.total_width, self.bottom_height)

        self.latest_plan = {}
        self.last_plan_time = 0.0
        self.plan_timeout = 1.0

        self.pad_vertical = np.full((self.top_size[1], self.pad_size, 3), 50, dtype=np.uint8)
        self.pad_horizontal = np.full((self.pad_size, self.total_width, 3), 50, dtype=np.uint8)

        base_debug = f"/{self.vehicle_name}/debug"
        rospy.Subscriber(f"{base_debug}/lane_croped", CompressedImage, self.cb_lane, queue_size=1)
        rospy.Subscriber(f"{base_debug}/lane_white", CompressedImage, self.cb_white, queue_size=1)
        rospy.Subscriber(f"{base_debug}/lane_yellow", CompressedImage, self.cb_yellow, queue_size=1)
        rospy.Subscriber(f"{base_debug}/obstacle_detection", CompressedImage, self.cb_obstacle, queue_size=1)
        rospy.Subscriber(f"/{self.vehicle_name}/debug/free_path_plan", String, self.cb_plan, queue_size=1)

        rospy.loginfo(f"[{self.node_name}] Dashboard started for {self.vehicle_name}")

    def placeholder(self, height, width, text):
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, text, (20, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (180, 180, 180), 2)
        return img

    def decode_image(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return self.placeholder(self.top_size[1], self.top_size[0], "Decode failed")
        return img

    def cb_lane(self, msg):
        self.img_lane = self.decode_image(msg)

    def cb_white(self, msg):
        self.img_white = self.decode_image(msg)

    def cb_yellow(self, msg):
        self.img_yellow = self.decode_image(msg)

    def cb_obstacle(self, msg):
        self.img_obstacle = self.decode_image(msg)

    def cb_plan(self, msg):
        try:
            self.latest_plan = json.loads(msg.data)
            self.last_plan_time = time.time()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] Could not parse free_path_plan: {e}")

    def safe_top(self, img):
        if img is None or img.size == 0:
            return self.placeholder(self.top_size[1], self.top_size[0], "No image")
        out = img.copy()
        if len(out.shape) == 2:
            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        if out.shape[:2] != (self.top_size[1], self.top_size[0]):
            out = cv2.resize(out, self.top_size)
        return out

    def scale_bottom(self, img):
        if img is None or img.size == 0:
            self.obstacle_roi = (0, 0, self.total_width, self.bottom_height)
            return self.placeholder(self.bottom_height, self.total_width, "No obstacle")

        out = img.copy()
        if len(out.shape) == 2:
            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

        h, w = out.shape[:2]
        scale = self.total_width / float(w)
        new_w = self.total_width
        new_h = int(h * scale)
        if new_h > self.bottom_height:
            new_h = self.bottom_height
            new_w = int(w * (new_h / float(h)))

        resized = cv2.resize(out, (new_w, new_h))
        canvas = np.zeros((self.bottom_height, self.total_width, 3), dtype=np.uint8)
        x_off = (self.total_width - new_w) // 2
        y_off = 0
        canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized
        self.obstacle_roi = (x_off, y_off, new_w, new_h)
        return canvas

    @staticmethod
    def clamp01(value):
        return max(0.0, min(1.0, float(value)))

    def as_intervals(self, value):
        intervals = []
        if not isinstance(value, list):
            return intervals
        for item in value:
            try:
                left = self.clamp01(item[0])
                right = self.clamp01(item[1])
            except Exception:
                continue
            if right > left:
                intervals.append((left, right))
        return intervals

    def draw_rect_alpha(self, img, x1, y1, x2, y2, color, alpha):
        if x2 <= x1 or y2 <= y1:
            return
        overlay = img.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0.0, img)

    def map_point(self, x_norm, y_norm):
        x_off, y_off, w, h = self.obstacle_roi
        return int(x_off + self.clamp01(x_norm) * w), int(y_off + self.clamp01(y_norm) * h)

    def apply_overlay(self, img):
        plan = self.latest_plan if isinstance(self.latest_plan, dict) else {}
        h, w = img.shape[:2]
        if not plan:
            cv2.putText(img, "No free_path_plan", (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 180, 255), 2)
            return img

        stale = (time.time() - self.last_plan_time) > self.plan_timeout
        y_min = self.clamp01(plan.get("plan_y_min", 0.5))
        y_max = self.clamp01(plan.get("plan_y_max", 0.92))
        if y_max <= y_min:
            y_min, y_max = 0.5, 0.92

        x_left = self.clamp01(plan.get("lane_left", 0.05))
        x_right = self.clamp01(plan.get("lane_right", 0.95))
        blocked = self.as_intervals(plan.get("blocked_intervals", []))
        free = self.as_intervals(plan.get("free_intervals", []))
        selected = self.as_intervals([plan.get("selected_free_interval", [])])
        selected_interval = selected[0] if selected else None

        x1_band, y1 = self.map_point(0.0, y_min)
        x2_band, y2 = self.map_point(1.0, y_max)
        cv2.rectangle(img, (x1_band, y1), (x2_band, y2), (180, 180, 180), 2)

        lane_x1, _ = self.map_point(x_left, y_min)
        lane_x2, _ = self.map_point(x_right, y_min)
        cv2.line(img, (lane_x1, y1), (lane_x1, y2), (255, 255, 255), 2)
        cv2.line(img, (lane_x2, y1), (lane_x2, y2), (255, 255, 255), 2)

        # Only draw green/yellow planning regions when there is a relevant blocked interval.
        if blocked:
            for left, right in free:
                x1, _ = self.map_point(left, y_min)
                x2, _ = self.map_point(right, y_max)
                self.draw_rect_alpha(img, x1, y1, x2, y2, (0, 170, 0), 0.18)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 0), 2)

            if selected_interval is not None:
                x1, _ = self.map_point(selected_interval[0], y_min)
                x2, _ = self.map_point(selected_interval[1], y_max)
                self.draw_rect_alpha(img, x1, y1, x2, y2, (0, 220, 220), 0.28)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 3)

            for left, right in blocked:
                x1, _ = self.map_point(left, y_min)
                x2, _ = self.map_point(right, y_max)
                self.draw_rect_alpha(img, x1, y1, x2, y2, (0, 0, 220), 0.35)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        target_x = plan.get("target_x", None)
        if isinstance(target_x, (int, float)):
            tx, _ = self.map_point(target_x, 0.0)
            cv2.line(img, (tx, 0), (tx, h - 1), (255, 0, 0), 3)
            cv2.putText(img, "target", (tx + 6, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2)

        reason = str(plan.get("reason", "unknown"))
        active = bool(plan.get("avoidance_active", False))
        status = "AVOIDANCE" if active else "LANE FOLLOWING"
        color = (0, 0, 255) if active else (0, 255, 0)
        if stale:
            color = (0, 180, 255)
            status += " / STALE"

        cv2.putText(img, f"{status}: {reason}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        cv2.putText(
            img,
            f"raw:{plan.get('num_raw_duckies','?')} active:{plan.get('num_active_duckies','?')} relevant:{plan.get('num_relevant_duckies','?')} small:{plan.get('num_small_duckies_filtered','?')}",
            (10, h - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (230, 230, 230),
            2,
        )
        cv2.putText(
            img,
            f"v:{plan.get('v','?')} omega:{plan.get('omega','?')} lane:{plan.get('lane_source','?')}",
            (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (230, 230, 230),
            2,
        )
        return img

    def run(self):
        rate = rospy.Rate(self.fps)
        cv2.namedWindow("Duckie Obstacle Avoidance Dashboard", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Duckie Obstacle Avoidance Dashboard", 1220, 820)

        while not rospy.is_shutdown():
            lane = self.safe_top(self.img_lane)
            white = self.safe_top(self.img_white)
            yellow = self.safe_top(self.img_yellow)
            obstacle = self.scale_bottom(self.img_obstacle)
            obstacle = self.apply_overlay(obstacle)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(lane, "Lane", (10, 30), font, 0.75, (0, 255, 0), 2)
            cv2.putText(white, "White mask", (10, 30), font, 0.75, (255, 255, 255), 2)
            cv2.putText(yellow, "Yellow mask", (10, 30), font, 0.75, (0, 255, 255), 2)

            top = cv2.hconcat([lane, self.pad_vertical, white, self.pad_vertical, yellow])
            dashboard = cv2.vconcat([top, self.pad_horizontal, obstacle])
            cv2.imshow("Duckie Obstacle Avoidance Dashboard", dashboard)
            cv2.waitKey(1)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = DashboardNode("debug_dashboard_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
