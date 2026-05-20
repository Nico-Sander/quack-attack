#!/usr/bin/env python3

import json
import math
import os

import rospy
from std_msgs.msg import Float64, String
from duckietown_msgs.msg import Twist2DStamped


class ControlLaneNode:
    """Lane controller with gated obstacle avoidance.

    Outside of a relevant duckie situation this node uses the same PID structure as
    the Challenge 1 lane controller. Obstacle avoidance is only activated when a
    sufficiently large duckie blocks the normal lane target inside the planning
    band.
    """

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.load_config()

        self.lastError = 0.0
        self.v = 0.0
        self.a = 0.0
        self.integral = 0.0
        self.last_time = None

        self.yellow_valid = False
        self.white_valid = False

        self.current_lane_error = 0.0
        self.last_lane_msg_time = 0.0

        self.lane_left_x = 0.05
        self.lane_right_x = 0.95
        self.lane_center_x = 0.5
        self.last_lane_borders_time = 0.0

        self.last_good_lane_left = 0.05
        self.last_good_lane_right = 0.95
        self.last_good_lane_time = 0.0

        self.active_duckies = []
        self.last_duckie_seen_time = 0.0
        self.last_obstacle_msg_time = 0.0
        self.duckie_missed_frames = 0
        self.raw_duckies_count = 0
        self.filtered_small_duckies_count = 0

        self.latest_debug = {
            "avoidance_active": False,
            "reason": "waiting_for_data",
        }

        self.pid_mode = "lane"
        self.last_avoidance_side = None
        self.last_avoidance_target_x = None
        self.last_avoidance_time = 0.0

        self.no_valid_escape_since = None
        self.blocked_recovery_active = False
        self.blocked_recovery_start_time = 0.0
        self.blocked_recovery_phase = None
        self.blocked_recovery_omega_cmd = 0.0
        self.blocked_recovery_best_target_x = None
        self.blocked_recovery_best_debug = None
        self.blocked_recovery_best_score = None
        self.blocked_recovery_best_elapsed = 0.0
        self.blocked_recovery_return_start_time = 0.0
        self.blocked_recovery_return_duration = 0.0

        base_topic = f"/{self.vehicle_name}"

        self.pub_cmd_vel = rospy.Publisher(
            f"{base_topic}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1,
        )

        self.pub_debug_plan = rospy.Publisher(
            f"{base_topic}/debug/free_path_plan",
            String,
            queue_size=1,
        )

        self.sub_lane = rospy.Subscriber(
            f"{base_topic}/detect/lane",
            Float64,
            self.cbFollowLane,
            queue_size=1,
        )

        self.sub_lane_borders = rospy.Subscriber(
            f"{base_topic}/detect/lane_borders",
            String,
            self.cbLaneBorders,
            queue_size=1,
        )

        self.sub_obstacles = rospy.Subscriber(
            f"{base_topic}/detect/duckie_BB",
            String,
            self.cbObstacles,
            queue_size=1,
        )

        rospy.on_shutdown(self.fnShutDown)

    def _param(self, group, key, default):
        return group.get(key, {}).get("default", default)

    def load_config(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "../config/control_lane_node.json")

        params = {}
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    config = json.load(f)
                params = config.get("parameters", config)
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Could not load config: {e}. Using defaults.")
        else:
            rospy.logwarn(f"[{self.node_name}] Config not found: {config_path}. Using defaults.")

        pid = params.get("pid", {})
        self.kp = float(self._param(pid, "p", 6.0))
        self.ki = float(self._param(pid, "i", 0.0))
        self.kd = float(self._param(pid, "d", 1.0))
        self.MAX_VEL = float(self._param(pid, "max_vel", 0.25))

        planner = params.get("path_planner", {})

        raw_y_min = float(self._param(planner, "y_min", 0.50))
        raw_y_max = float(self._param(planner, "y_max", 0.92))
        self.plan_y_min = min(raw_y_min, raw_y_max)
        self.plan_y_max = max(raw_y_min, raw_y_max)

        self.duckie_x_margin = float(self._param(planner, "duckie_x_margin", 0.08))
        self.duckie_y_margin = float(self._param(planner, "duckie_y_margin", 0.04))
        self.lane_margin = float(self._param(planner, "lane_margin", 0.02))

        self.planner_image_width_px = int(self._param(planner, "planner_image_width_px", 192))
        self.min_free_width_px = int(self._param(planner, "min_free_width_px", 30))

        self.lane_timeout = float(self._param(planner, "lane_timeout", 1.0))
        self.obstacle_timeout = float(self._param(planner, "obstacle_timeout", 1.2))
        self.duckie_hold_time = float(self._param(planner, "duckie_hold_time", 0.8))
        self.duckie_missed_frames_before_clear = int(
            self._param(planner, "duckie_missed_frames_before_clear", 4)
        )

        self.max_omega = float(self._param(planner, "max_omega", 5.0))
        self.min_vel = float(self._param(planner, "min_vel", 0.04))

        self.avoidance_vel = float(self._param(planner, "avoidance_vel", 0.08))
        self.avoidance_steering_gain = float(self._param(planner, "avoidance_steering_gain", 1.20))

        self.avoidance_kp = float(self._param(planner, "avoidance_kp", 4.0))
        self.avoidance_ki = float(self._param(planner, "avoidance_ki", 0.0))
        self.avoidance_kd = float(self._param(planner, "avoidance_kd", 0.0))

        self.avoidance_side_lock_time = float(self._param(planner, "avoidance_side_lock_time", 1.0))
        self.avoidance_side_lock_bonus = float(self._param(planner, "avoidance_side_lock_bonus", 0.20))
        self.avoidance_target_smoothing_alpha = float(
            self._param(planner, "avoidance_target_smoothing_alpha", 0.45)
        )

        self.avoidance_clear_hold_time = float(self._param(planner, "avoidance_clear_hold_time", 1.0))
        self.lane_reentry_blend_time = float(self._param(planner, "lane_reentry_blend_time", 0.8))
        self.reentry_vel = float(self._param(planner, "reentry_vel", 0.09))

        self.lane_target_block_margin = float(self._param(planner, "lane_target_block_margin", 0.10))
        self.escape_clearance = float(self._param(planner, "escape_clearance", 0.04))
        self.gap_width_bonus_weight = float(self._param(planner, "gap_width_bonus_weight", 0.05))
        self.narrow_gap_behavior = str(self._param(planner, "narrow_gap_behavior", "stop"))

        self.obstacle_image_width_px = int(self._param(planner, "obstacle_image_width_px", 640))
        self.obstacle_image_height_px = int(self._param(planner, "obstacle_image_height_px", 480))
        self.min_duckie_width_px = int(self._param(planner, "min_duckie_width_px", 24))
        self.min_duckie_height_px = int(self._param(planner, "min_duckie_height_px", 24))
        self.min_duckie_area_px = int(self._param(planner, "min_duckie_area_px", 700))

        self.default_lane_left = float(self._param(planner, "default_lane_left", 0.05))
        self.default_lane_right = float(self._param(planner, "default_lane_right", 0.95))

        self.open_side_width_bonus = float(self._param(planner, "open_side_width_bonus", 0.35))

        self.blocked_recovery_delay = float(self._param(planner, "blocked_recovery_delay", 2.0))
        self.blocked_recovery_omega = float(self._param(planner, "blocked_recovery_omega", 0.20))
        self.blocked_recovery_min_turn_time = float(
            self._param(planner, "blocked_recovery_min_turn_time", 0.45)
        )
        self.blocked_recovery_min_angle_deg = float(
            self._param(planner, "blocked_recovery_min_angle_deg", 35.0)
        )
        self.blocked_recovery_max_angle_deg = float(
            self._param(planner, "blocked_recovery_max_angle_deg", 80.0)
        )

        rospy.loginfo(
            f"[{self.node_name}] Params: kp={self.kp}, kd={self.kd}, max_vel={self.MAX_VEL}, "
            f"avoidance_vel={self.avoidance_vel}, y_band=({self.plan_y_min}, {self.plan_y_max}), "
            f"duckie_margin={self.duckie_x_margin}, target_block_margin={self.lane_target_block_margin}, "
            f"avoidance_pid=({self.avoidance_kp},{self.avoidance_ki},{self.avoidance_kd}), "
            f"min_duckie=({self.min_duckie_width_px}x{self.min_duckie_height_px}px, area={self.min_duckie_area_px}), "
            f"clear_hold={self.avoidance_clear_hold_time}, reentry_blend={self.lane_reentry_blend_time}, "
            f"open_side_bonus={self.open_side_width_bonus}, "
            f"blocked_recovery=({self.blocked_recovery_delay}s,{self.blocked_recovery_omega}rad/s,"
            f"min_time={self.blocked_recovery_min_turn_time}s,"
            f"min_angle={self.blocked_recovery_min_angle_deg}deg,"
            f"max_angle={self.blocked_recovery_max_angle_deg}deg)"
        )

    @staticmethod
    def clamp01(value):
        return max(0.0, min(1.0, float(value)))

    def lane_borders_recent(self):
        return (rospy.Time.now().to_sec() - self.last_lane_borders_time) < self.lane_timeout

    def good_lane_recent(self):
        return (rospy.Time.now().to_sec() - self.last_good_lane_time) < self.lane_timeout

    def obstacle_data_recent(self):
        if not self.active_duckies:
            return False
        now = rospy.Time.now().to_sec()
        return (now - self.last_duckie_seen_time) < self.duckie_hold_time

    def extract_duckie_box(self, duckie):
        xmin = float(duckie.get("xmin", duckie["x_center"] - duckie["width"] / 2.0))
        xmax = float(duckie.get("xmax", duckie["x_center"] + duckie["width"] / 2.0))
        ymin = float(duckie.get("ymin", duckie["y_center"] - duckie["height"] / 2.0))
        ymax = float(duckie.get("ymax", duckie["y_center"] + duckie["height"] / 2.0))
        return self.clamp01(xmin), self.clamp01(xmax), self.clamp01(ymin), self.clamp01(ymax)

    def duckie_large_enough(self, duckie):
        try:
            xmin, xmax, ymin, ymax = self.extract_duckie_box(duckie)
        except Exception:
            return False

        width_px = (xmax - xmin) * float(self.obstacle_image_width_px)
        height_px = (ymax - ymin) * float(self.obstacle_image_height_px)
        area_px = width_px * height_px

        return (
            width_px >= self.min_duckie_width_px and
            height_px >= self.min_duckie_height_px and
            area_px >= self.min_duckie_area_px
        )

    def cbObstacles(self, msg):
        now = rospy.Time.now().to_sec()
        self.last_obstacle_msg_time = now

        try:
            data = json.loads(msg.data)
            raw_duckies = data.get("duckies", [])

            if not raw_duckies and data.get("detected", False):
                if data.get("class_name", "").lower() == "duckie":
                    raw_duckies = [data]

            self.raw_duckies_count = len(raw_duckies)

            filtered = []
            small_count = 0

            for duckie in raw_duckies:
                if duckie.get("class_name", "").lower() != "duckie":
                    continue

                if self.duckie_large_enough(duckie):
                    filtered.append(duckie)
                else:
                    small_count += 1

            self.filtered_small_duckies_count = small_count

            if filtered:
                self.active_duckies = filtered
                self.last_duckie_seen_time = now
                self.duckie_missed_frames = 0
                return

            self.duckie_missed_frames += 1

            clear_by_frames = self.duckie_missed_frames >= self.duckie_missed_frames_before_clear
            clear_by_time = (now - self.last_duckie_seen_time) > self.duckie_hold_time

            if clear_by_frames and clear_by_time:
                self.active_duckies = []

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] Could not parse duckie_BB: {e}")

    def cbLaneBorders(self, msg):
        try:
            data = json.loads(msg.data)

            if not bool(data.get("valid", True)):
                return

            yellow_x = self.clamp01(data.get("yellow_x", self.lane_left_x))
            white_x = self.clamp01(data.get("white_x", self.lane_right_x))

            yellow_valid = bool(data.get("yellow_valid", True))
            white_valid = bool(data.get("white_valid", True))

            if yellow_valid and white_valid and yellow_x >= white_x:
                return

            if yellow_valid:
                self.lane_left_x = yellow_x

            if white_valid:
                self.lane_right_x = white_x

            self.yellow_valid = yellow_valid
            self.white_valid = white_valid

            if self.lane_left_x < self.lane_right_x:
                self.lane_center_x = self.clamp01(
                    data.get("lane_center_x", (self.lane_left_x + self.lane_right_x) / 2.0)
                )

            self.last_lane_borders_time = rospy.Time.now().to_sec()

            if yellow_valid and white_valid:
                width = self.lane_right_x - self.lane_left_x
                if 0.25 <= width <= 0.95:
                    self.last_good_lane_left = self.lane_left_x
                    self.last_good_lane_right = self.lane_right_x
                    self.last_good_lane_time = self.last_lane_borders_time

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] Could not parse lane_borders: {e}")

    def get_lane_limits(self):
        if self.lane_borders_recent():
            if self.yellow_valid:
                lane_left = self.lane_left_x + self.lane_margin
                left_open = False
            else:
                lane_left = 0.0
                left_open = True

            if self.white_valid:
                lane_right = self.lane_right_x - self.lane_margin
                right_open = False
            else:
                lane_right = 1.0
                right_open = True

            source = "current_lane_borders"

        elif self.good_lane_recent():
            lane_left = self.last_good_lane_left + self.lane_margin
            lane_right = self.last_good_lane_right - self.lane_margin
            left_open = False
            right_open = False
            source = "last_good_lane_borders"

        else:
            lane_left = self.default_lane_left
            lane_right = self.default_lane_right
            left_open = False
            right_open = False
            source = "default_lane_borders"

        lane_left = self.clamp01(lane_left)
        lane_right = self.clamp01(lane_right)

        if lane_right <= lane_left:
            lane_left = self.default_lane_left
            lane_right = self.default_lane_right
            left_open = False
            right_open = False
            source = "default_invalid_lane"

        return lane_left, lane_right, source, left_open, right_open

    def get_blocked_intervals(self, lane_left, lane_right):
        blocked = []
        relevant_count = 0

        if not self.obstacle_data_recent():
            return [], 0

        for duckie in self.active_duckies:
            try:
                xmin, xmax, ymin, ymax = self.extract_duckie_box(duckie)
            except Exception:
                continue

            if ymax < (self.plan_y_min - self.duckie_y_margin):
                continue

            if ymin > (self.plan_y_max + self.duckie_y_margin):
                continue

            left = max(lane_left, xmin - self.duckie_x_margin)
            right = min(lane_right, xmax + self.duckie_x_margin)

            if right > left:
                blocked.append((left, right))
                relevant_count += 1

        if not blocked:
            return [], relevant_count

        blocked.sort(key=lambda interval: interval[0])

        merged = [blocked[0]]
        for left, right in blocked[1:]:
            last_left, last_right = merged[-1]

            if left <= last_right:
                merged[-1] = (last_left, max(last_right, right))
            else:
                merged.append((left, right))

        return merged, relevant_count

    def get_free_intervals(self, lane_left, lane_right, blocked):
        free = []
        cursor = lane_left

        for left, right in blocked:
            if left > cursor:
                free.append((cursor, left))
            cursor = max(cursor, right)

        if cursor < lane_right:
            free.append((cursor, lane_right))

        return free

    def interval_width_px(self, interval):
        return int(round((interval[1] - interval[0]) * float(self.planner_image_width_px)))

    def find_blocking_interval(self, lane_target_x, blocked):
        for left, right in blocked:
            if (left - self.lane_target_block_margin) <= lane_target_x <= (right + self.lane_target_block_margin):
                return (left, right)
        return None

    def free_interval_containing(self, x, free_intervals):
        for interval in free_intervals:
            if interval[0] <= x <= interval[1]:
                return interval
        return None

    def interval_effective_width(self, interval, left_open=False, right_open=False):
        effective_width = interval[1] - interval[0]

        if left_open and interval[0] <= 0.001:
            effective_width += self.open_side_width_bonus

        if right_open and interval[1] >= 0.999:
            effective_width += self.open_side_width_bonus

        return effective_width

    def choose_escape_target(
        self,
        lane_target_x,
        blocking_interval,
        free_intervals,
        left_open=False,
        right_open=False,
    ):
        block_left, block_right = blocking_interval

        candidates = []
        now = rospy.Time.now().to_sec()

        side_lock_active = (
            self.last_avoidance_side is not None and
            (now - self.last_avoidance_time) < self.avoidance_side_lock_time
        )

        raw_points = [
            (block_left - self.escape_clearance, "left_escape"),
            (block_right + self.escape_clearance, "right_escape"),
        ]

        for point, side in raw_points:
            point = self.clamp01(point)
            interval = self.free_interval_containing(point, free_intervals)

            if interval is None:
                continue

            width_px = self.interval_width_px(interval)
            if width_px < self.min_free_width_px:
                continue

            safe_left = interval[0] + self.escape_clearance
            safe_right = interval[1] - self.escape_clearance

            if safe_right <= safe_left:
                target = (interval[0] + interval[1]) / 2.0
            else:
                target = (safe_left + safe_right) / 2.0

            target = max(interval[0], min(interval[1], target))

            distance = abs(target - lane_target_x)
            effective_width = self.interval_effective_width(
                interval,
                left_open=left_open,
                right_open=right_open,
            )
            width_bonus = effective_width * self.gap_width_bonus_weight

            score = distance - width_bonus

            if side == self.last_avoidance_side and side_lock_active:
                score -= self.avoidance_side_lock_bonus

            candidates.append((score, distance, target, side, interval, width_px))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            _, _, point, side, interval, width_px = candidates[0]
            return point, side, interval, width_px

        fallback = []
        for interval in free_intervals:
            width_px = self.interval_width_px(interval)

            if width_px < self.min_free_width_px:
                continue

            safe_left = interval[0] + self.escape_clearance
            safe_right = interval[1] - self.escape_clearance

            if safe_right <= safe_left:
                target = (interval[0] + interval[1]) / 2.0
            else:
                target = (safe_left + safe_right) / 2.0

            target = max(interval[0], min(interval[1], target))

            distance = abs(target - lane_target_x)
            effective_width = self.interval_effective_width(
                interval,
                left_open=left_open,
                right_open=right_open,
            )
            width_bonus = effective_width * self.gap_width_bonus_weight
            score = distance - width_bonus

            fallback.append((score, target, "nearest_free_interval", interval, width_px))

        if not fallback:
            return None, "no_valid_escape", None, 0

        fallback.sort(key=lambda item: item[0])
        _, point, side, interval, width_px = fallback[0]
        return point, side, interval, width_px

    def choose_avoidance_target(self, lane_error):
        lane_target_x = self.clamp01((1.0 - lane_error) / 2.0)

        lane_left, lane_right, lane_source, left_open, right_open = self.get_lane_limits()
        blocked, relevant_count = self.get_blocked_intervals(lane_left, lane_right)
        free_intervals = self.get_free_intervals(lane_left, lane_right, blocked)

        debug = {
            "avoidance_active": False,
            "valid": True,
            "reason": "lane_follow_no_relevant_duckie",
            "target_x": lane_target_x,
            "lane_target_x": lane_target_x,
            "lane_left": lane_left,
            "lane_right": lane_right,
            "lane_source": lane_source,
            "left_open": left_open,
            "right_open": right_open,
            "plan_y_min": self.plan_y_min,
            "plan_y_max": self.plan_y_max,
            "blocked_intervals": blocked,
            "free_intervals": free_intervals if blocked else [],
            "selected_free_interval": None,
            "selected_free_width_px": 0,
            "min_free_width_px": self.min_free_width_px,
            "num_raw_duckies": self.raw_duckies_count,
            "num_active_duckies": len(self.active_duckies),
            "num_small_duckies_filtered": self.filtered_small_duckies_count,
            "num_relevant_duckies": relevant_count,
        }

        if relevant_count <= 0 or not blocked:
            return None, debug

        blocking_interval = self.find_blocking_interval(lane_target_x, blocked)

        if blocking_interval is None:
            debug["reason"] = "lane_follow_target_not_blocked"
            return None, debug

        if not free_intervals:
            debug["avoidance_active"] = True
            debug["valid"] = False
            debug["reason"] = "no_free_interval"
            return "STOP", debug

        target_x, side, selected_interval, selected_width_px = self.choose_escape_target(
            lane_target_x,
            blocking_interval,
            free_intervals,
            left_open=left_open,
            right_open=right_open,
        )

        debug["avoidance_active"] = True
        debug["blocking_interval"] = blocking_interval
        debug["avoidance_side"] = side
        debug["selected_free_interval"] = selected_interval
        debug["selected_free_width_px"] = selected_width_px
        debug["widest_free_interval"] = selected_interval
        debug["widest_free_width_px"] = selected_width_px

        if target_x is None:
            debug["valid"] = False
            debug["reason"] = "no_valid_escape_target"

            if self.narrow_gap_behavior == "stop":
                return "STOP", debug

            return None, debug

        target_x = max(lane_left, min(lane_right, target_x))

        now = rospy.Time.now().to_sec()
        if (
            self.last_avoidance_target_x is not None and
            (now - self.last_avoidance_time) < self.avoidance_side_lock_time
        ):
            alpha = self.avoidance_target_smoothing_alpha
            target_x = alpha * target_x + (1.0 - alpha) * self.last_avoidance_target_x
            target_x = max(lane_left, min(lane_right, target_x))

        debug["reason"] = f"duckie_avoid_{side}"
        debug["target_x"] = target_x
        debug["target_smoothed"] = True

        return target_x, debug

    def calculate_pid(self, error, velocity_override=None, mode="lane"):
        current_time = rospy.Time.now().to_sec()

        mode_changed = mode != self.pid_mode
        if mode_changed:
            self.pid_mode = mode
            self.integral = 0.0

        if mode == "avoidance":
            kp = self.avoidance_kp
            ki = self.avoidance_ki
            kd = self.avoidance_kd
        else:
            kp = self.kp
            ki = self.ki
            kd = self.kd

        if self.last_time is None:
            self.last_time = current_time
            self.lastError = error
            self.v = self.MAX_VEL if velocity_override is None else velocity_override
            self.a = max(min(kp * error, self.max_omega), -self.max_omega)
            return

        dt = current_time - self.last_time

        if dt > 0.0:
            p_term = kp * error

            self.integral += error * dt
            max_integral = 1.0
            self.integral = max(min(self.integral, max_integral), -max_integral)

            i_term = ki * self.integral

            if mode_changed:
                d_term = 0.0
            else:
                d_term = kd * ((error - self.lastError) / dt)

            omega = p_term + i_term + d_term
            omega = max(min(omega, self.max_omega), -self.max_omega)

            velocity = self.MAX_VEL if velocity_override is None else velocity_override
            velocity = max(velocity, self.min_vel)

            self.v = velocity
            self.a = omega

        self.lastError = error
        self.last_time = current_time

    def get_post_avoidance_target(self, lane_error, debug):
        if self.last_avoidance_target_x is None:
            return None, None, debug

        now = rospy.Time.now().to_sec()
        elapsed = now - self.last_avoidance_time
        lane_target_x = self.clamp01((1.0 - lane_error) / 2.0)

        if elapsed < self.avoidance_clear_hold_time:
            debug["avoidance_active"] = True
            debug["reason"] = "post_avoidance_clear_hold"
            debug["target_x"] = self.last_avoidance_target_x
            debug["post_avoidance_elapsed"] = elapsed
            debug["post_avoidance_phase"] = "hold"
            return self.last_avoidance_target_x, self.avoidance_vel, debug

        blend_end = self.avoidance_clear_hold_time + self.lane_reentry_blend_time

        if elapsed < blend_end and self.lane_reentry_blend_time > 0.0:
            progress = (elapsed - self.avoidance_clear_hold_time) / self.lane_reentry_blend_time
            progress = max(0.0, min(1.0, progress))

            target_x = (1.0 - progress) * self.last_avoidance_target_x + progress * lane_target_x
            target_x = self.clamp01(target_x)

            debug["avoidance_active"] = True
            debug["reason"] = "post_avoidance_lane_reentry"
            debug["target_x"] = target_x
            debug["lane_target_x"] = lane_target_x
            debug["post_avoidance_elapsed"] = elapsed
            debug["post_avoidance_phase"] = "blend"
            debug["reentry_progress"] = progress

            return target_x, self.reentry_vel, debug

        self.last_avoidance_target_x = None
        self.last_avoidance_side = None

        return None, None, debug

    def reset_blocked_recovery(self):
        self.no_valid_escape_since = None
        self.blocked_recovery_active = False
        self.blocked_recovery_start_time = 0.0
        self.blocked_recovery_phase = None
        self.blocked_recovery_omega_cmd = 0.0
        self.blocked_recovery_best_target_x = None
        self.blocked_recovery_best_debug = None
        self.blocked_recovery_best_score = None
        self.blocked_recovery_best_elapsed = 0.0
        self.blocked_recovery_return_start_time = 0.0
        self.blocked_recovery_return_duration = 0.0

    def choose_blocked_recovery_omega(self, debug):
        omega = abs(self.blocked_recovery_omega)
        if omega <= 0.0:
            omega = 0.20

        # Wenn nur eine Linie sichtbar ist, ist die Lage am Rand eindeutig.
        # Außen zwischen weißer Linie und Duckie: nach links in die freie Fläche drehen.
        if self.white_valid and not self.yellow_valid:
            debug["blocked_recovery_direction_reason"] = "white_only_turn_left"
            return omega

        # Innen zwischen gelber Linie und Duckie: nach rechts in die freie Fläche drehen.
        if self.yellow_valid and not self.white_valid:
            debug["blocked_recovery_direction_reason"] = "yellow_only_turn_right"
            return -omega

        # Sonst möglichst von der zuletzt gewählten Avoidance-Seite wegdrehen.
        if self.last_avoidance_side == "left_escape":
            debug["blocked_recovery_direction_reason"] = "last_left_escape_turn_left"
            return omega

        if self.last_avoidance_side == "right_escape":
            debug["blocked_recovery_direction_reason"] = "last_right_escape_turn_right"
            return -omega

        debug["blocked_recovery_direction_reason"] = "default_turn_left"
        return omega

    def blocked_recovery_scan_duration(self):
        omega_abs = abs(self.blocked_recovery_omega_cmd)
        if omega_abs <= 0.0:
            omega_abs = max(abs(self.blocked_recovery_omega), 0.20)

        min_angle_time = math.radians(max(0.0, self.blocked_recovery_min_angle_deg)) / omega_abs
        max_angle_time = math.radians(max(0.0, self.blocked_recovery_max_angle_deg)) / omega_abs

        scan_time = max(0.0, self.blocked_recovery_min_turn_time, min_angle_time)

        if max_angle_time > 0.0:
            scan_time = min(scan_time, max_angle_time)

        return scan_time

    def start_blocked_recovery_scan(self, debug):
        self.blocked_recovery_active = True
        self.blocked_recovery_phase = "scan"
        self.blocked_recovery_start_time = rospy.Time.now().to_sec()
        self.blocked_recovery_omega_cmd = self.choose_blocked_recovery_omega(debug)
        self.blocked_recovery_best_target_x = None
        self.blocked_recovery_best_debug = None
        self.blocked_recovery_best_score = None
        self.blocked_recovery_best_elapsed = 0.0
        self.blocked_recovery_return_start_time = 0.0
        self.blocked_recovery_return_duration = 0.0

    def score_blocked_recovery_candidate(self, target_x, debug):
        selected_width_px = float(debug.get("selected_free_width_px", 0.0))
        lane_target_x = float(debug.get("lane_target_x", 0.5))
        distance_px = abs(float(target_x) - lane_target_x) * float(self.planner_image_width_px)
        return selected_width_px - distance_px

    def remember_blocked_recovery_candidate(self, target_x, debug):
        if target_x is None or target_x == "STOP":
            return

        score = self.score_blocked_recovery_candidate(target_x, debug)
        elapsed = rospy.Time.now().to_sec() - self.blocked_recovery_start_time

        if self.blocked_recovery_best_score is None or score > self.blocked_recovery_best_score:
            best_debug = dict(debug)
            best_debug["blocked_recovery_best_score"] = score
            best_debug["blocked_recovery_best_elapsed"] = elapsed

            self.blocked_recovery_best_score = score
            self.blocked_recovery_best_target_x = target_x
            self.blocked_recovery_best_debug = best_debug
            self.blocked_recovery_best_elapsed = elapsed

    def apply_blocked_recovery_turn(self, debug, omega, reason):
        self.v = 0.0
        self.a = omega
        debug["reason"] = reason
        debug["blocked_recovery_active"] = True
        debug["blocked_recovery_phase"] = self.blocked_recovery_phase
        debug["blocked_recovery_omega"] = omega
        debug["blocked_recovery_scan_elapsed"] = (
            rospy.Time.now().to_sec() - self.blocked_recovery_start_time
        )
        debug["blocked_recovery_scan_duration"] = self.blocked_recovery_scan_duration()
        debug["blocked_recovery_best_target_x"] = self.blocked_recovery_best_target_x
        debug["blocked_recovery_best_score"] = self.blocked_recovery_best_score
        return debug

    def handle_blocked_recovery_candidate(self, target_x, debug):
        if not self.blocked_recovery_active:
            return target_x, debug, True

        now = rospy.Time.now().to_sec()

        if self.blocked_recovery_phase == "scan":
            self.remember_blocked_recovery_candidate(target_x, debug)
            scan_elapsed = now - self.blocked_recovery_start_time
            scan_duration = self.blocked_recovery_scan_duration()

            if scan_elapsed < scan_duration:
                debug = self.apply_blocked_recovery_turn(
                    debug,
                    self.blocked_recovery_omega_cmd,
                    "blocked_recovery_scanning_best_gap",
                )
                return None, debug, False

            if self.blocked_recovery_best_target_x is None:
                return target_x, debug, True

            angle_back_time = max(0.0, scan_elapsed - self.blocked_recovery_best_elapsed)
            self.blocked_recovery_phase = "return"
            self.blocked_recovery_return_start_time = now
            self.blocked_recovery_return_duration = angle_back_time

        if self.blocked_recovery_phase == "return":
            return_elapsed = now - self.blocked_recovery_return_start_time

            if return_elapsed < self.blocked_recovery_return_duration:
                best_debug = dict(self.blocked_recovery_best_debug or debug)
                best_debug = self.apply_blocked_recovery_turn(
                    best_debug,
                    -self.blocked_recovery_omega_cmd,
                    "blocked_recovery_returning_to_best_gap",
                )
                best_debug["blocked_recovery_return_elapsed"] = return_elapsed
                best_debug["blocked_recovery_return_duration"] = self.blocked_recovery_return_duration
                return None, best_debug, False

            best_debug = dict(self.blocked_recovery_best_debug or debug)
            best_debug["reason"] = "blocked_recovery_use_best_gap"
            best_debug["blocked_recovery_active"] = False
            best_debug["blocked_recovery_phase"] = "done"
            return self.blocked_recovery_best_target_x, best_debug, True

        return target_x, debug, True

    def handle_no_valid_escape(self, debug):
        now = rospy.Time.now().to_sec()

        if self.no_valid_escape_since is None:
            self.no_valid_escape_since = now

        blocked_elapsed = now - self.no_valid_escape_since
        debug["blocked_elapsed"] = blocked_elapsed

        # Erst kurz warten, damit kurze Fehlentscheidungen nicht sofort zum Drehen führen.
        if blocked_elapsed < self.blocked_recovery_delay:
            self.v = 0.0
            self.a = 0.0
            debug["reason"] = "no_valid_escape_waiting"
            debug["blocked_recovery_active"] = False
            return debug

        if not self.blocked_recovery_active:
            self.start_blocked_recovery_scan(debug)

        # Während der Scan-Phase nicht sofort losfahren, sondern erst nach einer besseren Lücke suchen.
        if self.blocked_recovery_phase == "scan":
            return self.apply_blocked_recovery_turn(
                debug,
                self.blocked_recovery_omega_cmd,
                "blocked_recovery_scanning_no_gap_yet",
            )

        # Falls während des Zurückdrehens kurz kein Ziel sichtbar ist, weiter zur gespeicherten Lücke zurückdrehen.
        if self.blocked_recovery_phase == "return":
            return self.apply_blocked_recovery_turn(
                debug,
                -self.blocked_recovery_omega_cmd,
                "blocked_recovery_returning_to_best_gap",
            )

        return debug

    def cbFollowLane(self, msg):
        lane_error = float(msg.data)
        self.current_lane_error = lane_error
        self.last_lane_msg_time = rospy.Time.now().to_sec()

        target_x, debug = self.choose_avoidance_target(lane_error)

        if target_x == "STOP":
            if debug.get("reason") in ["no_valid_escape_target", "no_free_interval"]:
                debug = self.handle_no_valid_escape(debug)
            else:
                self.v = 0.0
                self.a = 0.0

            debug["error"] = 0.0
            debug["v"] = self.v
            debug["omega"] = self.a

            self.pub_debug_plan.publish(String(data=json.dumps(debug)))
            self.latest_debug = debug
            return

        if self.blocked_recovery_active and target_x is not None:
            target_x, debug, recovery_done = self.handle_blocked_recovery_candidate(target_x, debug)

            if not recovery_done:
                debug["error"] = 0.0
                debug["v"] = self.v
                debug["omega"] = self.a

                self.pub_debug_plan.publish(String(data=json.dumps(debug)))
                self.latest_debug = debug
                return

            self.reset_blocked_recovery()
        elif target_x is None:
            self.reset_blocked_recovery()

        if target_x is None:
            post_target_x, post_velocity, debug = self.get_post_avoidance_target(lane_error, debug)

            if post_target_x is None:
                error = lane_error
                velocity_override = None
                mode = "lane"
            else:
                raw_avoid_error = 1.0 - 2.0 * post_target_x
                error = max(-1.0, min(1.0, raw_avoid_error * self.avoidance_steering_gain))
                velocity_override = post_velocity
                mode = "avoidance"
        else:
            raw_avoid_error = 1.0 - 2.0 * target_x
            error = max(-1.0, min(1.0, raw_avoid_error * self.avoidance_steering_gain))
            velocity_override = self.avoidance_vel
            mode = "avoidance"

            self.last_avoidance_side = debug.get("avoidance_side")
            self.last_avoidance_target_x = target_x
            self.last_avoidance_time = rospy.Time.now().to_sec()

        self.calculate_pid(error, velocity_override=velocity_override, mode=mode)

        debug["error"] = error
        debug["v"] = self.v
        debug["omega"] = self.a
        self.pub_debug_plan.publish(String(data=json.dumps(debug)))
        self.latest_debug = debug    

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            twist.header.stamp = rospy.Time.now()
            twist.v = self.v
            twist.omega = self.a
            self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ControlLaneNode("control_lane_node")
        node.run()
    except rospy.ROSInterruptException:
        pass