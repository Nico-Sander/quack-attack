#!/usr/bin/env python3

"""Follow-the-gap driving controller for the duckie U-turn course.

Design (see docs/CONTROL_LANE_NODE.md):

The robot follows the lane as its nominal route and steers around duckies using a
"follow the gap" scheme. A tiny three-state machine (CRUISE / AVOID / ESCAPE_ROTATE)
owns every transition, and a single output guard is the ONLY place that writes the
motor command. That guard enforces one invariant that structurally kills the old
"freeze in front of a duckie" bug:

This is the lean variant: purely reactive, camera and time only. The odometry-backed
obstacle memory / mapping layer and its dead-end ESCAPE_REVERSE recovery have been
removed rather than left switched off, so there is no dormant code path and no
parameter here that does nothing.

    the robot is NEVER commanded (v == 0 and omega == 0)

If there is no forward path, the robot rotates in place to look for a gap instead of
stopping. If it rotates for too long without finding one, it progressively relaxes its
gap/clearance requirements until something opens up. So it can always make progress.

Conventions (normalized image coordinates, all in [0, 1]):
    x: 0 = left image edge, 1 = right image edge
    duckie ymax: larger = closer to the robot (bottom of frame)
    lane error (/detect/lane): positive => lane center is to the LEFT
    omega (output): positive => turn LEFT
"""

import json
import os

import rospy
from std_msgs.msg import Float64, String
from duckietown_msgs.msg import Twist2DStamped

# Live tuning is a convenience, not a requirement. If the package has not been
# rebuilt since the cfg was added (or dynamic_reconfigure is missing), the node
# still runs on the JSON values instead of refusing to start mid-session.
try:
    from dynamic_reconfigure.server import Server
    from ch3_obstacle_avoidance.cfg import ControlLaneConfig
    DYNAMIC_RECONFIGURE = True
except Exception:
    DYNAMIC_RECONFIGURE = False


# --- Hardcoded geometry / debounce constants (not worth exposing as tunables) ---
# The top of the planning band is the tunable `react_ymax`; PLAN_Y_MAX is the bottom.
PLAN_Y_MAX = 1.0           # bottom of the vertical band where duckies matter
LANE_TIMEOUT = 1.0         # s before lane-border data is considered stale
# Floor for the in-place rotation rate. Below roughly this the motors buzz without
# breaking static friction, so a lower tuned value would violate never-freeze in
# practice while looking fine in the command.
MIN_ESCAPE_OMEGA = 0.8
HYSTERESIS_MARGIN = 0.10   # other side must beat current by this to switch avoid side
GAP_INSET = 0.04           # keep the target this far inside the chosen gap's edges
GAP_STICKY_BONUS = 0.06    # width bonus for the gap we are already driving into
CLEAR_HOLD_TIME = 0.6      # s the road must stay clear before AVOID falls back to CRUISE
# Frames of a complete, correctly-ordered view needed to end a wrong-way turn. Kept
# small deliberately: the robot is already rotating when this is evaluated, so every
# extra frame is overshoot - at omega_rotate 2.0 rad/s one frame is roughly 11 deg.
WRONG_WAY_CLEAR_FRAMES = 2
# Safety valve. A 180 deg correction takes ~1.6 s at omega_rotate 2.0, so this is
# several full sweeps. If no clean view has appeared by then the detection is the
# problem and more rotation will not fix it - stop, rather than spin forever.
WRONG_WAY_MAX_ROTATE = 5.0
# Consecutive frames a marking must read as square-on before the robot treats it as
# a wall. Short: unlike wrong_way this is not latched and clears as soon as rotating
# makes the line look vertical again, so the cost of a false positive is one or two
# frames of rotation rather than an aborted manoeuvre.
HEAD_ON_FRAMES = 3
EPS = 1e-3

# Measured on the course: bbox-bottom ymax -> distance from the robot's front, cm.
# Apparent width obeys width_image = width_cm * camera_width_k / distance_cm, so a
# fixed image-space width is correct at exactly ONE distance. That is what made the
# old front_slice_half / duckie_margin_* untunable: at 8 cm the robot's own body is
# 6x wider in the image than at 52 cm, so a constant is either too timid far away or
# a collision up close.
YMAX_TO_CM = ((0.462, 120.0), (0.470, 100.0), (0.503, 70.0), (0.545, 50.0),
              (0.638, 30.0), (0.823, 15.0), (1.000, 8.0))
# Detection dies at ~4 cm; clamp above that so the 1/d conversion cannot explode.
MIN_RANGE_CM = 6.0


def robot_half_image_static(robot_width_cm, camera_width_k, dist_cm):
    """Module-level twin of GapPlanner.robot_half_image, for offline tooling."""
    return (robot_width_cm / 2.0) * camera_width_k / max(dist_cm, MIN_RANGE_CM)


def ymax_to_cm(ymax):
    """Interpolate the measured range curve. Outside it, clamp to the ends."""
    if ymax <= YMAX_TO_CM[0][0]:
        return YMAX_TO_CM[0][1]
    if ymax >= YMAX_TO_CM[-1][0]:
        return YMAX_TO_CM[-1][1]
    for (y0, d0), (y1, d1) in zip(YMAX_TO_CM, YMAX_TO_CM[1:]):
        if y0 <= ymax <= y1:
            f = (ymax - y0) / (y1 - y0) if y1 > y0 else 0.0
            return d0 + f * (d1 - d0)
    return YMAX_TO_CM[-1][1]


# Minimum duckie box size (normalized) to reject YOLO noise / far specks.
MIN_DUCKIE_WIDTH = 0.04
MIN_DUCKIE_HEIGHT = 0.04

# States
CRUISE = "CRUISE"
AVOID = "AVOID"
ESCAPE_ROTATE = "ESCAPE_ROTATE"


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp01(value):
    return clamp(float(value), 0.0, 1.0)


class GapPlanner:
    """Pure decision core: turns cached sensor signals into (v, omega, debug).

    Kept free of rospy so it can be exercised by an off-board test harness. All time
    comes in as an explicit ``now`` argument; all sensor state comes in via the
    ``update_*`` setters. ``step(now)`` returns the command and a debug dict.
    """

    # Every tunable, in one place. set_params() is driven from here, so adding a
    # parameter means touching this tuple and the JSON - nothing else.
    TUNABLES = (
        "v_cruise", "v_avoid", "v_min", "k_steer", "omega_rotate", "omega_max",
        "lane_margin", "camera_width_k", "robot_width_cm", "safety_margin_cm",
        "gap_min_cm", "react_ymax", "front_slow_ymax", "front_block_ymax",
        "escape_min_dwell", "avoid_min_dwell", "escape_relax_after",
        "lane_hold_frames", "duckie_hold_time",
        "yellow_anchor", "white_anchor",
    )

    def __init__(self, params):
        self.set_params(params)

        # Lane-border state.
        self.left_wall = 0.05        # last-known left wall x (order-agnostic)
        self.right_wall = 0.95       # last-known right wall x
        self.left_open = False       # True => no line on the left, corridor opens to 0
        self.right_open = False
        self.left_invalid_streak = 0
        self.right_invalid_streak = 0
        self.last_lane_time = 0.0

        # Colour-identified line state, tracked ALONGSIDE the order-agnostic walls.
        # The walls deliberately drop colour so a skewed pose still yields a corridor;
        # but which colour is visible is the robot's only absolute bearing reference
        # inside the bulb, so it is kept separately rather than thrown away.
        self.yellow_x = 0.05
        self.white_x = 0.95
        self.yellow_seen = False
        self.white_seen = False
        self.yellow_invalid_streak = 0
        self.white_invalid_streak = 0
        self.yellow_valid_now = False
        self.white_valid_now = False

        # Duckie state.
        self.duckies = []            # list of dicts with xmin,xmax,ymax
        self.last_duckie_time = 0.0
        self.raw_duckie_count = 0
        self.small_duckie_count = 0

        self.lane_error = 0.0
        self.corridor_source = "walls"

        # Wrong-way detection. With right-hand traffic the yellow centre line is on the
        # LEFT and the white outer line on the RIGHT whenever the robot is travelling
        # the correct way down a lane. Crossed order therefore means it is pointed
        # backwards - which is how it drove back out of the entry lane, following a
        # perfectly good lane centre in the wrong direction.
        #
        # Debounced, because a single crossed frame also happens transiently when the
        # bot is skewed mid-manoeuvre, and reacting to that would spin it up spuriously.
        # Head-on markings. A line the robot is driving at orthogonally is a wall,
        # not a lane edge - and its reported x is the median of a horizontal smear,
        # i.e. confidently wrong. Debounced like the other line signals.
        self.head_on = False
        self.head_on_streak = 0

        self.lines_crossed = False
        self.wrong_way = False
        self.crossed_streak = 0
        self.uncrossed_streak = 0
        self.wrong_way_since = 0.0
        self.wrong_way_timed_out = False

        # FSM state.
        self.state = CRUISE
        self.state_since = 0.0
        self.escape_dir = 1.0        # +1 => rotate left; latched during a dwell
        self.avoid_side = 0.0        # sign of last avoid steer (for hysteresis)
        self.clear_since = None      # when the road first became clear (for CLEAR_HOLD)
        self.last_target_x = 0.5

    def set_params(self, params):
        """(Re)apply the tunables. Safe to call while driving.

        Deliberately touches ONLY the tunables - never the FSM state, the wall
        estimates or the duckie cache. A live parameter change during a manoeuvre
        must not teleport the planner back to CRUISE or forget the duckie it is
        currently steering around.
        """
        for name in self.TUNABLES:
            if name in params:
                setattr(self, name, float(params[name]))

        # The never-freeze guard commands omega_rotate directly, so the floor has to
        # live here rather than at the call site - a live edit could otherwise set a
        # rotation rate that only buzzes the motors without turning the robot.
        self.omega_rotate = max(self.omega_rotate, MIN_ESCAPE_OMEGA)

    def cm_to_image(self, cm, dist_cm):
        """Physical size in cm -> width in normalized image x, at that range.

        The whole point of the rewrite: perspective means apparent size scales as
        1/distance, so every lateral margin has to be computed at the range of the
        thing it applies to. Calibrated by camera_width_k, which is measured -
        apparent_width x distance was constant at ~1.91 across a 68->8 cm approach,
        and dividing by the duckie's real 5 cm width gives k.
        """
        return cm * self.camera_width_k / max(dist_cm, MIN_RANGE_CM)

    def robot_half_image(self, dist_cm):
        """The robot's own half-width in image x, at a given range."""
        return self.cm_to_image(self.robot_width_cm / 2.0, dist_cm)

    # ---- sensor setters -----------------------------------------------------
    def update_lane_error(self, error):
        self.lane_error = clamp(float(error), -1.0, 1.0)

    def set_head_on(self, head_on):
        """Debounced 'a marking is square across our path'.

        Deliberately NOT latched, unlike wrong_way: rotating away from a wall makes
        it look progressively more vertical, so the condition clears itself as the
        manoeuvre succeeds. The robot ends up parallel to the marking, which is the
        behaviour wanted at the bulb boundary.
        """
        self.head_on_streak = self.head_on_streak + 1 if head_on else 0
        self.head_on = self.head_on_streak > HEAD_ON_FRAMES

    def set_lines_crossed(self, now, crossed):
        """Fold in the detector's crossed-order flag, debounced, and LATCH it.

        Entering only counts while BOTH colours are seen: with one line missing the
        other's position is a fallback constant, so the ordering carries no meaning.
        Reuses lane_hold_frames as the entry debounce.

        Leaving is deliberately NOT the mirror of entering. Rotating to correct a
        wrong-way pose swings a line out of frame almost immediately, so a symmetric
        rule would clear the flag one frame into the turn and abort it half-way -
        which is exactly what was observed on the course. Once latched, only a
        complete and correctly-ordered view ends it, which is the same condition as
        "the robot can see where it is again".

        The timeout is not optional: without it, a robot that rotated into the bulb -
        where both lines are never simultaneously visible - would spin forever, which
        is a worse failure than the one this fixes.
        """
        crossed = bool(crossed) and self.yellow_seen and self.white_seen
        self.lines_crossed = crossed

        if self.wrong_way:
            # Only a view with BOTH lines currently visible and correctly ordered
            # ends the turn. Using the debounced flags here would clear the latch one
            # frame after rotation swung a line out of shot, aborting the correction
            # half-way - which is the bug this latch exists to fix.
            both_valid_now = self.yellow_valid_now and self.white_valid_now
            self.uncrossed_streak = self.uncrossed_streak + 1 \
                if (both_valid_now and not crossed) else 0

            if self.uncrossed_streak >= WRONG_WAY_CLEAR_FRAMES:
                self.clear_wrong_way(timed_out=False)
            return

        self.crossed_streak = self.crossed_streak + 1 if crossed else 0
        if self.crossed_streak > self.lane_hold_frames:
            self.wrong_way = True
            self.wrong_way_since = now
            self.uncrossed_streak = 0
            self.wrong_way_timed_out = False

    def update_lane_borders(self, now, yellow_x, white_x, yellow_valid, white_valid):
        """Fold one lane-border message into debounced wall state.

        The lane node always reports yellow as the left column and white as the right,
        so we treat them order-agnostically as two wall candidates and never attach
        left/right *semantics* to the colors (matters on the return leg of the U-turn).
        """
        a = clamp01(yellow_x)
        b = clamp01(white_x)
        left_x, right_x = (a, b) if a <= b else (b, a)
        # yellow<->wall association follows the ordering swap too.
        left_valid, right_valid = (yellow_valid, white_valid) if a <= b else (white_valid, yellow_valid)

        # Debounce: only open a side after lane_hold_frames consecutive invalid frames.
        if left_valid:
            self.left_invalid_streak = 0
            self.left_wall = left_x
        else:
            self.left_invalid_streak += 1
        if right_valid:
            self.right_invalid_streak = 0
            self.right_wall = right_x
        else:
            self.right_invalid_streak += 1

        self.left_open = self.left_invalid_streak > self.lane_hold_frames
        self.right_open = self.right_invalid_streak > self.lane_hold_frames

        # Same debounce, but keyed on colour rather than on side.
        if yellow_valid:
            self.yellow_invalid_streak = 0
            self.yellow_x = a
        else:
            self.yellow_invalid_streak += 1
        if white_valid:
            self.white_invalid_streak = 0
            self.white_x = b
        else:
            self.white_invalid_streak += 1

        self.yellow_seen = self.yellow_invalid_streak <= self.lane_hold_frames
        self.white_seen = self.white_invalid_streak <= self.lane_hold_frames

        # RAW per-frame validity, kept separately from the debounced *_seen flags.
        # The debounce deliberately lags by lane_hold_frames, which is right for
        # holding a corridor through a dropout but wrong for asking "can I see both
        # lines right now" - during a wrong-way turn the stale flag reports a
        # complete view for 12 frames after a line has actually left the frame.
        self.yellow_valid_now = bool(yellow_valid)
        self.white_valid_now = bool(white_valid)

        self.last_lane_time = now

    def update_duckies(self, now, duckies_raw):
        """Filter YOLO detections to real, big-enough duckie spans."""
        self.raw_duckie_count = len(duckies_raw)
        filtered = []
        small = 0
        for d in duckies_raw:
            if str(d.get("class_name", "duckie")).lower() != "duckie":
                continue
            xmin = clamp01(d.get("xmin", d.get("x_center", 0.5) - d.get("width", 0.0) / 2.0))
            xmax = clamp01(d.get("xmax", d.get("x_center", 0.5) + d.get("width", 0.0) / 2.0))
            ymin = clamp01(d.get("ymin", d.get("y_center", 0.5) - d.get("height", 0.0) / 2.0))
            ymax = clamp01(d.get("ymax", d.get("y_center", 0.5) + d.get("height", 0.0) / 2.0))
            if (xmax - xmin) < MIN_DUCKIE_WIDTH or (ymax - ymin) < MIN_DUCKIE_HEIGHT:
                small += 1
                continue
            filtered.append({"xmin": xmin, "xmax": xmax, "ymax": ymax})
        self.small_duckie_count = small
        if filtered:
            self.duckies = filtered
            self.last_duckie_time = now

    # ---- derived signals ----------------------------------------------------
    def lane_fresh(self, now):
        return (now - self.last_lane_time) < LANE_TIMEOUT

    def corridor(self, now):
        """Return (L, R, left_open, right_open): the hard steerable bounds.

        Also records corridor_source, purely so the dashboard can distinguish the
        three ways the bounds can end up wide open. They look identical on screen but
        mean very different things, and a lane_margin too large for the lane silently
        lands in the third one.
        """
        if not self.lane_fresh(now):
            # No recent lane data: treat both sides as open unknown geometry.
            self.corridor_source = "stale_open"
            return 0.0, 1.0, True, True
        left = 0.0 if self.left_open else clamp01(self.left_wall + self.lane_margin)
        right = 1.0 if self.right_open else clamp01(self.right_wall - self.lane_margin)
        if right - left < 2 * EPS:
            # Degenerate corridor (walls crossed / too tight): open it up so we can move.
            self.corridor_source = "degenerate_open"
            return 0.0, 1.0, True, True
        self.corridor_source = "walls"
        return left, right, self.left_open, self.right_open

    def active_duckies(self, now):
        """Duckies close enough to matter (hold window + react_ymax gate).

        A duckie only starts creating a blocked interval / triggering avoidance once
        its ymax (closeness) crosses react_ymax. Raise react_ymax to react closer/later,
        lower it to react farther/earlier.
        """
        if (now - self.last_duckie_time) > self.duckie_hold_time:
            return []
        return [d for d in self.duckies if d["ymax"] >= self.react_ymax]

    def blocked_spans(self, now, left, right, relax=1.0):
        """Inflated, corridor-clipped horizontal spans for each active duckie.

        ``relax`` (<=1.0) shrinks the safety inflation while stuck in ESCAPE so a
        marginal gap can eventually open (paired with the gap-width relaxation).
        """
        spans = []
        for d in self.active_duckies(now):
            # Inflate by everything that must NOT overlap the duckie: half the robot,
            # plus the clearance we want to keep. The result is the set of columns the
            # robot's CENTRE may not occupy, computed at this duckie's actual range.
            dist_cm = ymax_to_cm(d["ymax"])
            inflate = self.cm_to_image(
                self.robot_width_cm / 2.0 + self.safety_margin_cm, dist_cm) * relax
            lo = max(left, d["xmin"] - inflate)
            hi = min(right, d["xmax"] + inflate)
            if hi > lo:
                spans.append((lo, hi))
        spans.sort()
        merged = []
        for lo, hi in spans:
            if merged and lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        return merged

    @staticmethod
    def free_intervals(left, right, spans):
        free = []
        cursor = left
        for lo, hi in spans:
            if lo > cursor:
                free.append((cursor, lo))
            cursor = max(cursor, hi)
        if cursor < right:
            free.append((cursor, right))
        return free

    def front_ymax(self, now, probe_x):
        """Closeness of the nearest duckie whose span overlaps the front slice.

        The slice represents the robot's own width around the column it is driving
        toward - "is something in my path", not "is something near me". Its width is
        computed at each duckie's range, so it widens correctly as one approaches.
        """
        worst = 0.0
        for d in self.active_duckies(now):
            # Probe half-width is evaluated per duckie, at ITS range - a fixed slice
            # is far too narrow up close, which is how a duckie 8 cm ahead registered
            # as "not in my path" while the robot drove into it.
            half = self.robot_half_image(ymax_to_cm(d["ymax"]))
            if d["xmax"] >= probe_x - half and d["xmin"] <= probe_x + half:
                worst = max(worst, d["ymax"])
        return worst

    def lane_goal_x(self):
        """Goal column from the markings, using LINE COLOUR as a bearing reference.

        Inside the bulb there is no two-line corridor, but which colour is visible is
        an absolute orientation cue: white is the outer boundary, yellow the inner
        centre line. Seeing only white means facing outward, so aim left of it; seeing
        only yellow means facing inward, so aim right of it. That cue is the only
        heading reference that survives an ESCAPE_ROTATE, which is why the one-line
        case is steered from the marking rather than collapsed into heading_bias.

        The missing line is replaced by an ANCHOR - an assumption about where it would
        have been. Those anchors used to be hidden inside detect_lane_node's fallback
        constants, which made the U-turn radius an accident of the detector instead of
        something tunable. Defaults reproduce the previous numbers exactly, so this is
        a refactor first and a tuning surface second.

        Returns None only when neither line is visible. That is the genuinely blind
        case heading_bias exists for, and on this course it is reachable: duckies
        sitting over both markings get erased from the segmentation mask.
        """
        if self.yellow_seen and self.white_seen:
            # Midpoint is order-agnostic, so a crossed/skewed pose is still usable.
            return clamp01((self.yellow_x + self.white_x) / 2.0)
        if self.white_seen:
            return clamp01((self.white_x + self.yellow_anchor) / 2.0)
        if self.yellow_seen:
            return clamp01((self.yellow_x + self.white_anchor) / 2.0)
        return None

    # ---- main step ----------------------------------------------------------
    def clear_wrong_way(self, timed_out):
        self.wrong_way = False
        self.wrong_way_timed_out = timed_out
        self.crossed_streak = 0
        self.uncrossed_streak = 0

    def step(self, now):
        # Safety valve, evaluated here rather than in the lane callback so that it
        # still fires if lane_borders stops arriving altogether - a wedged detector
        # would otherwise leave the robot latched in a permanent rotation.
        if self.wrong_way and (now - self.wrong_way_since) > WRONG_WAY_MAX_ROTATE:
            self.clear_wrong_way(timed_out=True)

        left, right, left_open, right_open = self.corridor(now)
        # "Unknown geometry": no trustworthy corridor to follow (both sides open,
        # whether from debounced line loss or stale data). Not the same as a stale
        # topic - invalid-but-arriving frames still land here.
        lines_valid = self.lane_fresh(now) and not (left_open and right_open)
        unknown_geometry = not lines_valid

        # Goal column. Driven by the markings whenever ANY line is visible - including
        # the one-line case, where the colour carries the bearing. heading_bias is the
        # last resort, for when both markings are gone.
        lane_goal = self.lane_goal_x() if self.lane_fresh(now) else None
        if lane_goal is not None:
            goal_x = clamp(lane_goal, left, right)
            goal_source = ("lane_two_line" if (self.yellow_seen and self.white_seen)
                           else "lane_white_only" if self.white_seen
                           else "lane_yellow_only")
        else:
            goal_x = self.heading_bias(left, right)
            goal_source = "heading_bias_blind"

        # Escape relaxation: after rotating a while with no gap, progressively shrink
        # both the required gap width and the duckie inflation so a marginal gap opens.
        relax = 1.0
        if self.state == ESCAPE_ROTATE:
            stuck_for = now - self.state_since
            if stuck_for > self.escape_relax_after:
                over = stuck_for - self.escape_relax_after
                relax = max(0.25, 1.0 - 0.25 * over)  # decays; floor keeps a tiny gap usable

        spans = self.blocked_spans(now, left, right, relax)
        frees = self.free_intervals(left, right, spans)
        # Minimum gap, in cm of lateral wiggle room for the robot's centre, converted
        # at the range of the nearest duckie that is actually constraining us. The
        # robot's own width is already inside the inflation above, so this is control
        # slack only, not clearance.
        nearest_cm = min((ymax_to_cm(d["ymax"]) for d in self.active_duckies(now)),
                         default=YMAX_TO_CM[0][1])
        gap_min = self.cm_to_image(self.gap_min_cm, nearest_cm) * relax

        best_gap = self.pick_gap(frees, goal_x)
        best_width = (best_gap[1] - best_gap[0]) if best_gap else 0.0
        passable = best_gap is not None and best_width >= gap_min

        # Column we would actually steer toward. The front clearance checks below MUST
        # be measured here and not at goal_x: goal_x is the nominal route (lane centre
        # / U-turn bias), which by definition still points at the duckie we are trying
        # to drive around. Measuring there means steering into a gap never clears the
        # front check, so the bot re-enters ESCAPE_ROTATE a frame or two after
        # committing - drive a centimetre, rotate, repeat.
        drive_x = self.gap_target(best_gap, goal_x)
        front = self.front_ymax(now, drive_x)

        goal_blocked = self.column_blocked(goal_x, spans)
        front_slow = front >= self.front_slow_ymax
        front_block = front >= self.front_block_ymax

        # Commitment: once AVOID has started on a passable gap, hold it for
        # avoid_min_dwell even if the gap momentarily measures too narrow. Duckie
        # inflation grows with proximity, so a gap accepted at range always narrows as
        # we approach it - without this the manoeuvre aborts halfway through by
        # construction. front_block is deliberately NOT suppressed: something genuinely
        # close ahead still wins immediately.
        committed = (self.state == AVOID
                     and (now - self.state_since) < self.avoid_min_dwell)

        # Pointed backwards down a lane. Rotating in place is the right response and
        # not merely the convenient one: it cannot carry the robot further off the
        # course the way an arcing turn would, and it resolves itself - once the bot
        # has swung round far enough the colours uncross, wrong_way clears, and normal
        # lane following resumes. Routed through blocked_front so it reuses the escape
        # dwell and the never-freeze guard rather than adding a fourth state.
        # A marking square across the path blocks the robot exactly like an obstacle
        # does, and for the same reason there is no way through it. Rotating turns it
        # edge-on, which both clears the condition and leaves the robot travelling
        # parallel to the marking instead of over it.
        blocked_front = (front_block or self.wrong_way or self.head_on
                         or (not passable and not committed))
        want_avoid = goal_blocked or front_slow or unknown_geometry

        # --- transitions (single owner) --------------------------------------
        if self.state == ESCAPE_ROTATE and not self.can_exit_escape(now):
            # Latched: rotate at least escape_min_dwell before reconsidering.
            self.clear_since = None
        elif blocked_front:
            self.enter_escape(now, left, right, frees)
            self.clear_since = None
        elif want_avoid:
            self.enter(AVOID, now)
            self.clear_since = None
        else:
            # Road ahead is clear. Fall back to CRUISE only after a short clear hold,
            # so a one-frame "all clear" glimpse mid-maneuver doesn't snap us back.
            if self.clear_since is None:
                self.clear_since = now
            if self.state != CRUISE and (now - self.clear_since) >= CLEAR_HOLD_TIME:
                self.enter(CRUISE, now)
            elif self.state == ESCAPE_ROTATE:
                # Cleared during escape but within the hold window: start moving via AVOID
                # rather than continuing to rotate in place.
                self.enter(AVOID, now)

        # --- outputs per state -----------------------------------------------
        if self.state == ESCAPE_ROTATE:
            v = 0.0
            omega = self.escape_dir * self.omega_rotate
            target_x = goal_x
            if self.wrong_way:
                reason = "escape_rotate_wrong_way"
            elif self.head_on:
                reason = "escape_rotate_head_on_line"
            else:
                reason = "escape_rotate_relaxed" if relax < 1.0 else "escape_rotate_no_gap"
        elif self.state == AVOID:
            target_x = self.avoid_target(best_gap, goal_x)
            # In unknown geometry cap at the creep floor - we don't have a validated
            # corridor, so move minimally while the U-turn bias and gaps steer us.
            v = self.v_min if unknown_geometry else self.avoid_speed(front)
            omega = self.steer(target_x, left, right)
            reason = "avoid_unknown_geometry_uturn" if unknown_geometry else "avoid_duckie"
        else:  # CRUISE
            target_x = goal_x
            v = self.v_cruise
            omega = self.steer(target_x, left, right)
            reason = "cruise_lane_follow"

        self.last_target_x = target_x

        # --- output guard: single writer, never-freeze invariant -------------
        omega = clamp(omega, -self.omega_max, self.omega_max)
        if abs(v) < EPS and abs(omega) < EPS:
            # omega_rotate is already floored at MIN_ESCAPE_OMEGA in __init__, so it is
            # strong enough to break static friction rather than just buzzing the motors.
            omega = self.escape_dir * self.omega_rotate
            reason = "never_freeze_guard"

        debug = self.build_debug(
            now, left, right, left_open, right_open, spans, frees,
            best_gap, goal_x, target_x, front, reason, v, omega,
            drive_x, committed, passable, goal_source,
        )
        return v, omega, debug

    # ---- helpers ------------------------------------------------------------
    def heading_bias(self, left, right):
        """Fallback goal column when lines are untrusted: bias into the left U-turn.

        Single seam where an encoder-measured turn could later replace the constant.
        """
        return clamp(left + 0.3 * (right - left), left, right)

    def steer(self, target_x, left, right):
        target_x = clamp(target_x, left, right)
        return self.k_steer * (0.5 - target_x) * 2.0

    def avoid_speed(self, front):
        if front <= self.front_slow_ymax:
            return self.v_avoid
        if front >= self.front_block_ymax:
            return self.v_min
        t = (front - self.front_slow_ymax) / max(EPS, self.front_block_ymax - self.front_slow_ymax)
        return self.v_avoid + t * (self.v_min - self.v_avoid)

    @staticmethod
    def column_blocked(x, spans):
        return any(lo <= x <= hi for lo, hi in spans)

    def pick_gap(self, frees, goal_x):
        """Widest free interval, tie-broken toward the goal column.

        The interval we are already driving into gets a width bonus, so two similar
        gaps cannot trade places frame to frame and drag the target across the image.
        """
        if not frees:
            return None

        def score(iv):
            width = iv[1] - iv[0]
            if iv[0] <= self.last_target_x <= iv[1]:
                width += GAP_STICKY_BONUS
            return (width, -abs((iv[0] + iv[1]) / 2.0 - goal_x))

        return max(frees, key=score)

    @staticmethod
    def gap_target(best_gap, goal_x):
        """Aim at the goal column, clamped to stay safely inside the chosen gap.

        This keeps us heading where we want to go (lane target / U-turn bias) while
        holding a fixed clearance from the gap edges (which are inflated duckie
        boundaries) - better for the "keep distance" requirement than blindly aiming
        at the gap center, and it preserves the goal bias in a wide-open gap.

        Pure and side-effect free, so ``step`` can ask "where would we drive?" for the
        front clearance check without disturbing the avoid-side hysteresis.
        """
        if best_gap is None:
            return goal_x
        lo, hi = best_gap
        inset = min(GAP_INSET, (hi - lo) / 2.0)
        lo_s, hi_s = lo + inset, hi - inset
        return (lo + hi) / 2.0 if hi_s < lo_s else clamp(goal_x, lo_s, hi_s)

    def avoid_target(self, best_gap, goal_x):
        """``gap_target`` plus directional hysteresis. Call once per tick."""
        target = self.gap_target(best_gap, goal_x)

        # Directional hysteresis: resist a small cross-center flip vs the last command
        # to avoid frame-to-frame left/right chatter between two similar gaps.
        side = 1.0 if target >= 0.5 else -1.0
        if self.avoid_side != 0.0 and side != self.avoid_side and \
                abs(target - self.last_target_x) < HYSTERESIS_MARGIN:
            return self.last_target_x
        self.avoid_side = side
        return target

    def enter(self, state, now):
        if self.state != state:
            self.state = state
            self.state_since = now
            if state != AVOID:
                self.avoid_side = 0.0

    def enter_escape(self, now, left, right, frees):
        if self.state == ESCAPE_ROTATE:
            # Latch direction until the minimum dwell elapses (anti-chatter).
            return
        self.state = ESCAPE_ROTATE
        self.state_since = now
        self.escape_dir = self.choose_escape_dir(left, right, frees)

    def choose_escape_dir(self, left, right, frees):
        """Rotate toward the side with more clearance; tie -> left (U-turn)."""
        if self.left_open and not self.right_open:
            return 1.0   # open on the left -> turn left into it
        if self.right_open and not self.left_open:
            return -1.0
        mid = (left + right) / 2.0
        left_clear = sum(min(hi, mid) - lo for lo, hi in frees if lo < mid)
        right_clear = sum(hi - max(lo, mid) for lo, hi in frees if hi > mid)
        if right_clear > left_clear + EPS:
            return -1.0
        return 1.0

    def can_exit_escape(self, now):
        return (now - self.state_since) >= self.escape_min_dwell

    def build_debug(self, now, left, right, left_open, right_open, spans, frees,
                    best_gap, goal_x, target_x, front, reason, v, omega,
                    drive_x, committed, passable, goal_source="unknown"):
        return {
            "state": self.state,
            "reason": reason,
            "avoidance_active": self.state != CRUISE,
            "plan_y_min": self.react_ymax,
            "plan_y_max": PLAN_Y_MAX,
            "lane_left": left,
            "lane_right": right,
            "corridor_left_open": left_open,
            # Raw wall positions BEFORE lane_margin, so the dashboard can draw the
            # inset the margin actually produces rather than only its result.
            "wall_left": self.left_wall,
            "wall_right": self.right_wall,
            "yellow_x": self.yellow_x,
            "white_x": self.white_x,
            "lane_margin": self.lane_margin,
            "corridor_source": self.corridor_source,
            "corridor_right_open": right_open,
            "blocked_intervals": [list(s) for s in spans],
            "free_intervals": [list(f) for f in frees],
            "selected_free_interval": list(best_gap) if best_gap else None,
            "target_x": target_x,
            "lane_target_x": goal_x,
            "nearest_front_ymax": front,
            # front is measured at drive_x (where we steer), not goal_x (the route).
            "front_probe_x": drive_x,
            "gap_passable": passable,
            "avoid_committed": committed,
            "num_raw_duckies": self.raw_duckie_count,
            "num_active_duckies": len(self.active_duckies(now)),
            "num_relevant_duckies": len(spans),
            "num_small_duckies_filtered": self.small_duckie_count,
            "lane_source": goal_source,
            "yellow_seen": self.yellow_seen,
            "white_seen": self.white_seen,
            "head_on": self.head_on,
            "lines_crossed": self.lines_crossed,
            "wrong_way": self.wrong_way,
            "crossed_streak": self.crossed_streak,
            "uncrossed_streak": self.uncrossed_streak,
            "wrong_way_timed_out": self.wrong_way_timed_out,
            "left_invalid_streak": self.left_invalid_streak,
            "right_invalid_streak": self.right_invalid_streak,
            "v": v,
            "omega": omega,
        }


class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        params = self.load_config()
        self.planner = GapPlanner(params)
        self.v = 0.0
        self.omega = 0.0

        # Readiness gate: hold still until every sensing node has published at least
        # once. The duckie node loads YOLO on startup (slow), so without this the
        # controller would start driving on stale/default data before it can see.
        self.got_lane = False
        self.got_borders = False
        self.got_duckies = False

        # Dry run: plan and publish debug exactly as normal, but never write a drive
        # command. The dashboard overlay is fed entirely by /debug/free_path_plan,
        # which only this node publishes, so simply not starting the node would blind
        # the very view you tune against. The publisher is not registered at all, so
        # `rostopic info car_cmd_switch_node/cmd` shows no publisher.
        self.publish_cmd_enabled = bool(rospy.get_param("~publish_cmd", True))

        base = f"/{self.vehicle_name}"
        if self.publish_cmd_enabled:
            self.pub_cmd = rospy.Publisher(f"{base}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        else:
            self.pub_cmd = None
            rospy.logwarn(
                f"[{node_name}] DRY RUN (publish_cmd=false): planning and debug output "
                f"are live, no drive command is published. The robot will not move."
            )
        self.pub_debug = rospy.Publisher(f"{base}/debug/free_path_plan", String, queue_size=1)

        rospy.Subscriber(f"{base}/detect/lane", Float64, self.cb_lane, queue_size=1)
        rospy.Subscriber(f"{base}/detect/lane_borders", String, self.cb_lane_borders, queue_size=1)
        rospy.Subscriber(f"{base}/detect/duckie_BB", String, self.cb_obstacles, queue_size=1)

        # Live tuning. Started last so the planner and every subscriber already
        # exist by the time the server fires its initial callback.
        self.reconfigure_server = None
        if DYNAMIC_RECONFIGURE:
            self.reconfigure_server = Server(ControlLaneConfig, self.cb_reconfigure)
            # The cfg's defaults are a BUILD-TIME snapshot of the JSON. Editing the
            # JSON and restarting without rebuilding would otherwise let the server's
            # initial callback silently revert the planner to the stale snapshot -
            # the node would log the JSON values it loaded, then quietly run others.
            # Pushing the freshly loaded values in makes the JSON authoritative and
            # leaves the sliders showing what is actually in force.
            self.reconfigure_server.update_configuration(params)
            rospy.loginfo(f"[{node_name}] live tuning enabled: rosrun rqt_reconfigure rqt_reconfigure")
        else:
            rospy.logwarn(
                f"[{node_name}] dynamic_reconfigure unavailable - running on the JSON "
                f"values only. Rebuild the workspace to enable live tuning."
            )

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"[{node_name}] follow-the-gap controller ready for {self.vehicle_name}")

    def cb_reconfigure(self, config, level):
        """Apply slider changes to the live planner.

        Called once at startup with the cfg's build-time defaults, then immediately
        again via update_configuration() with the values actually loaded from the
        JSON. The JSON wins, so a config edit does not need a rebuild to take effect.
        """
        self.planner.set_params(config)

        # Re-checked on every change, not just at startup: this is precisely the
        # constraint a slider session is likely to break, and the symptom (every
        # accepted gap aborts a frame or two in) looks like a tuning problem rather
        # than an invalid combination.
        if config["react_ymax"] >= config["front_slow_ymax"]:
            rospy.logwarn(
                f"[{self.node_name}] react_ymax ({config['react_ymax']}) >= front_slow_ymax "
                f"({config['front_slow_ymax']}): duckies start mattering only after the "
                f"speed ramp has begun. Lower react_ymax or raise front_slow_ymax."
            )
        return config

    def load_config(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "../config/control_lane_node.json")
        defaults = {
            "v_cruise": 0.10, "v_avoid": 0.06, "v_min": 0.06, "k_steer": 6.0,
            "omega_rotate": 3.0, "omega_max": 4.0, "lane_margin": 0.08,
            "camera_width_k": 0.382, "robot_width_cm": 13.0,
            "safety_margin_cm": 5.0, "gap_min_cm": 4.0,
            "react_ymax": 0.62, "front_slow_ymax": 0.72, "front_block_ymax": 0.88,
            "escape_min_dwell": 0.5, "escape_relax_after": 2.0,
            "lane_hold_frames": 12, "duckie_hold_time": 1.0,
            "avoid_min_dwell": 0.8,
            "yellow_anchor": 0.047, "white_anchor": 0.948,
        }
        params = dict(defaults)
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            group = cfg.get("parameters", cfg).get("controller", {})
            for key in defaults:
                if key in group and "default" in group[key]:
                    params[key] = float(group[key]["default"])
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] Could not load config ({e}); using defaults.")
        rospy.loginfo(f"[{self.node_name}] params: {params}")

        # Sanity: at contact range the robot fills a large part of the frame, so a
        # badly wrong camera_width_k silently reproduces the old failure.
        half_8cm = params["robot_width_cm"] / 2.0 * params["camera_width_k"] / 8.0
        if not 0.15 <= half_8cm <= 0.45:
            rospy.logwarn(
                f"[{self.node_name}] robot half-width at 8cm computes to {half_8cm:.3f} "
                f"of the frame, which looks wrong (expected ~0.2-0.35). Check "
                f"camera_width_k ({params['camera_width_k']}) and robot_width_cm "
                f"({params['robot_width_cm']})."
            )
        return params

    # ---- callbacks: cache into the planner ----------------------------------
    def cb_lane(self, msg):
        self.planner.update_lane_error(msg.data)
        self.got_lane = True

    def cb_lane_borders(self, msg):
        try:
            data = json.loads(msg.data)
            # `valid` means "the lane_center/error derived from these two lines is
            # meaningful", which is false when the lines come back in crossed order.
            # That does NOT mean the lines weren't seen: per-colour validity is
            # independent of ordering, and the planner sorts the two positions itself.
            # Discarding both here (as this used to) threw away a perfectly good
            # corridor in exactly the skewed pose where it matters most.
            self.planner.update_lane_borders(
                rospy.Time.now().to_sec(),
                data.get("yellow_x", 0.05), data.get("white_x", 0.95),
                bool(data.get("yellow_valid", True)), bool(data.get("white_valid", True)),
            )
            # Crossed colour order means the robot is pointed backwards down a
            # lane. The detector has always published this; nothing consumed it.
            self.planner.set_lines_crossed(
                rospy.Time.now().to_sec(), bool(data.get("lines_crossed", False)))
            self.planner.set_head_on(bool(data.get("white_head_on", False))
                                     or bool(data.get("yellow_head_on", False)))
            self.got_borders = True
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] bad lane_borders: {e}")

    def cb_obstacles(self, msg):
        try:
            data = json.loads(msg.data)
            duckies = data.get("duckies", [])
            if not duckies and data.get("detected", False) and str(data.get("class_name", "")).lower() == "duckie":
                duckies = [data]
            self.planner.update_duckies(rospy.Time.now().to_sec(), duckies)
            # duckie_BB arriving at all means YOLO has loaded and is publishing.
            self.got_duckies = True
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] bad duckie_BB: {e}")

    def sensors_ready(self):
        return self.got_lane and self.got_borders and self.got_duckies

    # ---- control loop -------------------------------------------------------
    def run(self):
        rate = rospy.Rate(10)
        announced_ready = False
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()

            if not self.sensors_ready():
                # Stay parked (this is the one legitimate v=0/omega=0: not yet armed)
                # and keep the dashboard informed while the sensing nodes warm up.
                self.v, self.omega = 0.0, 0.0
                self.publish_cmd(0.0, 0.0)
                self.publish_plan({
                    "state": "WAITING",
                    "reason": "waiting_for_sensors",
                    "avoidance_active": False,
                    "waiting_lane": not self.got_lane,
                    "waiting_lane_borders": not self.got_borders,
                    "waiting_duckies": not self.got_duckies,
                    "v": 0.0, "omega": 0.0,
                })
                rate.sleep()
                continue

            if not announced_ready:
                rospy.loginfo(f"[{self.node_name}] all sensors online; arming controller.")
                announced_ready = True

            self.v, self.omega, debug = self.planner.step(now)
            self.publish_cmd(self.v, self.omega)
            self.publish_plan(debug)
            rate.sleep()

    def publish_plan(self, debug):
        """Publish the planner snapshot, tagged with the drive-command mode.

        dry_run rides along on every message because the dashboard otherwise cannot
        tell the two stationary cases apart: a robot held still by dry run looks
        exactly like one that computed v=0 and is stuck.
        """
        debug["dry_run"] = not self.publish_cmd_enabled
        self.pub_debug.publish(String(data=json.dumps(debug)))

    def publish_cmd(self, v, omega):
        if self.pub_cmd is None:
            # Dry run. self.v/self.omega are still set by the caller and still travel
            # out on free_path_plan, so the dashboard shows what WOULD be commanded.
            rospy.loginfo_throttle(
                5.0, f"[{self.node_name}] DRY RUN - would command v={v:.3f} omega={omega:.3f}")
            return
        twist = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()
        twist.v = v
        twist.omega = omega
        self.pub_cmd.publish(twist)

    def shutdown(self):
        if self.pub_cmd is None:
            rospy.loginfo(f"[{self.node_name}] shutting down (dry run); nothing was ever commanded.")
            return
        rospy.loginfo(f"[{self.node_name}] shutting down; commanding zero velocity.")
        self.pub_cmd.publish(Twist2DStamped(v=0.0, omega=0.0))


if __name__ == "__main__":
    try:
        node = ControlLaneNode("control_lane_node")
        node.run()
    except rospy.ROSInterruptException:
        pass
