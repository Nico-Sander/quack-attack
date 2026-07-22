#!/usr/bin/env python3

"""GapPlanner behaviour tests.

GapPlanner is rospy-free and takes ``now`` as an argument, so every case here is a
deterministic loop with no clock and no ROS master. Only the import needs stubbing.

Focus: the wrong-way detection, and the invariants it must not break.
"""

import json
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
CONFIG = os.path.join(HERE, "..", "config", "control_lane_node.json")


def _stub_ros():
    for name in ("rospy", "std_msgs", "std_msgs.msg",
                 "duckietown_msgs", "duckietown_msgs.msg"):
        sys.modules.setdefault(name, types.ModuleType(name))
    for attr in ("Float64", "String"):
        setattr(sys.modules["std_msgs.msg"], attr, object)
    sys.modules["duckietown_msgs.msg"].Twist2DStamped = object


_stub_ros()
sys.path.insert(0, SRC)
import control_lane_node as cln  # noqa: E402


@pytest.fixture
def params():
    with open(CONFIG) as fh:
        ctrl = json.load(fh)["parameters"]["controller"]
    return {k: float(v["default"]) for k, v in ctrl.items()}


def feed_lane(planner, t, yellow_x, white_x, crossed=False,
              yellow_valid=True, white_valid=True):
    """One lane_borders message, in the same order the node applies them."""
    planner.update_lane_borders(t, yellow_x, white_x, yellow_valid, white_valid)
    planner.set_lines_crossed(t, crossed)


def settle(planner, frames, t0=1.0, **kwargs):
    """Feed `frames` identical lane messages; returns the final timestamp."""
    t = t0
    for _ in range(frames):
        t += 0.1
        feed_lane(planner, t, **kwargs)
    return t


# --- wrong-way detection -------------------------------------------------


def test_correct_direction_is_not_wrong_way(params):
    """Yellow left of white is normal travel and must never trip the detector."""
    p = cln.GapPlanner(params)
    settle(p, 40, yellow_x=0.229, white_x=0.786, crossed=False)
    assert not p.wrong_way
    assert not p.lines_crossed


def test_crossed_order_trips_wrong_way_after_debounce(params):
    """White left of yellow means the robot is pointed back down the lane."""
    p = cln.GapPlanner(params)
    hold = int(params["lane_hold_frames"])

    # Crossed, but not yet for long enough: must NOT fire.
    settle(p, hold, yellow_x=0.786, white_x=0.229, crossed=True)
    assert p.lines_crossed, "raw flag should follow the detector immediately"
    assert not p.wrong_way, "must not fire before lane_hold_frames"

    # Sustained: fires.
    settle(p, 5, yellow_x=0.786, white_x=0.229, crossed=True)
    assert p.wrong_way


def test_single_crossed_frame_does_not_trip(params):
    """A transient crossed frame while skewed must not spin the robot up."""
    p = cln.GapPlanner(params)
    settle(p, 30, yellow_x=0.229, white_x=0.786, crossed=False)
    feed_lane(p, 5.0, yellow_x=0.786, white_x=0.229, crossed=True)
    assert not p.wrong_way
    # and it clears again
    settle(p, 5, t0=5.0, yellow_x=0.229, white_x=0.786, crossed=False)
    assert p.crossed_streak == 0


def test_crossed_ignored_when_only_one_line_seen(params):
    """With one line missing the other's position is a fallback constant, so the
    ordering carries no information and must not be believed."""
    p = cln.GapPlanner(params)
    hold = int(params["lane_hold_frames"])
    settle(p, hold + 20, yellow_x=0.786, white_x=0.229,
           crossed=True, white_valid=False)
    assert not p.white_seen
    assert not p.wrong_way, "one-line ordering must not trip wrong_way"


def test_wrong_way_drives_escape_rotate_and_recovers(params):
    """It must rotate rather than follow the lane centre out of the course, and it
    must resume normal driving once the colours uncross."""
    p = cln.GapPlanner(params)
    hold = int(params["lane_hold_frames"])

    t = settle(p, hold + 5, yellow_x=0.786, white_x=0.229, crossed=True)
    p.update_duckies(t, [])
    v, omega, dbg = p.step(t)

    assert dbg["state"] == cln.ESCAPE_ROTATE
    assert dbg["reason"] == "escape_rotate_wrong_way"
    assert v == 0.0 and abs(omega) > 0.0, "rotate in place, never freeze"

    # Once round the right way, wrong_way clears and it leaves ESCAPE_ROTATE.
    t = settle(p, hold + 40, t0=t, yellow_x=0.229, white_x=0.786, crossed=False)
    assert not p.wrong_way
    for _ in range(30):
        t += 0.1
        p.update_duckies(t, [])
        v, omega, dbg = p.step(t)
    assert dbg["state"] != cln.ESCAPE_ROTATE, "must recover once facing forward"


# --- invariants the fix must not break -----------------------------------


def test_never_freeze_holds_while_wrong_way(params):
    """The output guard must survive the new transition path."""
    p = cln.GapPlanner(params)
    t = settle(p, 40, yellow_x=0.786, white_x=0.229, crossed=True)
    for _ in range(60):
        t += 0.1
        p.update_duckies(t, [])
        v, omega, dbg = p.step(t)
        assert not (abs(v) < 1e-3 and abs(omega) < 1e-3), \
            f"froze at {dbg['reason']}"
        assert abs(omega) <= params["omega_max"] + 1e-9
        assert v >= 0.0


def test_wrong_way_does_not_alter_the_corridor(params):
    """It changes the FSM transition only; wall estimates stay untouched."""
    p = cln.GapPlanner(params)
    t = settle(p, 40, yellow_x=0.786, white_x=0.229, crossed=True)
    before = p.corridor(t)
    p.set_lines_crossed(t, True)
    assert p.corridor(t) == before


# --- general planner regressions (previously a throwaway scratch harness) -


def duckie(xmin, xmax, ymax):
    return {"xmin": xmin, "xmax": xmax, "ymin": ymax - 0.2, "ymax": ymax,
            "class_name": "duckie", "width": xmax - xmin, "height": 0.2,
            "x_center": (xmin + xmax) / 2.0, "y_center": ymax - 0.1}


SCENARIOS = {
    "clear road":           ([], True),
    "one duckie centre":    ([duckie(0.40, 0.60, 0.75)], True),
    "duckie very close":    ([duckie(0.35, 0.65, 0.95)], True),
    "two duckies with gap": ([duckie(0.05, 0.35, 0.8), duckie(0.65, 0.95, 0.8)], True),
    "wall of duckies":      ([duckie(0.0, 0.5, 0.9), duckie(0.45, 1.0, 0.9)], True),
    "no lane lines":        ([duckie(0.4, 0.6, 0.8)], False),
    "nothing at all":       ([], False),
}


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_never_freeze_and_limits_hold(params, name):
    """The core invariant: never commanded (v==0 AND omega==0), omega within cap,
    and v never negative now that the reverse manoeuvre is gone."""
    ducks, lines = SCENARIOS[name]
    p = cln.GapPlanner(params)
    t = 0.0
    for _ in range(120):                      # 12 s at 10 Hz
        t += 0.1
        if lines:
            feed_lane(p, t, yellow_x=0.229, white_x=0.786)
        p.update_duckies(t, ducks)
        p.update_lane_error(0.1)
        v, omega, dbg = p.step(t)
        assert not (abs(v) < 1e-3 and abs(omega) < 1e-3), f"{name}: froze ({dbg['reason']})"
        assert abs(omega) <= params["omega_max"] + 1e-9, f"{name}: omega over cap"
        assert v >= 0.0, f"{name}: negative v ({v})"


def test_tunables_match_config_exactly(params):
    """Adding a JSON param without adding it to TUNABLES silently makes it inert."""
    assert set(cln.GapPlanner.TUNABLES) == set(params), \
        f"mismatch: {set(cln.GapPlanner.TUNABLES) ^ set(params)}"


def test_live_param_update_preserves_state(params):
    """A slider move mid-manoeuvre must not reset the FSM or forget sensor state."""
    p = cln.GapPlanner(params)
    t = 1.0
    for _ in range(12):
        t += 0.1
        feed_lane(p, t, yellow_x=0.229, white_x=0.786)
        p.update_duckies(t, [duckie(0.40, 0.60, 0.75)])
        p.step(t)

    state, since = p.state, p.state_since
    walls = (p.left_wall, p.right_wall)
    ducks = len(p.duckies)

    p.set_params({"v_cruise": 0.22, "safety_margin_cm": 8.0})

    assert p.v_cruise == 0.22 and p.safety_margin_cm == 8.0
    assert p.state == state and p.state_since == since
    assert (p.left_wall, p.right_wall) == walls
    assert len(p.duckies) == ducks
    assert p.v_avoid == params["v_avoid"], "untouched param changed"


def test_omega_rotate_floor_survives_live_edit(params):
    """Below the floor the motors buzz without turning, which would break
    never-freeze in practice while looking fine in the command."""
    p = cln.GapPlanner(params)
    p.set_params({"omega_rotate": 0.1})
    assert p.omega_rotate == cln.MIN_ESCAPE_OMEGA


# --- head-on markings ----------------------------------------------------


def test_head_on_needs_debounce_then_blocks(params):
    p = cln.GapPlanner(params)
    for _ in range(cln.HEAD_ON_FRAMES):
        p.set_head_on(True)
    assert not p.head_on, "must not fire before HEAD_ON_FRAMES"
    p.set_head_on(True)
    assert p.head_on


def test_head_on_drives_escape_rotate(params):
    """A marking square across the path is a wall: rotate, do not drive over it."""
    p = cln.GapPlanner(params)
    t = settle(p, 20, yellow_x=0.229, white_x=0.786)
    for _ in range(cln.HEAD_ON_FRAMES + 1):
        p.set_head_on(True)
    p.update_duckies(t, [])
    v, omega, dbg = p.step(t)
    assert dbg["state"] == cln.ESCAPE_ROTATE
    assert dbg["reason"] == "escape_rotate_head_on_line"
    assert v == 0.0 and abs(omega) > 0.0


def test_head_on_clears_when_line_turns_edge_on(params):
    """Not latched, unlike wrong_way: rotating makes the line vertical again, and
    the robot should resume once it is travelling parallel to the marking."""
    p = cln.GapPlanner(params)
    for _ in range(cln.HEAD_ON_FRAMES + 1):
        p.set_head_on(True)
    assert p.head_on
    p.set_head_on(False)
    assert not p.head_on, "must clear immediately once the line is edge-on"


def test_never_freeze_holds_while_head_on(params):
    p = cln.GapPlanner(params)
    t = 0.0
    for _ in range(60):
        t += 0.1
        feed_lane(p, t, yellow_x=0.229, white_x=0.786)
        p.set_head_on(True)
        p.update_duckies(t, [])
        v, omega, dbg = p.step(t)
        assert not (abs(v) < 1e-3 and abs(omega) < 1e-3), f"froze at {dbg['reason']}"
        assert abs(omega) <= params["omega_max"] + 1e-9
