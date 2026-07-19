#!/usr/bin/env python3

"""The reentrancy guards must never wedge.

A guard that is set but not reset stops the node detecting anything, forever,
with no error after the first traceback. That failure is silent and total, so it
is worth a test even though these nodes are otherwise ROS-coupled.

Both node modules are imported with their heavy dependencies stubbed; only the
guard logic is exercised.
"""

import os
import sys
import types

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "src"))


def _stub_modules():
    for name in ("rospy", "cv2", "torch", "ultralytics", "sensor_msgs",
                 "sensor_msgs.msg", "std_msgs", "std_msgs.msg",
                 "duckietown_msgs", "duckietown_msgs.msg",
                 "segmentation_models_pytorch", "albumentations",
                 "albumentations.pytorch"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["ultralytics"].YOLO = object
    sys.modules["sensor_msgs.msg"].CompressedImage = object
    for attr in ("Float64", "String", "Bool"):
        setattr(sys.modules["std_msgs.msg"], attr, object)
    setattr(sys.modules["duckietown_msgs.msg"], "Twist2DStamped", object)
    rospy = sys.modules["rospy"]
    rospy.logwarn_throttle = lambda *a, **k: None
    rospy.loginfo_throttle = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.loginfo = lambda *a, **k: None


_stub_modules()

import detect_obstacle_node as OBS  # noqa: E402


def make_obstacle_node():
    """Bare instance - __init__ needs a live ROS master and a YOLO model."""
    node = OBS.DetectObstacleNode.__new__(OBS.DetectObstacleNode)
    node.node_name = "test"
    node.is_running = False
    node.dropped_frames = 0
    return node


def test_guard_resets_after_a_normal_frame():
    node = make_obstacle_node()
    node._process_image = lambda msg: None
    node.cb_process_image(object())
    assert node.is_running is False


def test_guard_resets_after_an_exception():
    """The wedge case: without try/finally the node dies silently right here."""
    node = make_obstacle_node()

    def boom(msg):
        raise RuntimeError("inference blew up")

    node._process_image = boom
    with pytest.raises(RuntimeError):
        node.cb_process_image(object())
    assert node.is_running is False, "guard stuck True - node would never detect again"

    # And it must still accept the next frame.
    seen = []
    node._process_image = lambda msg: seen.append(msg)
    node.cb_process_image(object())
    assert len(seen) == 1


def test_reentrant_frame_is_dropped_not_queued():
    """A frame arriving mid-inference is discarded, which is what bounds latency."""
    node = make_obstacle_node()
    calls = []

    def slow(msg):
        calls.append(msg)
        # Simulate a frame arriving while this one is still being processed.
        node.cb_process_image("reentrant")

    node._process_image = slow
    node.cb_process_image("first")
    assert calls == ["first"], "reentrant frame was processed instead of dropped"
    assert node.dropped_frames == 1
    assert node.is_running is False
