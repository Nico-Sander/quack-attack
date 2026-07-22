"""
The bot must not pull away before gate detection is live.

Found on the robot: a mapping run started with a gate already in frame on the
first street, and that gate was never recorded. The cause is a race between two
node startups that nothing was synchronising:

    detect_lane    imports torch, loads and traces the U-Net   ~4.5 s
    detect_signs   builds the tagStandard52h13 decode table     ~5.5 s

Motion was gated on the first one and not on the second. control_wheels leaves
v at 0 until a /detect/lane message arrives -- which cannot happen before
detect_lane is ready -- and then drives at max_vel immediately. detect_signs
came up about a second later, so the bot covered ~0.2 m of its first street
with no tag detection running. Both times were measured in the container.

These tests pin the gate itself: no ready flag, no wheel command. They stub ROS
rather than run it, so they need no master, no camera and no robot -- the same
approach as tests/render_example_visualization.py.
"""

import sys
import types

import pytest


# ---------------------------------------------------------------------------
# A ROS stand-in, just big enough for control_wheels
# ---------------------------------------------------------------------------

class FakeMessage:
    """Stands in for any std_msgs/duckietown_msgs message."""

    def __init__(self, **fields):
        self.v = 0.0
        self.omega = 0.0
        self.state = ""
        self.data = None
        self.header = types.SimpleNamespace(stamp=0.0)

        for name, value in fields.items():
            setattr(self, name, value)


class FakePublisher:
    def __init__(self, *_args, **_kwargs):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeClock:
    """
    A clock that advances one control tick per reading.

    The PID divides by dt, so a frozen clock would make it skip its update
    entirely and hide whatever it is being asked about.
    """

    TICK = 1.0 / 30.0

    def __init__(self):
        self.seconds = 0.0

    def now(self):
        self.seconds += self.TICK
        return types.SimpleNamespace(to_sec=lambda seconds=self.seconds: seconds)


def install_fake_ros(params):
    """
    Puts a minimal fake `rospy`, `std_msgs.msg` and `duckietown_msgs.msg` in
    sys.modules, then returns the freshly imported control_wheels module.

    `params` backs rospy.get_param, so a test can set ~wait_for_signs.
    """
    rospy = types.ModuleType("rospy")

    rospy.init_node = lambda *a, **k: None
    rospy.get_param = lambda name, default=None: params.get(name, default)
    rospy.Subscriber = lambda *a, **k: None
    rospy.Publisher = lambda *a, **k: FakePublisher()
    rospy.Time = FakeClock()
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.on_shutdown = lambda *a, **k: None
    rospy.is_shutdown = lambda: True
    rospy.signal_shutdown = lambda *a, **k: None
    rospy.ROSInterruptException = RuntimeError

    for name in ("loginfo", "logwarn", "logerr", "logdebug",
                 "loginfo_throttle", "logwarn_throttle", "logdebug_throttle"):
        setattr(rospy, name, lambda *a, **k: None)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    duckietown_msgs = types.ModuleType("duckietown_msgs")
    duckietown_msgs.msg = types.ModuleType("duckietown_msgs.msg")

    for module, names in ((std_msgs.msg, ("Bool", "Float64", "Int32")),
                          (duckietown_msgs.msg, ("Twist2DStamped", "FSMState"))):
        for name in names:
            setattr(module, name, FakeMessage)

    sys.modules["rospy"] = rospy
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs.msg
    sys.modules["duckietown_msgs"] = duckietown_msgs
    sys.modules["duckietown_msgs.msg"] = duckietown_msgs.msg
    sys.modules.pop("control_wheels", None)

    import control_wheels

    return control_wheels


@pytest.fixture
def node_factory():
    """Builds a ControlWheelsNode with ROS faked out, then cleans up."""
    saved = {name: sys.modules.get(name)
             for name in ("rospy", "std_msgs", "std_msgs.msg",
                          "duckietown_msgs", "duckietown_msgs.msg",
                          "control_wheels")}

    def build(**params):
        module = install_fake_ros(params)
        return module.ControlWheelsNode("control_wheels_node"), module

    yield build

    for name, module in saved.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def lane_error(value):
    return FakeMessage(data=value)


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------

def test_wheels_stay_still_until_signs_are_ready(node_factory):
    """The failure that was seen on the track: driving with no detector up."""
    node, _module = node_factory()

    # detect_lane is up and steering hard; detect_signs is still building its
    # decode table. This is the ~1 s window the bot used to drive through.
    node._cb_lane(lane_error(0.4))

    twist = node._compute_twist()

    assert twist.v == 0.0
    assert twist.omega == 0.0


def test_wheels_are_released_once_signs_report_ready(node_factory):
    node, module = node_factory()

    node._cb_signs_ready(FakeMessage(data=True))
    node._cb_lane(lane_error(0.0))

    twist = node._compute_twist()

    assert twist.v > 0.0
    assert node.active_mode == module.DriveMode.LANE_FOLLOWING


def test_a_false_ready_flag_does_not_release_the_wheels(node_factory):
    node, _module = node_factory()

    node._cb_signs_ready(FakeMessage(data=False))
    node._cb_lane(lane_error(0.0))

    assert node.signs_ready is False
    assert node._compute_twist().v == 0.0


def test_the_gate_can_be_switched_off_for_bring_up(node_factory):
    """wait_for_signs:=false is the escape hatch for driving without signs."""
    node, _module = node_factory(**{"~wait_for_signs": False})

    node._cb_lane(lane_error(0.0))

    assert node.signs_ready is True
    assert node._compute_twist().v > 0.0


def test_readiness_is_one_way(node_factory):
    """
    A dropped or repeated flag must not re-freeze a bot that is already driving.

    Freezing mid-crossing would be worse than carrying on: the manoeuvre is open
    loop, so stopping halfway leaves the bot across an intersection.
    """
    node, _module = node_factory()

    node._cb_signs_ready(FakeMessage(data=True))
    node._cb_signs_ready(FakeMessage(data=False))

    assert node.signs_ready is True


def test_the_gate_also_holds_during_a_crossing(node_factory):
    """
    The hold sits in front of the open-loop crossing branch too.

    It cannot happen at startup -- the mode is LANE_FOLLOWING -- but the gate is
    a claim about the wheels, not about one drive mode, and a crossing is the
    one branch that moves without any perception input at all.
    """
    node, module = node_factory()

    node.active_mode = module.DriveMode.CROSSING_INTERSECTION
    node.intersection_phase = module.IntersectionPhase.STRAIGHT_BEFORE_TURN

    assert node._compute_twist().v == 0.0


# ---------------------------------------------------------------------------
# The PID must not run while the bot is held
# ---------------------------------------------------------------------------

def test_the_pid_does_not_wind_up_while_waiting(node_factory):
    """
    Lane messages arrive for about a second before the wheels are released.

    Integrating a steering error over a second of standing still would hand the
    controller a wound-up integrator and a stale timestamp the moment it starts
    driving.
    """
    node, _module = node_factory()

    for _ in range(30):
        node._cb_lane(lane_error(0.5))

    assert node.integral == 0.0
    assert node.last_time is None
