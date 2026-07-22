"""
Custom enumerations for the Duckiebot ROS stack.
"""
from enum import Enum

class IntersectionState(Enum):
    """Output from the detect_intersection node."""
    NO_INTERSECTION = 0
    APPROACHING_INTERSECTION = 1
    AT_INTERSECTION = 2

class DriveMode(Enum):
    """The unified operational state commanded by switch_control."""
    LANE_FOLLOWING = 1
    APPROACHING_STOP_LINE = 2
    STOPPED = 3
    CROSSING_INTERSECTION = 4

class TurnDirection(Enum):
    """Directional commands for intersection crossing."""
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3

# Published on /plan/turn_command when the planner has no opinion (no route,
# localization lost, or mission finished). switch_control falls back to its
# sign-based choice when it sees this.
TURN_COMMAND_NONE = 0

# "Stop at the red line and stay there." Distinct from TURN_COMMAND_NONE, which
# means "I have no opinion, drive on your own": here the planner does have an
# opinion and it is to hold position. Used when mapping finishes, so the bot can
# be picked up and placed for the gate run instead of wandering off.
TURN_COMMAND_HALT = -1


class MissionPhase(Enum):
    """Which half of the challenge the mapping node is running."""
    MAPPING = 1             # explore every street to find all gates (untimed)
    GATE_RUN = 2            # drive the gates in the announced order (timed)
    DONE = 3                # mission complete, no further turn commands
    AWAITING_GATE_RUN = 4   # mapping finished, halted at a red line, waiting
                            # for the gate run to be started

class IntersectionPhase(Enum):
    """Internal phases for the control_wheels node during a crossing."""
    STRAIGHT_BEFORE_TURN = 1
    INITIAL_TURNING = 2