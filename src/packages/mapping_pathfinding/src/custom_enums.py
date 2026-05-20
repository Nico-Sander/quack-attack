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

class IntersectionPhase(Enum):
    """Internal phases for the control_wheels node during a crossing."""
    STRAIGHT_BEFORE_TURN = 1
    INITIAL_TURNING = 2