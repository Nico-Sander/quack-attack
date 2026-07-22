"""
Enumerations shared by the challenge 2 nodes.

These are the vocabulary the nodes agree on: what perception saw, what the
state machine decided, and which way it decided to turn.
"""

from enum import Enum


class IntersectionState(Enum):
    """What detect_intersection makes of the stop line geometry."""

    NO_INTERSECTION = 0
    APPROACHING_INTERSECTION = 1
    AT_INTERSECTION = 2


class DriveMode(Enum):
    """
    The operational state, decided by switch_control and obeyed by everyone.

    STOPPED is the only state in which the robot stands still.
    CROSSING_INTERSECTION drives the turn manoeuvre, during which there are no
    lane markings to follow.
    """

    LANE_FOLLOWING = 1
    APPROACHING_STOP_LINE = 2
    STOPPED = 3
    CROSSING_INTERSECTION = 4


class TurnDirection(Enum):
    """Which way to go at the intersection."""

    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3


class IntersectionPhase(Enum):
    """
    The two phases control_wheels drives a crossing in.

    The robot first rolls straight far enough to get its wheels into the
    intersection, then turns. Starting the turn immediately would clip the
    corner of the lane it is leaving.
    """

    STRAIGHT_BEFORE_TURN = 1
    INITIAL_TURNING = 2
