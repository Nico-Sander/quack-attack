"""
Enumerations shared by the challenge 1 nodes.

Only the drive mode is shared: the perception nodes publish plain booleans, so
there is nothing else the nodes need to agree on beyond the state machine.
"""

from enum import Enum


class DriveMode(Enum):
    """
    The operational state, decided by switch_control and obeyed by everyone.

    AT_STOP_LINE is the only state in which the robot stands still. CROSSING
    drives blind over the stop line for a fixed time, CROSSING_CLEARING lane
    follows again while ignoring the red line until it has left the image, so
    the same line cannot trigger a second stop.
    """

    LANE_FOLLOWING = 1
    AT_STOP_LINE = 2
    CROSSING = 3
    CROSSING_CLEARING = 4
