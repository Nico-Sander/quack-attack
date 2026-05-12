from enum import Enum

### --- Detect Intersection ---
class IntersectionState(Enum):
    NO_INTERSECTION = 0
    APPROACHING_INTERSECTION = 1
    AT_INTERSECTION = 2
    
### --- Switch Control ---
# External Commands: What the motor controllers should do
class ControlType(Enum):
    LANE_FOLLOWING = 1
    FIND_STOP_LINE = 2
    STOP = 3
    DRIVE_INTERSECTION = 4

# Internal States: The phases of the brains decision making
class State(Enum):
    LANE_FOLLOWING = 1
    APPROACHING = 2
    STOPPED = 3
    CROSSING = 4
    CLEARING = 5

# 
class TurnDirection(Enum):
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3

class IntersectionPhase(Enum):
    STRAIGHT_BEFORE_TURN = 1
    INITIAL_TURNING = 2