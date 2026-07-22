#!/usr/bin/env python3

"""
When to stop crossing an intersection.

The crossing manoeuvre is open loop: a fixed arc for a fixed time. That makes
it only as accurate as the pose the bot happened to have at the stop line, and
in practice it is never square at the same distance twice. Tuning the arc to
compensate is chasing a moving target.

So the arc is not what ends the crossing. The bot leaves the intersection as
soon as it can see lane markings again, and the configured duration is demoted
to a timeout for when it never does. A turn that comes out slightly wide still
ends in the right place, because the exit condition is "I am in a lane", not
"2.5 seconds have passed".

Two guards:
  - a minimum blind time, because the markings of the road being *left* are
    still in view for the first moment of the turn
  - a confidence mode, so a single stray line does not count as a lane

ROS-free so the decision can be unit tested.
"""


# Both lines is the safe default: mid-turn the camera often catches one line of
# some other lane, and a single line is not enough to conclude the bot is back.
CONFIDENCE_BOTH = "both"
CONFIDENCE_YELLOW_ONLY = "yellow_only"

DEFAULT_MIN_BLIND_DURATION = 0.8


def lane_reacquired(white_detected, yellow_detected, mode=CONFIDENCE_BOTH):
    """Whether the markings in view are convincing enough to call it a lane."""
    if mode == CONFIDENCE_YELLOW_ONLY:
        return bool(yellow_detected)

    return bool(white_detected and yellow_detected)


def should_exit_crossing(time_in_state, turn_duration, white_detected,
                         yellow_detected, min_blind_duration=DEFAULT_MIN_BLIND_DURATION,
                         mode=CONFIDENCE_BOTH):
    """
    True when the crossing manoeuvre should hand back to lane following.

    Either the lane has been re-acquired after the blind period, or the
    configured duration has run out and the crossing is given up as timed out.
    """
    if time_in_state >= turn_duration:
        return True

    if time_in_state <= min_blind_duration:
        return False

    return lane_reacquired(white_detected, yellow_detected, mode=mode)


def exit_reason(time_in_state, turn_duration, white_detected, yellow_detected,
                min_blind_duration=DEFAULT_MIN_BLIND_DURATION,
                mode=CONFIDENCE_BOTH):
    """
    Why the crossing ended: "lane_reacquired", "timeout", or None.

    Worth logging -- a run that always times out means the exit detection is
    not working and the bot is back to being open loop without anyone noticing.
    """
    if (time_in_state > min_blind_duration
            and lane_reacquired(white_detected, yellow_detected, mode=mode)
            and time_in_state < turn_duration):
        return "lane_reacquired"

    if time_in_state >= turn_duration:
        return "timeout"

    return None
