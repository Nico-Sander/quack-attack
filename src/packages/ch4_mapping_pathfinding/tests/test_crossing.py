"""
Tests for the crossing exit decision.

The point of closing this loop is that a crossing should end because the bot
can see a lane again, not because a timer expired. Timing out is the fallback,
and a run where every crossing times out has quietly gone back to being open
loop.
"""

import pytest

import crossing


TURN_DURATION = 2.5
MIN_BLIND = 0.8


def exit_now(time_in_state, white, yellow, **kwargs):
    return crossing.should_exit_crossing(
        time_in_state, TURN_DURATION, white, yellow,
        min_blind_duration=MIN_BLIND, **kwargs
    )


# ---------------------------------------------------------------------------
# The blind period
# ---------------------------------------------------------------------------

def test_does_not_exit_during_the_blind_period():
    """
    At the start of a turn the markings of the road being left are still in
    view. Trusting them would end the crossing before it began.
    """
    assert not exit_now(0.0, True, True)
    assert not exit_now(0.5, True, True)
    assert not exit_now(MIN_BLIND, True, True)


def test_exits_once_the_lane_is_back_after_the_blind_period():
    assert exit_now(MIN_BLIND + 0.1, True, True)


def test_stays_in_the_crossing_while_no_lane_is_visible():
    assert not exit_now(1.5, False, False)
    assert not exit_now(2.0, False, False)


# ---------------------------------------------------------------------------
# Timeout
# ---------------------------------------------------------------------------

def test_times_out_when_the_lane_is_never_found():
    assert exit_now(TURN_DURATION, False, False)
    assert exit_now(TURN_DURATION + 1.0, False, False)


def test_timeout_wins_even_inside_the_blind_period():
    """A turn_duration shorter than the blind time must not deadlock."""
    assert crossing.should_exit_crossing(
        0.5, 0.4, False, False, min_blind_duration=MIN_BLIND
    )


# ---------------------------------------------------------------------------
# Confidence
# ---------------------------------------------------------------------------

def test_both_mode_needs_both_lines():
    assert not exit_now(1.5, True, False)
    assert not exit_now(1.5, False, True)
    assert exit_now(1.5, True, True)


def test_yellow_only_mode_needs_just_the_centre_line():
    assert exit_now(1.5, False, True, mode=crossing.CONFIDENCE_YELLOW_ONLY)
    assert not exit_now(1.5, True, False, mode=crossing.CONFIDENCE_YELLOW_ONLY)


def test_unknown_mode_falls_back_to_the_strict_rule():
    """An unrecognised setting must not silently become the lax option."""
    assert not crossing.lane_reacquired(True, False, mode="nonsense")
    assert crossing.lane_reacquired(True, True, mode="nonsense")


# ---------------------------------------------------------------------------
# Reason reporting
# ---------------------------------------------------------------------------

def test_reason_is_lane_reacquired_when_the_lane_comes_back_early():
    assert crossing.exit_reason(
        1.5, TURN_DURATION, True, True, min_blind_duration=MIN_BLIND
    ) == "lane_reacquired"


def test_reason_is_timeout_when_the_duration_runs_out():
    assert crossing.exit_reason(
        TURN_DURATION, TURN_DURATION, False, False, min_blind_duration=MIN_BLIND
    ) == "timeout"


def test_reason_is_none_while_still_crossing():
    assert crossing.exit_reason(
        1.0, TURN_DURATION, False, False, min_blind_duration=MIN_BLIND
    ) is None


def test_reason_agrees_with_the_exit_decision():
    """Whenever it exits there is a reason, and vice versa."""
    for time_in_state in [0.0, 0.4, 0.8, 0.9, 1.5, 2.4, 2.5, 3.0]:
        for white in (False, True):
            for yellow in (False, True):
                exits = crossing.should_exit_crossing(
                    time_in_state, TURN_DURATION, white, yellow,
                    min_blind_duration=MIN_BLIND
                )
                reason = crossing.exit_reason(
                    time_in_state, TURN_DURATION, white, yellow,
                    min_blind_duration=MIN_BLIND
                )

                assert exits == (reason is not None), (
                    f"disagreement at t={time_in_state}, "
                    f"white={white}, yellow={yellow}"
                )


# ---------------------------------------------------------------------------
# The property that matters
# ---------------------------------------------------------------------------

def test_a_wide_turn_still_ends_on_the_lane_not_the_clock():
    """
    The whole point: a turn that comes out late still ends when the lane
    appears, at whatever moment that happens, rather than at a fixed time.
    """
    for lane_appears_at in [1.0, 1.4, 1.8, 2.2]:
        exited_at = next(
            t / 100.0
            for t in range(0, 400)
            if exit_now(t / 100.0,
                        *(True, True) if t / 100.0 >= lane_appears_at else (False, False))
        )

        assert exited_at == pytest.approx(lane_appears_at, abs=0.02), (
            f"expected to exit when the lane appeared at {lane_appears_at}s"
        )


def test_config_defaults_are_the_conservative_ones():
    assert crossing.CONFIDENCE_BOTH == "both"
    assert crossing.DEFAULT_MIN_BLIND_DURATION > 0.0
