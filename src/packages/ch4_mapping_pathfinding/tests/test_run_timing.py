"""
Tests for the street and turn measurements taken during the mapping run.

The gate run is planned against these numbers, so a measurement filed under the
wrong street is worse than no measurement at all.
"""

import pytest

import run_timing
from run_timing import RunTimer, median


SEED_TURNS = {"LEFT": 2.7, "STRAIGHT": 3.2, "RIGHT": 0.9}


@pytest.fixture
def timer():
    return RunTimer(default_street_time=4.0, seed_turn_durations=SEED_TURNS)


# ---------------------------------------------------------------------------
# median
# ---------------------------------------------------------------------------

def test_median_of_an_odd_count():
    assert median([3.0, 1.0, 2.0]) == 2.0


def test_median_of_an_even_count_averages_the_middle():
    assert median([1.0, 2.0, 3.0, 4.0]) == 2.5


def test_median_of_nothing_is_none():
    assert median([]) is None


# ---------------------------------------------------------------------------
# Street timing
# ---------------------------------------------------------------------------

def test_a_street_is_timed_from_intersection_exit_to_the_red_line(timer):
    timer.start_street("A1__B1", 100.0)

    assert timer.finish_street(104.5) == ("A1__B1", 4.5)
    assert timer.street_time("A1__B1") == 4.5


def test_an_untimed_street_has_no_time(timer):
    assert timer.street_time("A1__B1") is None


def test_finishing_without_starting_records_nothing(timer):
    assert timer.finish_street(100.0) is None
    assert timer.street_samples == {}


def test_an_implausibly_short_street_is_rejected(timer):
    """A stop-line detection right after a crossing is not a street."""
    timer.start_street("A1__B1", 100.0)

    assert timer.finish_street(100.1) is None
    assert timer.street_time("A1__B1") is None
    assert timer.rejected == [("street", "A1__B1", pytest.approx(0.1))]


def test_repeated_measurements_are_reduced_with_the_median(timer):
    """Mapping drives some streets twice; one disturbed run must not win."""
    for start, end in ((0.0, 4.0), (10.0, 14.4), (20.0, 31.0)):
        timer.start_street("A1__B1", start)
        timer.finish_street(end)

    assert timer.street_time("A1__B1") == 4.4


def test_abandoning_drops_the_street_being_timed(timer):
    """Position lost: there is nothing to attribute the time to."""
    timer.start_street("A1__B1", 100.0)
    timer.abandon_street()

    assert timer.finish_street(104.0) is None
    assert timer.street_samples == {}


def test_a_new_street_replaces_the_one_being_timed(timer):
    timer.start_street("A1__B1", 100.0)
    timer.start_street("B3__C4", 105.0)

    assert timer.finish_street(109.0) == ("B3__C4", 4.0)
    assert timer.street_time("A1__B1") is None


# ---------------------------------------------------------------------------
# Turn timing
# ---------------------------------------------------------------------------

def test_a_turn_is_filed_under_the_direction_actually_executed(timer):
    """
    The direction is supplied when the crossing *ends*.

    switch_control publishes the drive mode before the turn direction, so at the
    start of a crossing the direction on the wire is still the previous one.
    """
    timer.start_turn(100.0)

    assert timer.finish_turn("RIGHT", 100.9) == ("RIGHT", pytest.approx(0.9))
    assert timer.turn_durations()["RIGHT"] == pytest.approx(0.9)


def test_an_unstarted_turn_records_nothing(timer):
    assert timer.finish_turn("LEFT", 100.0) is None


def test_an_implausibly_short_turn_is_rejected(timer):
    timer.start_turn(100.0)

    assert timer.finish_turn("LEFT", 100.05) is None
    assert timer.turn_durations()["LEFT"] == 2.7


# ---------------------------------------------------------------------------
# Feeding the cost model
# ---------------------------------------------------------------------------

def test_unmeasured_turns_keep_their_seeded_value(timer):
    """A five-street coverage route can easily never turn left."""
    timer.start_turn(0.0)
    timer.finish_turn("RIGHT", 1.4)

    durations = timer.turn_durations()

    assert durations["RIGHT"] == pytest.approx(1.4)
    assert durations["LEFT"] == 2.7
    assert durations["STRAIGHT"] == 3.2


def test_edge_durations_only_reports_measured_streets(timer):
    timer.start_street("A1__B1", 0.0)
    timer.finish_street(4.0)

    assert timer.edge_durations() == {"A1__B1": 4.0}


def test_unmeasured_lists_what_is_still_missing(timer):
    timer.start_street("A1__B1", 0.0)
    timer.finish_street(4.0)

    assert timer.unmeasured(["A1__B1", "B3__C4"]) == ["B3__C4"]


def test_a_rejected_sample_leaves_the_street_unmeasured(timer):
    """Otherwise mapping would "finish" on a measurement it threw away."""
    timer.start_street("A1__B1", 0.0)
    timer.finish_street(0.1)

    assert timer.unmeasured(["A1__B1"]) == ["A1__B1"]


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def test_the_state_dict_is_json_friendly(timer):
    timer.start_street("A1__B1", 0.0)
    timer.finish_street(4.0)
    timer.start_turn(0.0)
    timer.finish_turn("RIGHT", 0.9)

    state = timer.as_dict()

    assert state["street_times"] == {"A1__B1": 4.0}
    assert state["turn_times"] == {"RIGHT": 0.9}
    assert state["default_street_time"] == 4.0


def test_the_summary_names_streets_that_were_never_measured(timer):
    timer.start_street("A1__B1", 0.0)
    timer.finish_street(4.0)

    text = "\n".join(timer.summary_lines(["A1__B1", "B3__C4"]))

    assert "A1__B1" in text
    assert "not measured" in text


def test_the_module_constants_are_sane():
    assert run_timing.MIN_PLAUSIBLE_STREET_TIME > 0
    assert run_timing.MIN_PLAUSIBLE_TURN_TIME > 0
