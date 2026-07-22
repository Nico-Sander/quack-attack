"""
Tests for the guards in front of the map's gate table.

A gate is written to a street once and never overwritten, so everything here is
about making sure the *first* write is the right one.
"""

import json
import os

import city_map
import gate_detection
from custom_enums import DriveMode
from gate_detection import GateConfirmer

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "config.json"
)


def detection(tag_id, area, accepted=None):
    if accepted is None:
        accepted = gate_detection.is_confident(area)

    return {"tag_id": tag_id, "area": float(area), "accepted": accepted}


# ---------------------------------------------------------------------------
# When gate mapping is allowed at all
# ---------------------------------------------------------------------------

def test_gates_are_mapped_while_driving():
    assert gate_detection.mapping_allowed(DriveMode.LANE_FOLLOWING)


def test_gates_are_not_mapped_while_stopped_at_an_intersection():
    """Stopped at the line the camera looks across the intersection."""
    assert not gate_detection.mapping_allowed(DriveMode.STOPPED)


def test_gates_are_not_mapped_while_crossing():
    assert not gate_detection.mapping_allowed(DriveMode.CROSSING_INTERSECTION)


def test_approaching_is_allowed_by_default():
    """The bot is still on its own street, so gates at its far end count."""
    assert gate_detection.mapping_allowed(DriveMode.APPROACHING_STOP_LINE)


def test_approaching_can_be_switched_off():
    assert not gate_detection.mapping_allowed(
        DriveMode.APPROACHING_STOP_LINE, allow_approaching=False
    )


def test_switching_off_approaching_does_not_affect_normal_driving():
    assert gate_detection.mapping_allowed(
        DriveMode.LANE_FOLLOWING, allow_approaching=False
    )


# ---------------------------------------------------------------------------
# Size threshold
# ---------------------------------------------------------------------------

def test_a_close_tag_is_confident_and_a_distant_one_is_not():
    assert gate_detection.is_confident(5000.0, min_area=2000.0)
    assert not gate_detection.is_confident(400.0, min_area=2000.0)


def test_the_threshold_is_inclusive():
    assert gate_detection.is_confident(2000.0, min_area=2000.0)


def test_the_calibrated_threshold_lives_in_the_config():
    """
    detect_signs reads this at startup, so a missing or nonsense value would
    silently drop the whole node back to the module fallback.
    """
    with open(CONFIG_PATH) as handle:
        min_area = json.load(handle)["detect_signs"]["gate_min_area"]

    assert isinstance(min_area, (int, float))
    assert min_area > 0


# ---------------------------------------------------------------------------
# Picking a candidate out of one frame
# ---------------------------------------------------------------------------

def test_the_closest_accepted_gate_wins():
    frame = [detection(7, 9000.0), detection(9, 3000.0)]

    assert gate_detection.largest_accepted(frame)["tag_id"] == 7


def test_rejected_detections_are_never_candidates():
    """The case from the track: a gate across the intersection, seen small."""
    frame = [detection(9, 300.0)]

    assert gate_detection.largest_accepted(frame) is None


def test_a_big_far_gate_does_not_beat_a_smaller_near_one():
    """Only accepted detections compete, regardless of area ordering."""
    frame = [detection(9, 90000.0, accepted=False), detection(7, 3000.0)]

    assert gate_detection.largest_accepted(frame)["tag_id"] == 7


def test_intersection_signs_are_not_gates():
    frame = [detection(2, 50000.0), detection(7, 3000.0)]
    picked = gate_detection.largest_accepted(
        frame, known_sign_ids=city_map.KNOWN_SIGN_IDS
    )

    assert picked["tag_id"] == 7


def test_an_empty_frame_yields_no_candidate():
    assert gate_detection.largest_accepted([]) is None


# ---------------------------------------------------------------------------
# Gate-run progress, as shown to a person
# ---------------------------------------------------------------------------

def test_nothing_seen_means_nothing_done():
    done, next_gate, pending = gate_detection.gate_run_progress([7, 5, 6], [])

    assert done == []
    assert next_gate == 7
    assert pending == [7, 5, 6]


def test_a_seen_gate_is_done_and_the_next_moves_on():
    done, next_gate, pending = gate_detection.gate_run_progress([7, 5, 6], [7])

    assert done == [7]
    assert next_gate == 5
    assert pending == [5, 6]


def test_everything_seen_leaves_no_next():
    done, next_gate, pending = gate_detection.gate_run_progress(
        [7, 5, 6], [7, 5, 6]
    )

    assert done == [7, 5, 6]
    assert next_gate is None
    assert pending == []


def test_results_stay_in_the_announced_order():
    """Sightings could arrive in any order; the display must not reorder."""
    done, _next, pending = gate_detection.gate_run_progress([7, 5, 6], [6, 7])

    assert done == [7, 6]
    assert pending == [5]


def test_gates_seen_that_were_not_announced_are_ignored():
    done, next_gate, _pending = gate_detection.gate_run_progress([7, 5], [9, 7])

    assert done == [7]
    assert next_gate == 5


def test_missing_inputs_are_tolerated():
    """/mapping/state from an older node, or before a run has started."""
    assert gate_detection.gate_run_progress(None, None) == ([], None, [])
    assert gate_detection.gate_run_progress([7], None) == ([], 7, [7])


# ---------------------------------------------------------------------------
# Confirmation streak
# ---------------------------------------------------------------------------

def test_one_sighting_is_not_enough():
    confirmer = GateConfirmer(confirm_frames=3)

    assert not confirmer.confirm("A1__B1", 7, 0.0)
    assert not confirmer.confirm("A1__B1", 7, 0.1)
    assert confirmer.confirm("A1__B1", 7, 0.2)


def test_a_single_frame_is_enough_when_asked_for():
    confirmer = GateConfirmer(confirm_frames=1)

    assert confirmer.confirm("A1__B1", 7, 0.0)


def test_the_streak_stays_confirmed_while_it_holds():
    """The write it guards is idempotent, so it need not fire exactly once."""
    confirmer = GateConfirmer(confirm_frames=2)
    confirmer.confirm("A1__B1", 7, 0.0)

    assert confirmer.confirm("A1__B1", 7, 0.1)
    assert confirmer.confirm("A1__B1", 7, 0.2)


def test_a_different_gate_restarts_the_count():
    confirmer = GateConfirmer(confirm_frames=2)
    confirmer.confirm("A1__B1", 7, 0.0)

    assert not confirmer.confirm("A1__B1", 9, 0.1)
    assert confirmer.confirm("A1__B1", 9, 0.2)


def test_a_different_street_restarts_the_count():
    """Sightings from before a crossing must not confirm a gate after it."""
    confirmer = GateConfirmer(confirm_frames=2)
    confirmer.confirm("A1__B1", 7, 0.0)

    assert not confirmer.confirm("B3__C4", 7, 0.1)


def test_a_gap_restarts_the_count():
    confirmer = GateConfirmer(confirm_frames=2, streak_timeout=0.5)
    confirmer.confirm("A1__B1", 7, 0.0)

    assert not confirmer.confirm("A1__B1", 7, 10.0)
    assert confirmer.confirm("A1__B1", 7, 10.1)


def test_reset_clears_a_part_built_streak():
    confirmer = GateConfirmer(confirm_frames=2)
    confirmer.confirm("A1__B1", 7, 0.0)
    confirmer.reset()

    assert not confirmer.confirm("A1__B1", 7, 0.1)


def test_no_current_street_confirms_nothing():
    confirmer = GateConfirmer(confirm_frames=1)

    assert not confirmer.confirm(None, 7, 0.0)
