"""
Full-mission simulation.

These tests run the decision sequence the mapping node follows -- explore every
street, then drive the announced gate order -- against the real test track,
without ROS. What they cannot cover is the ROS glue itself (topics, timers) and
whether the open-loop crossings physically succeed; both need the robot.
"""

import itertools

import pytest

import city_map
import planner
from city_map import GraphMap


REAL_TRACK_GATES = {"A1__B1": 5, "B3__C4": 6, "A3__C1": 7}


def run_mapping_phase(city, start_edge, placed_gates, gate_config, max_moves=200):
    """
    Drives the exploration strategy until every street has been covered,
    recording gates as their street is driven. Mirrors _plan_exploration().
    """
    graph = GraphMap(city, start_edge, gate_config=gate_config)

    def record_if_present():
        gate_id = placed_gates.get(graph.current_edge_key)
        if gate_id is not None:
            graph.record_gate(gate_id)

    record_if_present()
    moves = 0

    while not graph.is_fully_explored():
        steps = planner.plan_exploration_step(
            city, graph.current_state_tuple(), graph.unvisited_edge_keys()
        )

        assert steps, f"stuck with {sorted(graph.unvisited_edge_keys())} unexplored"

        for step in steps:
            assert graph.move(step.direction)["success"]
            record_if_present()

            moves += 1
            assert moves < max_moves, "exploration did not terminate"

    return graph, moves


def run_gate_phase(city, graph, announced_order, strict_order=False):
    """
    Plans and drives the announced gate order, replanning after every move the
    way the node does. Returns the order in which gate streets were driven.
    """
    remaining = list(announced_order)
    driven = []
    moves = 0

    while remaining:
        edge_keys = [graph.edge_key_for_gate(gate) for gate in remaining]
        assert all(edge_keys), f"gate(s) {remaining} were never mapped"

        steps, _legs = planner.plan_gate_run(
            city, graph.current_state_tuple(), edge_keys,
            strict_order=strict_order
        )

        if not steps:
            # Already standing on the next gate's street.
            driven.append(remaining.pop(0))
            continue

        # Execute only the first step, then replan -- this is what makes the
        # node self-healing after a corrected position.
        assert graph.move(steps[0].direction)["success"]

        if graph.current_edge_key == graph.edge_key_for_gate(remaining[0]):
            driven.append(remaining.pop(0))

        moves += 1
        assert moves < 200, "gate run did not terminate"

    return driven


# ---------------------------------------------------------------------------

def test_full_mission_on_the_real_track(city, gate_config):
    """Map the track, then drive an announced gate order end to end."""
    graph, moves = run_mapping_phase(
        city, ("A", 1, "B", 1), REAL_TRACK_GATES, gate_config
    )

    assert graph.is_fully_explored()
    assert {entry["gate_id"] for entry in graph.gate_edges()} == {5, 6, 7}
    assert moves <= 10, f"exploration took {moves} moves, expected a short route"

    announced = [7, 5, 6]
    assert run_gate_phase(city, graph, announced) == announced


def test_every_announced_order_is_drivable(city, gate_config):
    """The order is announced on site, so all of them have to work."""
    graph, _ = run_mapping_phase(
        city, ("A", 1, "B", 1), REAL_TRACK_GATES, gate_config
    )

    for announced in itertools.permutations([5, 6, 7]):
        # A fresh copy per run: the gate run must not depend on leftover state.
        fresh = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
        for key, gate_id in REAL_TRACK_GATES.items():
            fresh.record_gate(gate_id, edge_key_override=key)

        assert run_gate_phase(city, fresh, list(announced)) == list(announced)


@pytest.mark.parametrize("start_edge", [
    ("A", 1, "B", 1), ("B", 1, "A", 1), ("C", 4, "B", 3),
    ("A", 2, "C", 2), ("B", 2, "A", 4),
])
def test_mission_works_from_any_start_edge(city, gate_config, start_edge):
    """The sheet lets us pick the start pose, but it must not be load-bearing."""
    graph, _ = run_mapping_phase(city, start_edge, REAL_TRACK_GATES, gate_config)

    assert graph.is_fully_explored()
    assert run_gate_phase(city, graph, [6, 7, 5]) == [6, 7, 5]


def test_gate_run_may_start_on_a_different_edge(city, gate_config):
    """Phase 2 is allowed to start somewhere else than phase 1 ended."""
    graph, _ = run_mapping_phase(
        city, ("A", 1, "B", 1), REAL_TRACK_GATES, gate_config
    )

    graph.start_on_edge(("C", 1, "A", 3))

    assert run_gate_phase(city, graph, [5, 6, 7]) == [5, 6, 7]


def test_strict_order_still_completes_the_run(city, gate_config):
    graph, _ = run_mapping_phase(
        city, ("A", 1, "B", 1), REAL_TRACK_GATES, gate_config
    )

    announced = [5, 7, 6]
    assert run_gate_phase(city, graph, announced, strict_order=True) == announced


def test_unmapped_gate_is_detected_before_the_run(city, gate_config):
    """A gate that was never seen must fail loudly, not plan a bogus route."""
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
    graph.record_gate(5)

    assert graph.edge_key_for_gate(5) is not None
    assert graph.edge_key_for_gate(9) is None


def test_coverage_complete_does_not_imply_all_gates_found(city, gate_config):
    """
    Regression test for a bug found in the ROS simulation.

    Coverage completes the moment the last street is *entered*, but its gate is
    only seen while driving it. Anything that triggers on "every street driven"
    -- notably auto_start_gate_run -- must also check that the gates it needs
    have actually been recorded, or it plans against an incomplete map.
    """
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
    last_street_gate = ("A3__C1", 7)

    # Drive the exploration route, recording every gate except the one on the
    # street that completes coverage.
    while not graph.is_fully_explored():
        steps = planner.plan_exploration_step(
            city, graph.current_state_tuple(), graph.unvisited_edge_keys()
        )

        for step in steps:
            graph.move(step.direction)

            gate_id = REAL_TRACK_GATES.get(graph.current_edge_key)
            if gate_id is not None and graph.current_edge_key != last_street_gate[0]:
                graph.record_gate(gate_id)

    # The moment coverage completes, the final street's gate is still unknown.
    assert graph.is_fully_explored()
    assert graph.current_edge_key == last_street_gate[0]
    assert graph.edge_key_for_gate(last_street_gate[1]) is None

    # Only once it is seen is the announced order actually plannable.
    graph.record_gate(last_street_gate[1])
    assert graph.edge_key_for_gate(last_street_gate[1]) == last_street_gate[0]
    assert run_gate_phase(city, graph, [7, 5, 6]) == [7, 5, 6]


def test_mapping_covers_the_track_even_with_no_gates_placed(city, gate_config):
    """Coverage must not depend on finding anything."""
    graph, _ = run_mapping_phase(city, ("A", 1, "B", 1), {}, gate_config)

    assert graph.is_fully_explored()
    assert graph.gate_edges() == []
