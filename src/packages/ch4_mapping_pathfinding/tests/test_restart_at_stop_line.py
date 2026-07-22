"""
Starting the gate run from where mapping stopped.

The bot ends mapping stopped at a red line, which is the *end* of a street. A
gate on that street is behind it: it was driven during mapping, not during the
timed run. The graph position cannot tell that apart from having just entered
the same street, because a state means "just traversed the street attached to
entry_port" and that is true throughout the traversal and at the line.

Found on the robot: mapping ended on B3__C4 (gate 9), the announced order began
with 9, and the run counted it done before moving.
"""

import pytest

import planner
from city_map import GraphMap


REAL_TRACK_GATES = {"A1__B1": 7, "A2__C2": 6, "A3__C1": 8,
                    "A4__B2": 10, "B3__C4": 9}


@pytest.fixture
def costs():
    return planner.TurnCosts(
        turn_durations={"LEFT": 2.5, "STRAIGHT": 2.5, "RIGHT": 1.3},
        stop_duration=3.0,
        edge_duration=4.0,
    )


def key_for(gate):
    return next(key for key, tag in REAL_TRACK_GATES.items() if tag == gate)


# ---------------------------------------------------------------------------
# The planner has to be able to route back onto the street it is standing on
# ---------------------------------------------------------------------------

def test_without_require_move_the_current_street_is_already_reached(city, costs):
    """The old behaviour, kept explicit because everything else relies on it."""
    steps = planner.plan_to_edge(city, ("C", 4), "B3__C4", costs=costs)

    assert steps == []


def test_require_move_forces_the_street_to_be_driven_again(city, costs):
    """The bot is at the far end, so the street has to be re-driven."""
    steps = planner.plan_to_edge(city, ("C", 4), "B3__C4", costs=costs,
                                 require_move=True)

    assert steps, "a route must be planned, not 'already there'"
    assert steps[-1].edge_key == "B3__C4", "the route must end on that street"


def test_the_re_drive_route_is_actually_drivable(city, costs, gate_config):
    """Planned turns must be executable on the graph, not merely computed."""
    graph = GraphMap(city, ("B", 3, "C", 4), gate_config=gate_config)
    steps = planner.plan_to_edge(city, graph.current_state_tuple(), "B3__C4",
                                 costs=costs, require_move=True)

    for step in steps:
        assert graph.move(step.direction)["success"], step

    assert graph.current_edge_key == "B3__C4"


def test_require_move_never_returns_an_empty_route(city, costs):
    """Every state, every street: a forced move is always at least one step."""
    for node in city:
        for port in city[node]:
            for target in REAL_TRACK_GATES:
                steps = planner.plan_to_edge(city, (node, port), target,
                                             costs=costs, require_move=True)
                assert steps, (node, port, target)
                assert steps[-1].edge_key == target


def test_require_move_leaves_other_targets_unchanged(city, costs):
    """It must only affect the street the bot is standing on."""
    plain = planner.plan_to_edge(city, ("C", 4), "A1__B1", costs=costs)
    forced = planner.plan_to_edge(city, ("C", 4), "A1__B1", costs=costs,
                                  require_move=True)

    assert plain == forced


# ---------------------------------------------------------------------------
# The run as a whole
# ---------------------------------------------------------------------------

def test_the_bug_from_the_track(city, costs, gate_config):
    """
    Mapping ended at the red line of B3__C4, which carries gate 9, and the
    announced order was 9,6,7. Gate 9 must be driven, not assumed.
    """
    graph = GraphMap(city, ("B", 3, "C", 4), gate_config=gate_config)
    for key, gate in REAL_TRACK_GATES.items():
        graph.record_gate(gate, edge_key_override=key)

    order = [9, 6, 7]
    targets = [key_for(gate) for gate in order]

    steps, legs = planner.plan_gate_run(
        city, graph.current_state_tuple(), targets, costs=costs,
        start_street_driven=True,
    )

    assert legs[0]["steps"] > 0, "gate 9 must be driven to, not counted for free"

    # Drive it and confirm each gate street is traversed, in order.
    driven = []
    for step in steps:
        assert graph.move(step.direction)["success"], step
        driven.append(graph.current_edge_key)

    hit = [key for key in driven if key in targets]
    deduped = [key for i, key in enumerate(hit) if i == 0 or key != hit[i - 1]]

    assert deduped == targets


def test_starting_from_an_exit_still_counts_the_street_it_is_on(city, costs,
                                                                gate_config):
    """
    The other half of the same rule.

    Placed at an intersection exit, the whole street is ahead, so a gate on it
    genuinely will be driven and must count -- that is what makes the
    recommended placement worth taking.
    """
    graph = GraphMap(city, ("B", 3, "C", 4), gate_config=gate_config)
    for key, gate in REAL_TRACK_GATES.items():
        graph.record_gate(gate, edge_key_override=key)

    steps, legs = planner.plan_gate_run(
        city, graph.current_state_tuple(), [key_for(9), key_for(6)],
        costs=costs, start_street_driven=False,
    )

    assert legs[0]["steps"] == 0, "the gate ahead of the bot is collected free"


def test_re_driving_costs_more_than_being_placed_on_it(city, costs, gate_config):
    """Which is exactly why repositioning is worth doing."""
    graph = GraphMap(city, ("B", 3, "C", 4), gate_config=gate_config)
    for key, gate in REAL_TRACK_GATES.items():
        graph.record_gate(gate, edge_key_override=key)

    targets = [key_for(9), key_for(6), key_for(7)]
    state = graph.current_state_tuple()

    from_line, _ = planner.plan_gate_run(city, state, targets, costs=costs,
                                         start_street_driven=True)
    from_exit, _ = planner.plan_gate_run(city, state, targets, costs=costs,
                                         start_street_driven=False)

    assert planner.total_cost(from_line) > planner.total_cost(from_exit)


def test_only_the_first_leg_is_forced(city, costs, gate_config):
    """
    Later legs begin where the previous one ended -- a street the bot really
    has just driven -- so forcing them would add a pointless lap.
    """
    graph = GraphMap(city, ("B", 3, "C", 4), gate_config=gate_config)
    for key, gate in REAL_TRACK_GATES.items():
        graph.record_gate(gate, edge_key_override=key)

    targets = [key_for(9), key_for(9)]

    steps, legs = planner.plan_gate_run(
        city, graph.current_state_tuple(), targets, costs=costs,
        start_street_driven=True,
    )

    assert legs[0]["steps"] > 0
    assert legs[1]["steps"] == 0
