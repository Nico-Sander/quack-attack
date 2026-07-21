"""
Tests for the route planner.

The important ones simulate a full drive: the planner produces turns, a
GraphMap executes them, and the resulting position is checked against what was
planned. That is the off-robot equivalent of driving the track.
"""

import itertools

import pytest

import city_map
import planner
from city_map import GraphMap
from conftest import grid_city


ALL_START_STATES = [("A", 1), ("A", 2), ("A", 3), ("A", 4),
                    ("B", 1), ("B", 2), ("B", 3),
                    ("C", 1), ("C", 2), ("C", 4)]


def start_edge_for(city, state):
    """The directed edge that puts a GraphMap into `state`."""
    node, port = state
    neighbour, neighbour_port = city[node][port]
    return (neighbour, neighbour_port, node, port)


def drive(city, start_state, steps):
    """
    Executes a plan on a fresh GraphMap and returns it.

    Every move must succeed -- a failure means the planner proposed a turn the
    city does not allow.
    """
    graph = GraphMap(city, start_edge_for(city, start_state))

    for step in steps:
        assert graph.current_state_tuple() == step.from_state, (
            f"plan desynchronised: expected {step.from_state}, "
            f"bot is at {graph.current_state_tuple()}"
        )

        result = graph.move(step.direction)

        assert result["success"], (
            f"planner proposed an impossible turn: {step.direction} "
            f"at {step.from_state} (available: {result.get('available')})"
        )
        assert result["new_edge_key"] == step.edge_key

    return graph


# ---------------------------------------------------------------------------
# State space
# ---------------------------------------------------------------------------

def test_successors_never_include_a_u_turn(city):
    for state in ALL_START_STATES:
        arrived_on = planner.traversed_edge_key(city, state)

        for _direction, next_state, key in planner.successors(city, state):
            # Driving back down the street just arrived on is the definition of
            # a U-turn here, and it must never be offered.
            assert key != arrived_on
            # Nor may the move land the bot back where it came from.
            assert next_state != city[state[0]][state[1]]


def test_successors_only_use_existing_ports(city):
    for state in ALL_START_STATES:
        node, _ = state
        for direction, _next_state, _key in planner.successors(city, state):
            port = city_map.exit_port(state[1], direction)
            assert port in city[node]


def test_every_state_has_somewhere_to_go(city):
    """No dead ends -- the bot can never be stuck without a U-turn."""
    for state in ALL_START_STATES:
        assert planner.successors(city, state)


def test_edge_goal_states_are_the_two_driving_directions(city):
    assert planner.edge_goal_states(city, "A1__B1") == {("A", 1), ("B", 1)}
    assert planner.edge_goal_states(city, "B3__C4") == {("B", 3), ("C", 4)}


def test_edge_goal_states_rejects_unknown_street(city):
    with pytest.raises(planner.PlanningError):
        planner.edge_goal_states(city, "A1__Z9")


# ---------------------------------------------------------------------------
# Costs
# ---------------------------------------------------------------------------

def test_costs_come_from_the_shared_config():
    costs = planner.TurnCosts.from_config({
        "switch_control": {
            "timers": {
                "stop_duration": 3.0,
                "turn_durations": {"LEFT": 2.4, "RIGHT": 1.2, "STRAIGHT": 2.0},
            }
        }
    })

    assert costs.stop_duration == 3.0
    assert costs.turn_durations["LEFT"] == 2.4


def test_right_turns_are_cheaper_than_left_turns():
    costs = planner.TurnCosts()

    assert costs.cost("RIGHT", "A1__B1") < costs.cost("STRAIGHT", "A1__B1")
    assert costs.cost("STRAIGHT", "A1__B1") < costs.cost("LEFT", "A1__B1")


def test_per_street_durations_override_the_default():
    costs = planner.TurnCosts(edge_duration=4.0, edge_durations={"A1__B1": 10.0})

    assert costs.cost("STRAIGHT", "A1__B1") > costs.cost("STRAIGHT", "A2__C2")


# ---------------------------------------------------------------------------
# Planning to a single street
# ---------------------------------------------------------------------------

def test_planning_to_the_street_already_being_driven_is_a_no_op(city):
    # Arriving at B via port 1 means A1__B1 was just driven.
    assert planner.plan_to_edge(city, ("B", 1), "A1__B1") == []


def test_every_street_is_reachable_from_every_state(city):
    """Nothing on this track is unreachable, whatever the start pose."""
    for state in ALL_START_STATES:
        for key in city_map.all_edge_keys(city):
            steps = planner.plan_to_edge(city, state, key)
            graph = drive(city, state, steps)
            assert graph.current_edge_key == key


def test_planner_prefers_the_cheaper_of_two_routes(city):
    steps = planner.plan_to_edge(city, ("B", 1), "B3__C4")

    # Straight out of B port 3 reaches it in one move; anything else is longer.
    assert planner.turn_sequence(steps) == ["STRAIGHT"]
    assert planner.route_edge_keys(steps) == ["B3__C4"]


def test_forbidden_streets_are_avoided(city):
    direct = planner.plan_to_edge(city, ("B", 1), "B3__C4")
    detour = planner.plan_to_edge(city, ("B", 1), "B3__C4",
                                  forbidden_edge_keys={"A1__B1"})

    assert "A1__B1" not in planner.route_edge_keys(detour)
    assert planner.total_cost(detour) >= planner.total_cost(direct)


def test_unreachable_goal_raises():
    # Two disconnected pairs of nodes.
    split = {
        "A": {1: ("B", 1)}, "B": {1: ("A", 1)},
        "X": {1: ("Y", 1)}, "Y": {1: ("X", 1)},
    }
    city_map.validate_city(split)

    with pytest.raises(planner.PlanningError):
        planner.plan_to_edge(split, ("B", 1), "X1__Y1")


# ---------------------------------------------------------------------------
# Gate runs -- the timed challenge
# ---------------------------------------------------------------------------

def test_gate_run_visits_every_gate_in_order(city):
    """The acceptance test from the plan, over every gate ordering."""
    gate_streets = ["A1__B1", "A2__C2", "B3__C4"]

    for order in itertools.permutations(gate_streets):
        for state in ALL_START_STATES:
            steps, legs = planner.plan_gate_run(city, state, list(order))
            graph = drive(city, state, steps)

            # Each leg must end on its gate street, in the requested order.
            visited_at = []
            position = 0
            for leg in legs:
                position += leg["steps"]
                visited_at.append(
                    steps[position - 1].edge_key if position else None
                )

            expected = [
                key if key != planner.traversed_edge_key(city, state) else None
                for key in order
            ]
            for actual, want in zip(visited_at, expected):
                if want is not None:
                    assert actual == want

            assert graph.current_edge_key == order[-1]


def test_gate_run_from_a_gate_street_skips_the_first_leg(city):
    # Starting already on A1__B1 with that as gate 1.
    steps, legs = planner.plan_gate_run(city, ("B", 1), ["A1__B1", "B3__C4"])

    assert legs[0]["steps"] == 0
    assert legs[0]["cost"] == 0
    assert steps  # the second leg still has work to do


def test_strict_order_avoids_driving_through_later_gates(city):
    gates = ["A4__B2", "A1__B1"]

    lenient, _ = planner.plan_gate_run(city, ("C", 1), gates, strict_order=False)
    strict, _ = planner.plan_gate_run(city, ("C", 1), gates, strict_order=True)

    # Both must still be legal, complete runs.
    for steps in (lenient, strict):
        graph = drive(city, ("C", 1), steps)
        assert graph.current_edge_key == "A1__B1"

    first_leg_length = len(planner.plan_to_edge(city, ("C", 1), "A4__B2"))
    assert "A1__B1" not in planner.route_edge_keys(strict[:first_leg_length])


def test_strict_order_falls_back_rather_than_failing():
    """
    A one-way ring, where reaching the second street means driving through the
    first. Avoiding a later gate is then impossible, and the planner must fall
    back to the direct route instead of refusing to plan the run.
    """
    ring = {
        "A": {1: ("B", 3), 3: ("D", 1)},
        "B": {1: ("C", 3), 3: ("A", 1)},
        "C": {1: ("D", 3), 3: ("B", 1)},
        "D": {1: ("A", 3), 3: ("C", 1)},
    }
    city_map.validate_city(ring)

    # Every node has only ports 1 and 3, so STRAIGHT is the only legal move and
    # the ring can be driven in exactly one direction from a given entry.
    # Starting on A1__B3, reaching C1__D3 forces driving B1__C3 on the way.
    steps, legs = planner.plan_gate_run(
        ring, ("B", 3), ["C1__D3", "B1__C3"], strict_order=True
    )

    assert len(legs) == 2
    assert "B1__C3" in planner.route_edge_keys(steps[:legs[0]["steps"]])

    graph = drive(ring, ("B", 3), steps)
    assert graph.current_edge_key == "B1__C3"


def test_gate_run_is_not_hard_coded_but_follows_the_given_order(city):
    """Reversing the requested order must reverse the route."""
    forward, _ = planner.plan_gate_run(city, ("B", 1), ["A2__C2", "A4__B2"])
    backward, _ = planner.plan_gate_run(city, ("B", 1), ["A4__B2", "A2__C2"])

    assert planner.route_edge_keys(forward)[-1] == "A4__B2"
    assert planner.route_edge_keys(backward)[-1] == "A2__C2"


# ---------------------------------------------------------------------------
# Exploration coverage -- the mapping phase
# ---------------------------------------------------------------------------

def test_greedy_exploration_covers_the_whole_track(city):
    """Repeatedly driving to the nearest unseen street must visit them all."""
    for state in ALL_START_STATES:
        graph = GraphMap(city, start_edge_for(city, state))

        for _ in range(100):
            steps = planner.plan_exploration_step(
                city, graph.current_state_tuple(), graph.unvisited_edge_keys()
            )

            if steps is None:
                break

            for step in steps:
                assert graph.move(step.direction)["success"]
        else:
            pytest.fail(f"exploration did not terminate from {state}")

        assert graph.is_fully_explored(), (
            f"missed {sorted(graph.unvisited_edge_keys())} starting at {state}"
        )


def test_exploration_returns_none_when_done(city):
    graph = GraphMap(city, ("A", 1, "B", 1))

    for key in graph.edges:
        graph.edges[key]["visited"] = True

    assert planner.plan_exploration_step(
        city, graph.current_state_tuple(), graph.unvisited_edge_keys()
    ) is None


def test_exploration_finds_every_gate(city, gate_config):
    """Coverage is the point: full coverage means no gate can be missed."""
    placed = {"A3__C1": 7, "B3__C4": 5, "A4__B2": 11}

    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    while True:
        steps = planner.plan_exploration_step(
            city, graph.current_state_tuple(), graph.unvisited_edge_keys()
        )

        if steps is None:
            break

        for step in steps:
            graph.move(step.direction)
            # A gate on the street just driven would be seen now.
            if graph.current_edge_key in placed:
                graph.record_gate(placed[graph.current_edge_key])

    found = {entry["edge_key"]: entry["gate_id"] for entry in graph.gate_edges()}
    assert found == placed


# ---------------------------------------------------------------------------
# Larger synthetic maps
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("rows,cols", [(2, 2), (3, 3), (4, 5)])
def test_planner_scales_to_grid_cities(rows, cols):
    grid = grid_city(rows, cols)
    start = ("AA", 3)  # arrived at the top-left node from below

    for key in sorted(city_map.all_edge_keys(grid)):
        steps = planner.plan_to_edge(grid, start, key)
        graph = drive(grid, start, steps)
        assert graph.current_edge_key == key


def test_exploration_covers_a_grid_city():
    grid = grid_city(3, 3)
    start_state = ("AA", 3)
    graph = GraphMap(grid, start_edge_for(grid, start_state))

    for _ in range(500):
        steps = planner.plan_exploration_step(
            grid, graph.current_state_tuple(), graph.unvisited_edge_keys()
        )

        if steps is None:
            break

        for step in steps:
            assert graph.move(step.direction)["success"]

    assert graph.is_fully_explored()


def test_gate_run_on_a_grid_city():
    grid = grid_city(4, 4)
    start_state = ("AA", 3)

    # Derived from the grid itself, so the test cannot disagree with either the
    # key format or the port-to-compass mapping.
    def street(node, port):
        return city_map.edge_key(node, port, *grid[node][port])

    gates = [
        street("AD", 3),   # AD -> south
        street("CA", 1),   # CA -> north
        street("BB", 4),   # BB -> east
    ]

    steps, legs = planner.plan_gate_run(grid, start_state, gates)
    graph = drive(grid, start_state, steps)

    assert len(legs) == 3
    assert graph.current_edge_key == gates[-1]
