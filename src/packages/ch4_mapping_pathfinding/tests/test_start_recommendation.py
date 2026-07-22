"""
Tests for recommending where to put the bot down for the gate run.

After mapping the bot is picked up, so the placement is a free choice -- and on
a timed run it is worth real seconds.
"""

import pytest

import city_map
import planner
from city_map import GraphMap


REAL_TRACK_GATES = {"A1__B1": 5, "B3__C4": 6, "A3__C1": 7}


@pytest.fixture
def costs():
    """Cost model using the turn times measured on the track."""
    return planner.TurnCosts(
        turn_durations={"LEFT": 2.7, "STRAIGHT": 3.2, "RIGHT": 0.9},
        stop_duration=3.0,
        edge_duration=4.0,
    )


def gate_edge_keys(order):
    return [key for gate in order
            for key, tag in REAL_TRACK_GATES.items() if tag == gate]


# ---------------------------------------------------------------------------
# Directed edges = legal placements
# ---------------------------------------------------------------------------

def test_every_street_is_a_placement_in_both_directions(city):
    edges = planner.all_directed_edges(city)

    assert len(edges) == 2 * len(city_map.all_edge_keys(city))


def test_each_placement_is_a_real_street(city):
    for from_node, from_port, to_node, to_port in planner.all_directed_edges(city):
        assert city[from_node][from_port] == (to_node, to_port)


# ---------------------------------------------------------------------------
# Recommendations
# ---------------------------------------------------------------------------

def test_no_gates_means_no_recommendation(city, costs):
    assert planner.recommend_start_edges(city, [], costs=costs) == []


def test_recommendations_are_sorted_cheapest_first(city, costs):
    options = planner.recommend_start_edges(
        city, gate_edge_keys([7, 5, 6]), costs=costs, limit=0
    )

    costs_in_order = [option["cost"] for option in options]

    assert costs_in_order == sorted(costs_in_order)


def test_the_best_placement_starts_on_the_first_gates_street(city, costs):
    """
    Starting on the first gate's street collects it for free, which is a whole
    crossing cheaper than having to drive to it.
    """
    for order in ([7, 5, 6], [5, 6, 7], [6, 7, 5]):
        best = planner.recommend_start_edges(
            city, gate_edge_keys(order), costs=costs
        )[0]

        assert best["first_gate_on_start_street"], order


def test_a_recommendation_is_drivable_and_reaches_every_gate(city, costs, gate_config):
    """The recommendation is worthless if the route it implies cannot be driven."""
    order = [7, 5, 6]
    targets = gate_edge_keys(order)

    best = planner.recommend_start_edges(city, targets, costs=costs)[0]

    graph = GraphMap(city, tuple(best["start_edge"]), gate_config=gate_config)
    for key, gate in REAL_TRACK_GATES.items():
        graph.record_gate(gate, edge_key_override=key)

    steps, _legs = planner.plan_gate_run(
        city, graph.current_state_tuple(), targets, costs=costs
    )

    seen = [graph.current_edge_key]
    for step in steps:
        assert graph.move(step.direction)["success"]
        seen.append(graph.current_edge_key)

    # Every gate street is driven, in the announced order.
    hit = [key for key in seen if key in targets]
    assert [key for i, key in enumerate(hit) if i == 0 or key != hit[i - 1]] == targets


def test_the_recommended_arg_is_what_the_launch_file_takes(city, costs):
    best = planner.recommend_start_edges(
        city, gate_edge_keys([7, 5, 6]), costs=costs
    )[0]

    parts = best["start_edge_arg"].split(",")

    assert len(parts) == 4
    assert [parts[0], int(parts[1]), parts[2], int(parts[3])] == best["start_edge"]


def test_the_limit_is_honoured(city, costs):
    assert len(planner.recommend_start_edges(
        city, gate_edge_keys([7, 5, 6]), costs=costs, limit=2
    )) == 2


def test_the_cost_includes_driving_the_street_it_starts_on(city, costs):
    """
    The run is timed from the moment it drives off, and it starts at an
    intersection exit -- so the first street counts even when it holds gate one.
    """
    targets = gate_edge_keys([7, 5, 6])
    best = planner.recommend_start_edges(city, targets, costs=costs)[0]

    steps, _legs = planner.plan_gate_run(
        city, (best["start_edge"][2], best["start_edge"][3]), targets, costs=costs
    )
    approach = costs.edge_durations.get(best["start_edge_key"], costs.edge_duration)

    assert best["cost"] == pytest.approx(approach + planner.total_cost(steps))


def test_which_way_round_the_start_street_matters(city, costs):
    """
    The real value of the recommendation on a track this small.

    The first gate's street always wins -- it has to be driven either way, and
    starting on it saves a crossing. What is *not* obvious is which direction
    to point the bot: both are on the gate street, but they leave the bot at
    opposite intersections, and the rest of the route follows from that.
    """
    targets = gate_edge_keys([5, 6])
    options = planner.recommend_start_edges(city, targets, costs=costs, limit=0)

    on_gate_street = [option for option in options
                      if option["start_edge_key"] == targets[0]]

    assert len(on_gate_street) == 2, "a street can be driven both ways"
    assert on_gate_street[0]["cost"] < on_gate_street[1]["cost"], (
        "the two directions should not be equally good -- if they were, there "
        "would be nothing to recommend"
    )
    assert options[0] is on_gate_street[0]


def test_measured_street_times_are_reflected_in_the_estimate(city):
    """A slow street must make every route that drives it look slower."""
    targets = gate_edge_keys([5])

    cheap = planner.recommend_start_edges(
        city, targets, costs=planner.TurnCosts(edge_durations={"A1__B1": 1.0})
    )[0]
    dear = planner.recommend_start_edges(
        city, targets, costs=planner.TurnCosts(edge_durations={"A1__B1": 30.0})
    )[0]

    assert dear["cost"] - cheap["cost"] == pytest.approx(29.0)


# ---------------------------------------------------------------------------
# Cost model wiring
# ---------------------------------------------------------------------------

def test_turn_costs_prefer_the_planner_section_over_crossing_timeouts():
    """
    switch_control's turn_durations are crossing *timeouts* -- an upper bound.
    Charging them would systematically overcharge every turn.
    """
    config = {
        "planner": {"turn_durations": {"RIGHT": 0.9}, "default_street_time": 5.0},
        "switch_control": {"timers": {"turn_durations": {"RIGHT": 1.4},
                                      "stop_duration": 3.0}},
    }

    costs = planner.TurnCosts.from_config(config)

    assert costs.turn_durations["RIGHT"] == 0.9
    assert costs.edge_duration == 5.0
    assert costs.stop_duration == 3.0


def test_turn_costs_fall_back_to_switch_control_without_a_planner_section():
    config = {"switch_control": {"timers": {"turn_durations": {"RIGHT": 1.4}}}}

    assert planner.TurnCosts.from_config(config).turn_durations["RIGHT"] == 1.4


def test_measurements_override_the_config(city):
    config = {"planner": {"turn_durations": {"RIGHT": 0.9}}}

    costs = planner.TurnCosts.from_config(
        config,
        edge_durations={"A1__B1": 7.5},
        turn_durations={"RIGHT": 1.1},
    )

    assert costs.turn_durations["RIGHT"] == 1.1
    assert costs.edge_durations["A1__B1"] == 7.5
    assert costs.cost("RIGHT", "A1__B1") == pytest.approx(1.1 + 7.5 + 3.0)
