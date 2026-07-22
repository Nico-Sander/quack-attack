"""Tests for the city representation and the live map state."""

import json

import pytest

import city_map
from city_map import CityMapError, GraphMap, edge_key, exit_port


# ---------------------------------------------------------------------------
# Port geometry
# ---------------------------------------------------------------------------

def test_port_1_and_3_are_opposite():
    assert exit_port(1, "STRAIGHT") == 3
    assert exit_port(3, "STRAIGHT") == 1


def test_port_2_is_right_of_port_1_for_an_entering_bot():
    """
    The challenge sheet's "2 ist immer rechts von 1", read from the driver's
    seat. Confirmed on the track -- see the two cases below.
    """
    assert exit_port(1, "RIGHT") == 2
    assert exit_port(1, "LEFT") == 4


def test_handedness_matches_the_track(city):
    """
    Regression test for a left/right swap that only showed up on the robot.

    Two observations from the real track, which the geometry must reproduce:
      - entering B through port 1 and turning right comes out at B2 (-> A4)
      - entering A through port 4 and turning right comes out at A1 (-> B1)
    """
    graph = GraphMap(city, ("A", 1, "B", 1))

    assert graph.current_state_tuple() == ("B", 1)
    assert graph.is_direction_available("RIGHT")
    assert graph.direction_to_exit_port("RIGHT") == 2

    first = graph.move("RIGHT")
    assert first["success"]
    assert first["exit_port"] == 2
    assert graph.current_state_tuple() == ("A", 4)

    second = graph.move("RIGHT")
    assert second["success"]
    assert second["exit_port"] == 1
    assert graph.current_edge_key == "A1__B1"


def test_no_direction_ever_leads_back_out_the_entry_port():
    """This is what structurally forbids U-turns."""
    for port in (1, 2, 3, 4):
        for direction in city_map.DIRECTIONS:
            assert exit_port(port, direction) != port


def test_left_and_right_are_inverse():
    for port in (1, 2, 3, 4):
        assert exit_port(exit_port(port, "LEFT"), "RIGHT") == port


def test_unknown_direction_and_port_are_rejected():
    with pytest.raises(CityMapError):
        exit_port(1, "BACKWARDS")

    with pytest.raises(CityMapError):
        exit_port(9, "LEFT")


def test_edge_key_is_direction_independent():
    assert edge_key("A", 1, "B", 1) == edge_key("B", 1, "A", 1)
    assert edge_key("A", 1, "B", 1) == "A1__B1"


def test_edge_key_separates_parallel_streets():
    # A and B are joined twice on the real track; the keys must not collide.
    assert edge_key("A", 1, "B", 1) != edge_key("A", 4, "B", 2)


# ---------------------------------------------------------------------------
# Loading and validation
# ---------------------------------------------------------------------------

def test_real_city_matches_the_challenge_sheet(city):
    assert set(city) == {"A", "B", "C"}
    assert city["A"] == {1: ("B", 1), 2: ("C", 2), 3: ("C", 1), 4: ("B", 2)}
    assert city["B"] == {1: ("A", 1), 2: ("A", 4), 3: ("C", 4)}
    assert city["C"] == {1: ("A", 3), 2: ("A", 2), 4: ("B", 3)}


def test_real_city_has_five_streets(city):
    assert city_map.all_edge_keys(city) == {
        "A1__B1", "A2__C2", "A3__C1", "A4__B2", "B3__C4"
    }


def test_real_city_ports_are_ints_not_json_strings(city):
    for ports in city.values():
        assert all(isinstance(port, int) for port in ports)


def test_validate_rejects_asymmetric_connections():
    with pytest.raises(CityMapError, match="Asymmetric"):
        city_map.validate_city({
            "A": {1: ("B", 1)},
            "B": {2: ("A", 1)},  # A1 points at B1, but B has no port 1
        })


def test_validate_rejects_unknown_neighbour():
    with pytest.raises(CityMapError, match="unknown node"):
        city_map.validate_city({"A": {1: ("Z", 1)}})


def test_validate_rejects_out_of_range_port():
    with pytest.raises(CityMapError, match="expected one of 1-4"):
        city_map.validate_city({"A": {7: ("B", 1)}, "B": {1: ("A", 7)}})


def test_parse_city_converts_json_string_keys():
    parsed = city_map.parse_city({
        "A": {"1": ["B", 1]},
        "B": {"1": ["A", 1]},
    })

    assert parsed == {"A": {1: ("B", 1)}, "B": {1: ("A", 1)}}


def test_gate_config_covers_the_challenge_id_range(gate_config):
    assert set(gate_config) == set(range(5, 14))
    assert all("colour" in entry and "hex" in entry
               for entry in gate_config.values())


def test_gate_colours_are_distinct(gate_config):
    colours = [entry["colour"] for entry in gate_config.values()]
    assert len(set(colours)) == len(colours)


def test_intersection_sign_ids_are_not_gates(gate_config):
    assert not city_map.KNOWN_SIGN_IDS & set(gate_config)


# ---------------------------------------------------------------------------
# GraphMap
# ---------------------------------------------------------------------------

def test_start_edge_sets_position_and_marks_visited(city):
    graph = GraphMap(city, ("A", 1, "B", 1))

    assert graph.current_edge_key == "A1__B1"
    assert graph.current_node() == "B"
    assert graph.entry_port() == 1
    assert graph.current_state_tuple() == ("B", 1)
    assert graph.edges["A1__B1"]["visited"]


def test_start_edge_must_exist(city):
    with pytest.raises(CityMapError, match="does not exist"):
        GraphMap(city, ("A", 1, "C", 4))


def test_start_edge_must_be_oriented_consistently(city):
    # A1__B1 is a real street, but B is reached via port 1, not port 3.
    with pytest.raises(CityMapError):
        GraphMap(city, ("A", 1, "B", 3))


def test_available_directions_match_the_graph(city):
    graph = GraphMap(city, ("A", 1, "B", 1))

    # Arriving at B through port 1: B has ports 1, 2, 3.
    # RIGHT -> 2 (exists), STRAIGHT -> 3 (exists), LEFT -> 4 (absent).
    assert graph.available_directions() == ["STRAIGHT", "RIGHT"]


def test_move_advances_position(city):
    graph = GraphMap(city, ("A", 1, "B", 1))
    result = graph.move("STRAIGHT")

    assert result["success"]
    assert result["new_edge"] == ["B", 3, "C", 4]
    assert graph.current_edge_key == "B3__C4"
    assert graph.current_state_tuple() == ("C", 4)


def test_move_into_a_missing_street_fails_without_moving(city):
    graph = GraphMap(city, ("A", 1, "B", 1))
    before = graph.current_edge

    result = graph.move("LEFT")  # LEFT_OF[1] is port 4, which B does not have

    assert not result["success"]
    assert result["reason"] == "direction_not_available"
    assert graph.current_edge == before


def test_move_accepts_enum_like_objects(city):
    class FakeEnum:
        name = "STRAIGHT"

    graph = GraphMap(city, ("A", 1, "B", 1))
    assert graph.move(FakeEnum())["success"]


def test_record_gate_stamps_the_current_street(city, gate_config):
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    assert graph.record_gate(5) == city_map.GATE_RECORDED
    assert graph.edges["A1__B1"]["gate_id"] == 5
    assert graph.edges["A1__B1"]["gate_colour"] == "pink"
    assert graph.edge_key_for_gate(5) == "A1__B1"


def test_record_gate_is_idempotent(city, gate_config):
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    assert graph.record_gate(5) == city_map.GATE_RECORDED
    assert graph.record_gate(5) == city_map.GATE_ALREADY_SET
    assert graph.gate_edges() == [{
        "u": "A", "v": "B", "edge_key": "A1__B1",
        "gate_id": 5, "gate_colour": "pink",
    }]


def test_a_mapped_gate_is_never_overwritten(city, gate_config):
    """
    The bug this guards: a tag glimpsed across an intersection replacing the
    gate that was correctly mapped while driving the street.
    """
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    assert graph.record_gate(5) == city_map.GATE_RECORDED
    assert graph.record_gate(9) == city_map.GATE_EDGE_LOCKED

    assert graph.edges["A1__B1"]["gate_id"] == 5
    assert graph.edges["A1__B1"]["gate_colour"] == "pink"
    assert graph.edge_key_for_gate(9) is None


def test_a_gate_cannot_be_mapped_onto_two_streets(city, gate_config):
    """A gate is physically on one street, so the second claim must lose."""
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
    graph.record_gate(5)

    graph.move("STRAIGHT")
    assert graph.current_edge_key != "A1__B1"

    assert graph.record_gate(5) == city_map.GATE_ELSEWHERE
    assert graph.edge_key_for_gate(5) == "A1__B1"
    assert graph.edges[graph.current_edge_key]["gate_id"] is None


def test_force_overrides_the_lock(city, gate_config):
    """Seeding a known map is allowed to write over what is there."""
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
    graph.record_gate(5)

    assert graph.record_gate(9, force=True) == city_map.GATE_RECORDED
    assert graph.edges["A1__B1"]["gate_id"] == 9


def test_record_gate_without_a_known_edge(city, gate_config):
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    assert graph.record_gate(5, edge_key_override="X9__Y9") == city_map.GATE_NO_EDGE


def test_gate_is_direction_independent(city, gate_config):
    """Driving a street the other way must land the gate on the same key."""
    forward = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)
    forward.record_gate(5)

    backward = GraphMap(city, ("B", 1, "A", 1), gate_config=gate_config)
    backward.record_gate(5)

    assert forward.current_edge_key == backward.current_edge_key
    assert forward.edge_key_for_gate(5) == backward.edge_key_for_gate(5)


def test_unknown_gate_id_records_without_a_colour(city, gate_config):
    graph = GraphMap(city, ("A", 1, "B", 1), gate_config=gate_config)

    assert graph.record_gate(99)
    assert graph.edges["A1__B1"]["gate_id"] == 99
    assert graph.edges["A1__B1"]["gate_colour"] is None


def test_coverage_tracking(city):
    graph = GraphMap(city, ("A", 1, "B", 1))

    assert not graph.is_fully_explored()
    assert "A1__B1" not in graph.unvisited_edge_keys()

    for key in list(graph.unvisited_edge_keys()):
        graph.edges[key]["visited"] = True

    assert graph.is_fully_explored()


def test_re_seeding_position_for_the_gate_run(city):
    """The sheet allows the two phases to start on different streets."""
    graph = GraphMap(city, ("A", 1, "B", 1))
    graph.start_on_edge(("C", 4, "B", 3))

    assert graph.current_edge_key == "B3__C4"
    assert graph.current_state_tuple() == ("B", 3)


def test_published_state_is_json_serialisable(city):
    graph = GraphMap(city, ("A", 1, "B", 1))

    json.dumps(graph.current_state())
    json.dumps(graph.edges_state())
    json.dumps(graph.gate_edges())
