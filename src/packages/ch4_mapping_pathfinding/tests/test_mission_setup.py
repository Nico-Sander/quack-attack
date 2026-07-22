"""
Tests for the menu's validation and command building.

A mistyped placement is the expensive mistake: the bot believes it, and every
turn after it lands at the wrong intersection with no symptom until the map
comes out wrong. So these lean on rejecting bad input, not on accepting good.
"""

import pytest

import mission_setup
from mission_setup import SetupError


# ---------------------------------------------------------------------------
# Placements
# ---------------------------------------------------------------------------

def test_a_real_placement_is_accepted(city):
    assert mission_setup.parse_start_edge("A,1,B,1", city) == "A,1,B,1"


def test_whitespace_is_tolerated(city):
    assert mission_setup.parse_start_edge(" A , 1 , B , 1 ", city) == "A,1,B,1"


@pytest.mark.parametrize("text", [
    "[A,1,B,1]", "(A,1,B,1)", "[A, 1, B, 1]", " (A,1,B,1) ",
])
def test_brackets_are_tolerated(city, text):
    """How the value looks when copied out of a log or a Python session."""
    assert mission_setup.parse_start_edge(text, city) == "A,1,B,1"


def test_every_legal_placement_parses(city):
    for _edge, arg, _key in mission_setup.legal_start_edges(city):
        assert mission_setup.parse_start_edge(arg, city) == arg


def test_both_directions_of_every_street_are_offered(city):
    import city_map

    placements = mission_setup.legal_start_edges(city)

    assert len(placements) == 2 * len(city_map.all_edge_keys(city))


def test_a_wrong_destination_is_rejected(city):
    """The most likely typo: right street, wrong far end."""
    with pytest.raises(SetupError, match="leads to"):
        mission_setup.parse_start_edge("A,1,C,1", city)


def test_an_unknown_intersection_is_rejected(city):
    with pytest.raises(SetupError, match="No intersection"):
        mission_setup.parse_start_edge("Z,1,B,1", city)


def test_a_port_the_intersection_does_not_have_is_rejected(city):
    """B is a T-junction, so it has no port 4."""
    with pytest.raises(SetupError, match="no port 4"):
        mission_setup.parse_start_edge("B,4,A,1", city)


def test_the_wrong_number_of_fields_is_rejected(city):
    with pytest.raises(SetupError, match="4 comma-separated"):
        mission_setup.parse_start_edge("A,1,B", city)


def test_a_non_numeric_port_is_rejected(city):
    with pytest.raises(SetupError, match="Ports must be numbers"):
        mission_setup.parse_start_edge("A,x,B,1", city)


def test_the_readback_names_the_street_and_destination(city):
    text = mission_setup.describe_start_edge("A,1,B,1", city)

    assert "A1__B1" in text
    assert "towards B" in text


# ---------------------------------------------------------------------------
# Gate order
# ---------------------------------------------------------------------------

def test_a_gate_order_parses():
    assert mission_setup.parse_gate_order("7,5,6") == [7, 5, 6]


def test_spaces_work_as_separators():
    assert mission_setup.parse_gate_order("7 5 6") == [7, 5, 6]


@pytest.mark.parametrize("text", [
    "[7,5,6]", "(7, 5, 6)", "7;5;6", " [7, 5, 6] ",
])
def test_brackets_and_semicolons_are_tolerated(text):
    assert mission_setup.parse_gate_order(text) == [7, 5, 6]


def test_an_empty_order_is_allowed():
    assert mission_setup.parse_gate_order("") == []
    assert mission_setup.parse_gate_order("   ") == []


def test_a_repeated_gate_is_rejected():
    """A gate cannot be due twice."""
    with pytest.raises(SetupError, match="more than once"):
        mission_setup.parse_gate_order("7,5,7")


def test_a_non_numeric_gate_is_rejected():
    with pytest.raises(SetupError, match="must be numbers"):
        mission_setup.parse_gate_order("7,pink")


def test_unknown_ids_are_reported_but_not_rejected(gate_config):
    """The announced order is whatever the organizers say."""
    assert mission_setup.parse_gate_order("42") == [42]
    assert mission_setup.unknown_gates([42, 7], gate_config) == [42]


# ---------------------------------------------------------------------------
# Launch command
# ---------------------------------------------------------------------------

def test_an_unchanged_config_launches_with_no_arguments():
    """Anything left alone must visibly come from the launch file."""
    command = mission_setup.build_launch_command(mission_setup.default_config())

    assert command == ["roslaunch", "ch4_mapping_pathfinding",
                       "ch4_mapping_pathfinding.launch"]


def test_only_changed_values_are_passed():
    config = mission_setup.default_config()
    config["start_edge"] = "C,1,A,3"
    config["dashboard"] = True

    command = mission_setup.build_launch_command(config)

    assert "start_edge:=C,1,A,3" in command
    assert "dashboard:=true" in command
    assert not any(token.startswith("visualization:") for token in command)


def test_booleans_are_rendered_the_way_roslaunch_wants():
    config = mission_setup.default_config()
    config["driving"] = False

    assert "driving:=false" in mission_setup.build_launch_command(config)


def test_an_unknown_option_is_refused():
    config = mission_setup.default_config()
    config["turbo"] = True

    with pytest.raises(SetupError, match="Unknown launch option"):
        mission_setup.build_launch_command(config)


# ---------------------------------------------------------------------------
# Gate run commands
# ---------------------------------------------------------------------------

def test_the_gate_run_commands_match_the_documented_ones():
    commands = mission_setup.build_gate_run_commands(
        "/track", [7, 5, 6], "C,1,A,3", strict_order=False
    )
    rendered = [mission_setup.as_shell(command) for command in commands]

    assert rendered == [
        "rosparam set /track/mapping_pathplanning_node/gate_order 7,5,6",
        "rosparam set /track/mapping_pathplanning_node/gate_run_start_edge C,1,A,3",
        "rosparam set /track/mapping_pathplanning_node/strict_gate_order false",
        "rosservice call /track/mapping_pathplanning_node/start_gate_run",
    ]


def test_the_service_call_is_always_last():
    """Parameters have to be set before the run reads them."""
    for start_edge in (None, "C,1,A,3"):
        commands = mission_setup.build_gate_run_commands(
            "/track", [7], start_edge
        )
        assert commands[-1][0] == "rosservice"


def test_no_placement_means_no_reposition_parameter():
    """Leaving it unset is what makes the bot cross rather than drive off."""
    commands = mission_setup.build_gate_run_commands("/track", [7], None)

    assert not any("gate_run_start_edge" in " ".join(c) for c in commands)


def test_a_trailing_slash_in_the_namespace_does_not_double_up():
    commands = mission_setup.build_gate_run_commands("/track/", [7])

    assert "/track//" not in " ".join(commands[0])
