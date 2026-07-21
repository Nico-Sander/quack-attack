#!/usr/bin/env python3

"""
Turning answers typed at a menu into a launch command.

ROS-free so it can be unit tested off-robot: everything here is validation and
string building, which is exactly the part that must not be wrong at the track.

A mistyped `start_edge` is the expensive mistake this guards against. The bot
believes whatever it is told about where it starts, and a placement that does
not match the city sends every subsequent turn to the wrong intersection --
with no symptom until the map comes out wrong. So placements are checked
against the city graph here, before anything is launched.
"""

import city_map


LAUNCH_PACKAGE = "mapping_pathfinding"
LAUNCH_FILE = "mapping_pathfinding.launch"
SIMULATE_FILE = "simulate.launch"

# simulate.launch runs the mapping node against a fake driver, so it takes only
# the mission arguments -- there is no perception or driving to configure.
# Passing it anything else is a roslaunch error, so the set is explicit.
SIMULATE_ARGS = frozenset({
    "start_edge", "gate_order", "gate_run_start_edge", "strict_gate_order",
    "auto_start_gate_run", "force_turn", "visualization",
})

# Long enough that the halt outlasts someone reading a menu and typing.
SIMULATE_EXTRA = {"halt_timeout": "600", "step_time": "0.3", "moves": "40"}

# Sentinel for gate_min_area: the launch file passes -1 to mean "use the value
# in config.json" (see detect_signs._load_gate_min_area).
USE_CONFIG = -1


class SetupError(Exception):
    """Raised when an answer cannot be used."""


def _normalise(text):
    """
    Strips the punctuation people naturally type around a list.

    "[A,1,B,1]" and "(7, 5, 6)" are how these values look when copied out of a
    log or a Python session, and refusing them teaches nothing -- the intent is
    unambiguous. Semicolons are accepted as separators for the same reason.
    """
    cleaned = str(text).strip()

    for char in "[]()":
        cleaned = cleaned.replace(char, "")

    return cleaned.replace(";", ",").strip()


# ---------------------------------------------------------------------------
# Placements
# ---------------------------------------------------------------------------

def legal_start_edges(city):
    """
    Every legal placement, as (edge_tuple, "A,1,B,1", "A1__B1") triples.

    A placement is a directed edge: the bot is set down at an intersection exit
    facing along the street, so both directions of every street qualify.
    """
    result = []

    for node in sorted(city):
        for port in sorted(city[node]):
            neighbour, neighbour_port = city[node][port]
            result.append((
                (node, port, neighbour, neighbour_port),
                f"{node},{port},{neighbour},{neighbour_port}",
                city_map.edge_key(node, port, neighbour, neighbour_port),
            ))

    return result


def parse_start_edge(text, city):
    """
    Parses "A,1,B,1" and checks it is a street the city actually has.

    Returns the normalised "A,1,B,1" string. Raises SetupError with something
    actionable rather than letting a bad placement reach the robot.
    """
    parts = [part.strip() for part in _normalise(text).split(",") if part.strip()]

    if len(parts) != 4:
        raise SetupError(
            f"Expected 4 comma-separated values like A,1,B,1 "
            f"(from,port,to,port), got {text!r}"
        )

    from_node, from_port, to_node, to_port = parts

    try:
        from_port = int(from_port)
        to_port = int(to_port)
    except ValueError:
        raise SetupError(f"Ports must be numbers 1-4, got {text!r}")

    if from_node not in city:
        raise SetupError(
            f"No intersection {from_node!r}. Known: {', '.join(sorted(city))}"
        )

    if from_port not in city[from_node]:
        available = ", ".join(str(p) for p in sorted(city[from_node]))
        raise SetupError(
            f"{from_node} has no port {from_port}. It has ports {available}"
        )

    actual = city[from_node][from_port]

    if actual != (to_node, to_port):
        raise SetupError(
            f"{from_node} port {from_port} leads to {actual[0]} port {actual[1]}, "
            f"not {to_node} port {to_port}"
        )

    return f"{from_node},{from_port},{to_node},{to_port}"


def describe_start_edge(text, city):
    """Plain-English readback, so a typo is visible before anything moves."""
    normalised = parse_start_edge(text, city)
    from_node, from_port, to_node, to_port = normalised.split(",")
    key = city_map.edge_key(from_node, int(from_port), to_node, int(to_port))

    return (f"at {from_node} port {from_port}, driving along {key} "
            f"towards {to_node} (entering port {to_port})")


# ---------------------------------------------------------------------------
# Gate order
# ---------------------------------------------------------------------------

def parse_gate_order(text, gate_config=None):
    """
    Parses "7,5,6" into [7, 5, 6].

    Unknown IDs are allowed -- the announced order is whatever the organizers
    say, and gates.json is only a colour lookup -- but duplicates are not, since
    a gate cannot be due twice.
    """
    text = _normalise(text)

    if not text:
        return []

    order = []

    for chunk in text.replace(" ", ",").split(","):
        chunk = chunk.strip()

        if not chunk:
            continue

        try:
            order.append(int(chunk))
        except ValueError:
            raise SetupError(f"Gate IDs must be numbers, got {chunk!r}")

    duplicates = {gate for gate in order if order.count(gate) > 1}

    if duplicates:
        raise SetupError(
            f"Gate(s) {sorted(duplicates)} appear more than once in the order"
        )

    return order


def unknown_gates(order, gate_config):
    """IDs not in gates.json -- worth mentioning, not worth refusing."""
    if not gate_config:
        return []

    return [gate for gate in order if gate not in gate_config]


# ---------------------------------------------------------------------------
# Launch command
# ---------------------------------------------------------------------------

def default_config():
    """The answers a plain mapping run would give."""
    return {
        "start_edge": "A,1,B,1",
        "gate_order": "",
        "driving": True,
        "visualization": True,
        "dashboard": False,
        "strict_gate_order": False,
        "auto_start_gate_run": False,
        "force_turn": "NONE",
        "gate_min_area": USE_CONFIG,
        "gate_confirm_frames": 3,
        "map_gates_while_approaching": True,
        "gate_cooldown": 2.0,
        "command_timeout": 1.0,
        "sign_cooldown": 0.1,
    }


def _as_arg(value):
    """roslaunch wants lowercase booleans."""
    if isinstance(value, bool):
        return "true" if value else "false"

    return str(value)


def build_launch_command(config, simulate=False):
    """
    The roslaunch argv for a mapping run.

    Only values that differ from the defaults are passed, so the printed
    command stays short enough to read back and check -- and so anything left
    alone is visibly coming from the launch file and config.json rather than
    from the menu.

    With `simulate`, the same answers drive simulate.launch instead: the menu
    can then be rehearsed end to end with no robot, which is the only way to
    practise the phase-switch commands before the track.
    """
    defaults = default_config()
    launch_file = SIMULATE_FILE if simulate else LAUNCH_FILE
    command = ["roslaunch", LAUNCH_PACKAGE, launch_file]

    for key in sorted(config):
        if key not in defaults:
            raise SetupError(f"Unknown launch option {key!r}")

        # Options about perception and driving have no meaning without a robot,
        # and simulate.launch does not declare them.
        if simulate and key not in SIMULATE_ARGS:
            continue

        value = config[key]

        if value == defaults[key]:
            continue

        command.append(f"{key}:={_as_arg(value)}")

    if simulate:
        for key, value in sorted(SIMULATE_EXTRA.items()):
            command.append(f"{key}:={value}")

    return command


def build_gate_run_commands(namespace, gate_order, start_edge=None,
                            strict_order=None):
    """
    The rosparam/rosservice calls that start the timed run.

    Deliberately the same commands documented in workflow.md: the menu is a
    convenience over the interface, not a second interface. If the menu is
    unavailable the printed commands can be typed by hand.
    """
    node = f"{namespace.rstrip('/')}/mapping_pathplanning_node"
    commands = []

    order = ",".join(str(gate) for gate in gate_order)
    commands.append(["rosparam", "set", f"{node}/gate_order", order])

    if start_edge:
        commands.append(
            ["rosparam", "set", f"{node}/gate_run_start_edge", start_edge]
        )

    if strict_order is not None:
        commands.append(["rosparam", "set", f"{node}/strict_gate_order",
                         _as_arg(bool(strict_order))])

    commands.append(["rosservice", "call", f"{node}/start_gate_run"])

    return commands


def as_shell(command):
    """A copy-pasteable rendering of one argv."""
    parts = []

    for token in command:
        parts.append(f'"{token}"' if " " in token else token)

    return " ".join(parts)
