#!/usr/bin/env python3

"""
City graph representation and live map state.

This module is deliberately free of ROS and networkx imports so the whole
mapping/planning core can be unit tested off-robot. Nodes are intersections,
edges are streets, and a street may carry at most one gate.

Port convention (see docs/02-additional-information.md): ports 1-4 are arranged
cyclically around an intersection, so port 1 is opposite port 3 and port 2 is
always to the right of port 1. Turning is therefore a pure lookup on the port
the bot entered through -- and since none of the tables maps a port onto
itself, U-turns are structurally impossible to express.
"""

import json
import os


DEFAULT_CITY_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../config/city.json"
)
DEFAULT_GATES_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../config/gates.json"
)

DIRECTIONS = ("LEFT", "STRAIGHT", "RIGHT")

# Port numbering runs cyclically so that, for a bot entering through port p,
# port p+1 is on its right and port p-1 on its left. That is the challenge
# sheet's "2 ist immer rechts von 1" read from the driver's seat, and it was
# confirmed on the track: entering B through port 1 and turning right comes out
# at B2, and entering A through port 4 and turning right comes out at A1.
#
# Seen from a bot that entered through port 1: port 3 is straight ahead, port 2
# is to the right and port 4 is to the left.
OPPOSITE = {1: 3, 2: 4, 3: 1, 4: 2}
RIGHT_OF = {1: 2, 2: 3, 3: 4, 4: 1}
LEFT_OF = {1: 4, 2: 1, 3: 2, 4: 3}

_DIRECTION_TABLES = {
    "LEFT": LEFT_OF,
    "STRAIGHT": OPPOSITE,
    "RIGHT": RIGHT_OF,
}

# IDs 1-4 are the intersection signs; everything else is treated as a gate.
KNOWN_SIGN_IDS = frozenset({1, 2, 3, 4})

# Outcomes of record_gate(). A gate is written to a street exactly once and is
# never overwritten, so the caller needs to tell "wrote it" apart from the
# several ways a sighting can be correctly ignored.
GATE_RECORDED = "recorded"          # newly stamped onto the street
GATE_ALREADY_SET = "already_set"    # same gate, same street: nothing to do
GATE_EDGE_LOCKED = "edge_locked"    # street already carries a different gate
GATE_ELSEWHERE = "gate_elsewhere"   # this gate is already on another street
GATE_NO_EDGE = "no_edge"            # no current street, or an unknown key


class CityMapError(Exception):
    """Raised when a city definition is malformed or inconsistent."""


# ---------------------------------------------------------------------------
# Port geometry
# ---------------------------------------------------------------------------

def direction_name(direction):
    """Accepts a TurnDirection enum or a plain string and returns the name."""
    if isinstance(direction, str):
        return direction.upper()
    return direction.name


def exit_port(entry_port, direction):
    """Port the bot leaves through when entering via `entry_port` and turning."""
    name = direction_name(direction)

    try:
        table = _DIRECTION_TABLES[name]
    except KeyError:
        raise CityMapError(f"Unknown turn direction: {direction!r}")

    try:
        return table[entry_port]
    except KeyError:
        raise CityMapError(f"Port out of range (expected 1-4): {entry_port!r}")


def edge_key(node_a, port_a, node_b, port_b):
    """
    Stable, direction-independent identifier for a street, e.g. "A1__B1".

    Both endpoints are encoded and sorted, so driving A1->B1 and B1->A1 yields
    the same key -- which is what makes gates direction-independent.

    Node and port are concatenated without a separator to keep the key readable
    in logs and on the visualization. That is only unambiguous while node names
    contain no digits, which validate_city() enforces.
    """
    return "__".join(sorted([f"{node_a}{port_a}", f"{node_b}{port_b}"]))


# ---------------------------------------------------------------------------
# Loading and validation
# ---------------------------------------------------------------------------

def validate_city(city):
    """
    Checks a city dict for the invariants the planner relies on.

    Raises CityMapError on the first problem found.
    """
    if not city:
        raise CityMapError("City is empty")

    for node, ports in city.items():
        if not ports:
            raise CityMapError(f"Node {node!r} has no ports")

        # edge_key() concatenates node and port, so a digit in a node name
        # would make keys ambiguous ("N03" + port 3 vs "N0" + port 33).
        if not node or any(char.isdigit() for char in str(node)):
            raise CityMapError(
                f"Node name {node!r} must be non-empty and contain no digits"
            )

        for port, target in ports.items():
            if port not in OPPOSITE:
                raise CityMapError(
                    f"Node {node!r} uses port {port!r}, expected one of 1-4"
                )

            neighbour, neighbour_port = target

            if neighbour not in city:
                raise CityMapError(
                    f"{node}{port} points at unknown node {neighbour!r}"
                )

            back = city[neighbour].get(neighbour_port)

            if back != (node, port):
                raise CityMapError(
                    f"Asymmetric connection: {node}{port} -> "
                    f"{neighbour}{neighbour_port}, but {neighbour}{neighbour_port} "
                    f"-> {back!r}"
                )

    return True


def parse_city(raw_nodes):
    """
    Normalises a raw JSON 'nodes' mapping into the internal representation.

    JSON object keys are always strings, so ports arrive as "1" and have to be
    converted to int; targets arrive as lists and become tuples.
    """
    city = {}

    for node, ports in raw_nodes.items():
        parsed = {}

        for port, target in ports.items():
            if len(target) != 2:
                raise CityMapError(
                    f"{node}{port} must map to [node, port], got {target!r}"
                )

            parsed[int(port)] = (str(target[0]), int(target[1]))

        city[str(node)] = parsed

    validate_city(city)

    return city


def load_city(path=None):
    """
    Loads the city from JSON. Returns (city, layout).

    `layout` is an optional {node: (x, y)} hint used by the visualization; it is
    empty when the file does not provide one.
    """
    path = path or DEFAULT_CITY_PATH

    with open(path, "r") as handle:
        data = json.load(handle)

    if "nodes" not in data:
        raise CityMapError(f"City file {path} has no 'nodes' key")

    city = parse_city(data["nodes"])
    layout = {
        str(node): tuple(coords)
        for node, coords in data.get("layout", {}).items()
    }

    return city, layout


def load_gate_config(path=None):
    """
    Loads the gate ID -> colour mapping. Returns a dict keyed by int tag ID:

        {5: {"colour": "pink", "hex": "#E5399B"}, ...}

    The mapping is cosmetic (see config/gates.json); callers must tolerate IDs
    that are not listed.
    """
    path = path or DEFAULT_GATES_PATH

    with open(path, "r") as handle:
        data = json.load(handle)

    return {
        int(tag_id): dict(entry)
        for tag_id, entry in data.get("gates", {}).items()
    }


def all_edge_keys(city):
    """Every street in the city, as a set of edge keys."""
    return {
        edge_key(node, port, *target)
        for node, ports in city.items()
        for port, target in ports.items()
    }


# ---------------------------------------------------------------------------
# Live map state
# ---------------------------------------------------------------------------

class GraphMap:
    """
    The city plus everything learned while driving: which streets have been
    traversed and which gate sits on which street.

    Position is the directed edge currently being driven,
    (from_node, from_port, to_node, to_port), so `to_node` is the intersection
    the bot is approaching and `to_port` is the port it will enter through.
    """

    def __init__(self, city, start_edge, gate_config=None):
        self.city = city
        self.gate_config = gate_config or {}

        self.edges = {}
        self._build_edges()

        self.current_edge = None
        self.current_edge_key = None
        self.start_on_edge(start_edge)

    def _build_edges(self):
        for node, ports in self.city.items():
            for port, (neighbour, neighbour_port) in ports.items():
                key = edge_key(node, port, neighbour, neighbour_port)

                if key in self.edges:
                    continue

                # Endpoints are stored in the same sorted order as the key so
                # the representation does not depend on iteration order.
                first, second = sorted([(node, port), (neighbour, neighbour_port)],
                                       key=lambda side: f"{side[0]}{side[1]}")

                self.edges[key] = {
                    "u": first[0],
                    "v": second[0],
                    "ports": {first[0]: first[1], second[0]: second[1]},
                    "gate_id": None,
                    "gate_colour": None,
                    "visited": False,
                }

    # -- position ----------------------------------------------------------

    def start_on_edge(self, edge):
        """
        Seeds (or re-seeds) the position, e.g. when the gate run starts from a
        different street than the mapping phase did.
        """
        from_node, from_port, to_node, to_port = edge
        key = edge_key(from_node, from_port, to_node, to_port)

        if key not in self.edges:
            raise CityMapError(f"Start edge does not exist in the city: {edge}")

        if self.city.get(to_node, {}).get(to_port) != (from_node, from_port):
            raise CityMapError(
                f"Start edge {edge} is not oriented consistently with the city"
            )

        self.current_edge = (from_node, from_port, to_node, to_port)
        self.current_edge_key = key
        self.edges[key]["visited"] = True

    def current_node(self):
        """Intersection the bot is driving towards."""
        return self.current_edge[2]

    def entry_port(self):
        """Port the bot will enter the upcoming intersection through."""
        return self.current_edge[3]

    def current_state_tuple(self):
        """Planner state: (approaching node, port entered through)."""
        return (self.current_node(), self.entry_port())

    # -- turns -------------------------------------------------------------

    def direction_to_exit_port(self, direction):
        return exit_port(self.entry_port(), direction)

    def is_direction_available(self, direction):
        node = self.current_node()

        try:
            port = self.direction_to_exit_port(direction)
        except CityMapError:
            return False

        return port in self.city.get(node, {})

    def available_directions(self):
        return [d for d in DIRECTIONS if self.is_direction_available(d)]

    def move(self, direction):
        """
        Advances the position after an intersection has actually been crossed.

        Returns a result dict; on failure the position is left untouched so the
        caller can decide how to recover.
        """
        name = direction_name(direction)
        node = self.current_node()

        if not self.is_direction_available(name):
            return {
                "success": False,
                "reason": "direction_not_available",
                "node": node,
                "entry_port": self.entry_port(),
                "direction": name,
                "available": self.available_directions(),
            }

        port = self.direction_to_exit_port(name)
        next_node, next_entry_port = self.city[node][port]

        old_edge = self.current_edge
        new_edge = (node, port, next_node, next_entry_port)
        new_key = edge_key(node, port, next_node, next_entry_port)

        self.edges[new_key]["visited"] = True
        self.current_edge = new_edge
        self.current_edge_key = new_key

        return {
            "success": True,
            "old_edge": list(old_edge),
            "new_edge": list(new_edge),
            "new_edge_key": new_key,
            "direction": name,
            "exit_port": port,
            "entry_port_next_node": next_entry_port,
            "available_next": self.available_directions(),
        }

    # -- gates -------------------------------------------------------------

    def record_gate(self, gate_id, edge_key_override=None, force=False):
        """
        Stamps a gate onto a street (the one currently being driven by default).

        The first gate written to a street stays there. Later sightings of a
        different tag on the same street are refused, and so is a gate that is
        already recorded on another street -- a gate is physically on exactly
        one street, so a second street claiming it means one of the two
        sightings is wrong, and the first one was made from closer up.

        Without this, a tag glimpsed across an intersection while stopped at the
        line would silently replace the gate that was correctly mapped while
        driving the street, and the gate run would then plan against a map that
        no longer matches the track.

        Returns one of the GATE_* constants. `force` bypasses both locks and is
        for seeding a known map, not for live detections.
        """
        key = edge_key_override or self.current_edge_key

        if key is None or key not in self.edges:
            return GATE_NO_EDGE

        entry = self.edges[key]

        if entry["gate_id"] == gate_id:
            return GATE_ALREADY_SET

        if not force:
            if entry["gate_id"] is not None:
                return GATE_EDGE_LOCKED

            existing = self.edge_key_for_gate(gate_id)

            if existing is not None:
                return GATE_ELSEWHERE

        entry["gate_id"] = gate_id
        entry["gate_colour"] = self.gate_config.get(gate_id, {}).get("colour")

        return GATE_RECORDED

    def gate_edges(self):
        """All streets that carry a gate, as plain dicts for JSON publishing."""
        return [
            {
                "u": data["u"],
                "v": data["v"],
                "edge_key": key,
                "gate_id": data["gate_id"],
                "gate_colour": data["gate_colour"],
            }
            for key, data in sorted(self.edges.items())
            if data["gate_id"] is not None
        ]

    def edge_key_for_gate(self, gate_id):
        """Street carrying `gate_id`, or None if that gate has not been seen."""
        for key, data in sorted(self.edges.items()):
            if data["gate_id"] == gate_id:
                return key

        return None

    # -- coverage ----------------------------------------------------------

    def unvisited_edge_keys(self):
        return {key for key, data in self.edges.items() if not data["visited"]}

    def is_fully_explored(self):
        return not self.unvisited_edge_keys()

    # -- reporting ---------------------------------------------------------

    def current_state(self):
        return {
            "current_edge": list(self.current_edge),
            "current_edge_key": self.current_edge_key,
            "approaching_node": self.current_node(),
            "entry_port": self.entry_port(),
            "available_directions": self.available_directions(),
            "unvisited_edges": sorted(self.unvisited_edge_keys()),
        }

    def edges_state(self):
        """Full edge table, for the visualization to render without a copy of the city."""
        return {
            key: {
                "u": data["u"],
                "v": data["v"],
                "ports": {str(node): port for node, port in data["ports"].items()},
                "gate_id": data["gate_id"],
                "gate_colour": data["gate_colour"],
                "visited": data["visited"],
            }
            for key, data in sorted(self.edges.items())
        }
