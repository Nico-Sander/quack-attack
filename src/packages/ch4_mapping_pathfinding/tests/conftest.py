"""
Test setup.

The package's Python modules live flat in src/ because that is how rosrun
imports them, so the test suite puts that directory on sys.path. Only the
ROS-free modules (city_map, planner) are importable here -- the node modules
pull in rospy and are exercised on the robot.
"""

import os
import sys

import pytest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(PACKAGE_ROOT, "src")
CONFIG_DIR = os.path.join(PACKAGE_ROOT, "config")

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import city_map  # noqa: E402  (needs the sys.path tweak above)


# The suite is written against the small practice track (docs/challenge-4-
# graph.png): 3 nodes, 5 streets, every case hand-checkable. config/city.json is
# now the challenge city, so the fixtures name the practice file explicitly.
PRACTICE_CITY_PATH = os.path.join(CONFIG_DIR, "city_practice.json")


@pytest.fixture(scope="session")
def city():
    """The practice track (docs/challenge-4-graph.png)."""
    loaded, _layout = city_map.load_city(PRACTICE_CITY_PATH)
    return loaded


@pytest.fixture(scope="session")
def gate_config():
    return city_map.load_gate_config(os.path.join(CONFIG_DIR, "gates.json"))


def grid_city(rows, cols):
    """
    Builds a rows x cols grid city for planner stress tests.

    Ports follow the compass reading of the challenge convention:
    1 = North, 2 = West, 3 = South, 4 = East. Entering through port 1 means
    arriving from the north and driving south, so the right-hand exit is west --
    which is port 2, matching RIGHT_OF[1] == 2. Port 1 is opposite 3 and 2 is
    opposite 4, as required.

    Node names are letter pairs ("AA", "AB", ...) because edge keys concatenate
    node and port, so digits in a node name would make them ambiguous.
    """
    if rows > 26 or cols > 26:
        raise ValueError("grid_city only names up to 26x26 nodes")

    def name(row, col):
        return chr(ord("A") + row) + chr(ord("A") + col)

    city = {}

    for row in range(rows):
        for col in range(cols):
            ports = {}

            if row > 0:
                ports[1] = (name(row - 1, col), 3)      # north
            if col > 0:
                ports[2] = (name(row, col - 1), 4)      # west
            if row < rows - 1:
                ports[3] = (name(row + 1, col), 1)      # south
            if col < cols - 1:
                ports[4] = (name(row, col + 1), 2)      # east

            city[name(row, col)] = ports

    city_map.validate_city(city)
    return city
