"""
Tests for the drawn map geometry.

The property under test is that **streets must not be drawn crossing each
other**. A crossing reads as an intersection, and the city has no intersections
beyond its nodes and no overpasses -- so a crossing in the picture is a lie
about the map.
"""

import math

import pytest

import city_map
import graph_layout as gl


def build_drawing_inputs(city, layout):
    """The same edge list, pair grouping and positions the renderer builds."""
    edges, pairs, seen = [], {}, set()

    for node, ports in sorted(city.items()):
        for port, (neighbour, neighbour_port) in sorted(ports.items()):
            key = city_map.edge_key(node, port, neighbour, neighbour_port)

            if key in seen:
                continue

            seen.add(key)
            edges.append((key, node, neighbour))
            pairs.setdefault(tuple(sorted([node, neighbour])), []).append(
                (key, min(port, neighbour_port))
            )

    return edges, pairs, layout


@pytest.fixture
def drawing(city):
    _c, layout = city_map.load_city(city_map.DEFAULT_CITY_PATH)
    return build_drawing_inputs(city, layout)


# ---------------------------------------------------------------------------
# The detector itself must work
# ---------------------------------------------------------------------------
# Guard against a detector that simply never finds anything: that would make
# every layout look clean, which is exactly how a real crossing slipped through
# once already.

def test_segments_intersect_detects_a_real_crossing():
    assert gl.segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))


def test_segments_intersect_ignores_non_crossing_segments():
    assert not gl.segments_intersect((0, 0), (1, 0), (0, 1), (1, 1))
    assert not gl.segments_intersect((0, 0), (1, 1), (2, 2), (3, 3))


def test_find_crossings_detects_a_deliberately_crossed_layout():
    """An X: two straight streets through each other."""
    positions = {"A": (0, 0), "B": (2, 2), "C": (0, 2), "D": (2, 0)}
    edges = [("A1__B1", "A", "B"), ("C1__D1", "C", "D")]

    crossings = gl.find_crossings(edges, positions, {"A1__B1": 0.0, "C1__D1": 0.0})

    assert crossings == [("A1__B1", "C1__D1")]


def test_find_crossings_ignores_streets_meeting_at_a_node():
    """Two streets sharing a node touch there; that is not a crossing."""
    positions = {"A": (0, 0), "B": (2, 1), "C": (2, -1)}
    edges = [("A1__B1", "A", "B"), ("A2__C1", "A", "C")]

    assert gl.find_crossings(edges, positions,
                             {"A1__B1": 0.0, "A2__C1": 0.0}) == []


def test_excessive_curvature_would_be_caught(drawing):
    """
    The old value. If this ever stops crossing, the detector has gone blind.
    """
    edges, pairs, positions = drawing
    rads = gl.compute_edge_rads(pairs, spread=1.7)

    assert gl.find_crossings(edges, positions, rads)


# ---------------------------------------------------------------------------
# Bezier geometry
# ---------------------------------------------------------------------------

def test_curve_starts_and_ends_at_its_nodes():
    assert gl.bezier_point((0, 0), (4, 2), 0.0, 0.5) == pytest.approx((0, 0))
    assert gl.bezier_point((0, 0), (4, 2), 1.0, 0.5) == pytest.approx((4, 2))


def test_zero_curvature_is_a_straight_line():
    for t in (0.25, 0.5, 0.75):
        x, y = gl.bezier_point((0, 0), (4, 2), t, 0.0)
        assert (x, y) == pytest.approx((4 * t, 2 * t))


def test_curvature_bows_the_curve_off_the_chord():
    straight = gl.bezier_point((0, 0), (4, 0), 0.5, 0.0)
    bowed = gl.bezier_point((0, 0), (4, 0), 0.5, 0.5)

    assert bowed[1] != pytest.approx(straight[1])


def test_opposite_curvatures_are_mirror_images():
    up = gl.bezier_point((0, 0), (4, 0), 0.5, 0.4)
    down = gl.bezier_point((0, 0), (4, 0), 0.5, -0.4)

    assert up[0] == pytest.approx(down[0])
    assert up[1] == pytest.approx(-down[1])


def test_canonical_endpoints_are_order_independent():
    assert gl.canonical_endpoints("B", "A") == gl.canonical_endpoints("A", "B")


def test_curve_points_are_trimmed_away_from_the_nodes():
    """The gap around a node marker, so touching streets do not read as crossing."""
    points = gl.curve_points((0, 0), (4, 0), 0.0, samples=10, trim=True)

    assert points[0][0] > 0.0
    assert points[-1][0] < 4.0


# ---------------------------------------------------------------------------
# Parallel streets
# ---------------------------------------------------------------------------

def test_single_street_is_drawn_straight():
    rads = gl.compute_edge_rads({("A", "B"): [("A1__B1", 1)]})
    assert rads == {"A1__B1": 0.0}


def test_parallel_streets_bow_symmetrically():
    rads = gl.compute_edge_rads(
        {("A", "B"): [("A1__B1", 1), ("A4__B2", 2)]}, spread=0.45
    )

    assert rads["A1__B1"] == pytest.approx(-0.225)
    assert rads["A4__B2"] == pytest.approx(0.225)
    assert sum(rads.values()) == pytest.approx(0.0)


def test_curvature_assignment_is_deterministic():
    """Same input, same picture -- regardless of dict ordering upstream."""
    forward = gl.compute_edge_rads(
        {("A", "B"): [("A1__B1", 1), ("A4__B2", 2)]}
    )
    reversed_input = gl.compute_edge_rads(
        {("A", "B"): [("A4__B2", 2), ("A1__B1", 1)]}
    )

    assert forward == reversed_input


# ---------------------------------------------------------------------------
# The real city
# ---------------------------------------------------------------------------

def test_real_city_is_drawn_without_any_street_crossings(drawing):
    edges, pairs, positions = drawing
    rads = gl.compute_edge_rads(pairs)

    crossings = gl.find_crossings(edges, positions, rads)

    assert crossings == [], (
        f"streets drawn crossing each other: {crossings}. A crossing reads as "
        f"an intersection that does not exist on the track."
    )


def test_real_city_labels_do_not_sit_on_top_of_each_other(drawing):
    edges, pairs, positions = drawing
    rads = gl.compute_edge_rads(pairs)

    # Parallel streets are the tight case; their labels are ~0.9 apart at the
    # default spread and a label box is roughly 0.5 units tall.
    assert gl.min_label_separation(edges, positions, rads) > 0.6


def test_default_spread_has_margin_before_streets_cross(drawing):
    """
    The chosen curvature should not sit right on the edge of crossing, so a
    small layout change does not silently produce a misleading map.
    """
    edges, pairs, positions = drawing

    for factor in (1.0, 1.2, 1.4):
        rads = gl.compute_edge_rads(pairs, spread=gl.DEFAULT_SPREAD * factor)
        assert gl.find_crossings(edges, positions, rads) == [], (
            f"crossings appear at only {factor:.1f}x the default curvature"
        )


def _distance_to_segment(point, a, b):
    """Shortest distance from a point to the segment a-b."""
    (px, py), (ax, ay), (bx, by) = point, a, b
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy

    if length_squared == 0:
        return math.dist(point, a)

    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))

    return math.dist(point, (ax + t * dx, ay + t * dy))


def distance_to_curve(point, curve):
    """Shortest distance from a point to a sampled curve, not just its vertices."""
    return min(
        _distance_to_segment(point, curve[i], curve[i + 1])
        for i in range(len(curve) - 1)
    )


def test_every_street_label_lands_on_its_own_curve(drawing):
    """
    Labels are placed with the same geometry the streets are drawn with, so a
    label must lie on its street. This is what broke when the renderer used
    matplotlib's display-space arc while the labels used data-space maths.
    """
    edges, pairs, positions = drawing
    rads = gl.compute_edge_rads(pairs)
    labels = gl.label_positions(edges, positions, rads)

    for key, u, v in edges:
        start, end = gl.canonical_endpoints(u, v)
        curve = gl.curve_points(positions[start], positions[end],
                                rads[key], samples=200, trim=False)

        # Measured against the segments, so the result is the true point-to-
        # curve distance rather than the sampling resolution.
        distance = distance_to_curve(labels[key], curve)
        assert distance < 1e-4, f"{key} label is not on its own street"
