#!/usr/bin/env python3

"""
Geometry for drawing the city graph.

Kept free of ROS and matplotlib so the layout can be checked off-robot. The
property that matters is that **drawn streets must not cross**: a crossing on a
road map reads as an intersection, and the city has no intersections beyond its
nodes and no overpasses. Two streets may only meet at a node.

Parallel streets (two nodes joined more than once, as A and B are on the test
track) still have to be told apart, so they are bowed apart symmetrically -- but
only just enough to separate their labels, never enough to sweep across the
rest of the map.
"""

import math


# Perpendicular bow applied to parallel streets, as a fraction of the distance
# between the two nodes. Large enough to separate labels, small enough that a
# street stays in a narrow ribbon around the straight line between its nodes,
# which is what keeps the drawing crossing-free.
#
# On the real city, A3__C1 and A4__B2 start crossing at 0.67; this sits a third
# below that while keeping labels ~0.9 units apart. The old value was 1.7,
# which bowed streets so far they swept across the whole map.
# tests/test_layout.py holds the line.
DEFAULT_SPREAD = 0.45

# Curves are sampled between these t values when checking for crossings. The
# trimmed ends match the gap the renderer leaves around each node marker, so
# two streets meeting at a shared node do not count as crossing.
TRIM_START = 0.14
TRIM_END = 0.86


def bezier_point(p0, p1, t, rad):
    """
    Point at parameter t on the curve matplotlib draws for arc3,rad=<rad>.

    Mirrors FancyArrowPatch's construction so labels and markers land on the
    line that is actually rendered.
    """
    x0, y0 = p0
    x1, y1 = p1

    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    dx, dy = x1 - x0, y1 - y0

    # Control point pushed perpendicular to the chord.
    cx, cy = mid_x + rad * dy, mid_y - rad * dx

    u = 1.0 - t
    return (
        u * u * x0 + 2 * u * t * cx + t * t * x1,
        u * u * y0 + 2 * u * t * cy + t * t * y1,
    )


def compute_edge_rads(edges_by_pair, spread=DEFAULT_SPREAD):
    """
    Assigns a curvature to every street.

    `edges_by_pair` maps a node pair to the list of (edge_key, sort_value)
    joining it. A lone street is drawn straight; parallel ones are fanned
    symmetrically around the straight line so the set stays balanced.
    """
    rads = {}

    for edges in edges_by_pair.values():
        ordered = sorted(edges, key=lambda item: (item[1], item[0]))
        count = len(ordered)

        if count == 1:
            rads[ordered[0][0]] = 0.0
            continue

        for index, (key, _sort_value) in enumerate(ordered):
            rads[key] = -spread / 2.0 + index * (spread / (count - 1))

    return rads


def curve_points(p0, p1, rad, samples=40, trim=True):
    """Polyline approximation of a drawn street."""
    start, end = (TRIM_START, TRIM_END) if trim else (0.0, 1.0)
    step = (end - start) / float(samples - 1)

    return [bezier_point(p0, p1, start + i * step, rad) for i in range(samples)]


def _orientation(a, b, c):
    value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])

    if abs(value) < 1e-12:
        return 0

    return 1 if value > 0 else -1


def segments_intersect(a1, a2, b1, b2):
    """Proper intersection test for two line segments."""
    o1 = _orientation(a1, a2, b1)
    o2 = _orientation(a1, a2, b2)
    o3 = _orientation(b1, b2, a1)
    o4 = _orientation(b1, b2, a2)

    return o1 != o2 and o3 != o4


def curves_cross(curve_a, curve_b):
    """True if two sampled streets cross anywhere."""
    for i in range(len(curve_a) - 1):
        for j in range(len(curve_b) - 1):
            if segments_intersect(curve_a[i], curve_a[i + 1],
                                  curve_b[j], curve_b[j + 1]):
                return True

    return False


def canonical_endpoints(u, v):
    """
    Fixed endpoint order for a street.

    `rad` bows a curve relative to the start -> end direction, so the two
    endpoints must always be taken in the same order or the same street would
    bow to opposite sides in different places. Both the renderer and the
    crossing check go through this.
    """
    return tuple(sorted((u, v)))


def find_crossings(edges, positions, rads):
    """
    Every pair of streets that visually cross.

    `edges` is a list of (edge_key, node_u, node_v). Returns a sorted list of
    (edge_key_a, edge_key_b) pairs -- empty means the drawing is clean.
    """
    curves = {}

    for key, u, v in edges:
        start, end = canonical_endpoints(u, v)
        curves[key] = curve_points(positions[start], positions[end],
                                   rads.get(key, 0.0))

    crossings = []

    for index, (key_a, _u, _v) in enumerate(edges):
        for key_b, _u2, _v2 in edges[index + 1:]:
            if curves_cross(curves[key_a], curves[key_b]):
                crossings.append(tuple(sorted([key_a, key_b])))

    return sorted(set(crossings))


def label_positions(edges, positions, rads, t=0.5):
    """Where each street's label goes: the midpoint of its drawn curve."""
    labels = {}

    for key, u, v in edges:
        start, end = canonical_endpoints(u, v)
        labels[key] = bezier_point(positions[start], positions[end], t,
                                   rads.get(key, 0.0))

    return labels


def min_label_separation(edges, positions, rads, t=0.5):
    """
    Closest distance between any two street labels.

    Parallel streets are the tight case: if their labels sit on top of each
    other the drawing is unreadable even though nothing crosses.
    """
    labels = label_positions(edges, positions, rads, t=t)
    keys = sorted(labels)

    if len(keys) < 2:
        return float("inf")

    return min(
        math.dist(labels[a], labels[b])
        for i, a in enumerate(keys)
        for b in keys[i + 1:]
    )
