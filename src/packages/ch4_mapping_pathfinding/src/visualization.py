#!/usr/bin/env python3

"""
Live view of the map, the planned path and the believed position.

This covers the three things the challenge asks to be made visible:
  (a) the map that was built      -- streets, with the gates found on them
  (b) the chosen path             -- blue dashed overlay with step numbers
  (c) where the bot thinks it is  -- red current street, ring on the next node

The city itself is read from the same config file the mapping node uses, so
there is only ever one definition of the map. Everything else comes in over
/mapping/state.
"""

import json
import os

import cv2
import numpy as np
import networkx as nx
import rospy

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from std_msgs.msg import String

import city_map
import gate_detection
import graph_layout


CURRENT_COLOUR = "#E8483C"
VISITED_COLOUR = "#2A9D8F"
UNVISITED_COLOUR = "#BBBBBB"
ROUTE_COLOUR = "#3D5AFE"
NEXT_NODE_COLOUR = "#FFD400"

# Where the info box and the legend sit, in axes fractions: the bottom-left and
# bottom-right corners.
INFO_BOX_XY = (0.02, 0.02)
LEGEND_XY = (0.98, 0.02)

# Clearance kept between the bottom boxes and the lowest thing on the map, in
# axes fractions. A street label is anchored at its centre and is up to five
# lines tall, so this has to cover half a label rather than just a hairline.
BOX_CLEARANCE = 0.07


class MappingVisualizationNode:
    def __init__(self):
        rospy.init_node("mapping_visualization_node")

        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        city_path = rospy.get_param("~city_path", city_map.DEFAULT_CITY_PATH)
        gates_path = rospy.get_param("~gates_path", city_map.DEFAULT_GATES_PATH)

        self.city, self.layout = city_map.load_city(city_path)
        self.gate_config = city_map.load_gate_config(gates_path)

        with open(gates_path, "r") as handle:
            self.unknown_gate_hex = json.load(handle).get(
                "unknown_gate_hex", "#999999"
            )

        # Headless mode renders without opening a window, for machines with no
        # display. Combined with snapshot_path it is how the layout gets checked
        # without a screen, and how a finished map is captured for a report.
        self.headless = bool(rospy.get_param("~headless", False))
        self.snapshot_path = str(rospy.get_param("~snapshot_path", ""))

        self.latest_state = None
        self.dirty = False
        # edge_key -> measured seconds, filled in from /mapping/state.
        self.street_times = {}

        self.G = self._build_graph()
        self.edge_rad = self._compute_edge_rads()
        self.pos = self._compute_node_positions()

        base = f"/{self.vehicle_name}"

        self.sub_state = rospy.Subscriber(
            f"{base}/mapping/state", String, self._cb_state, queue_size=1
        )

        self.window_name = "Duckiebot Mapping Graph"

        if not self.headless:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1100, 700)

        rospy.loginfo("mapping_visualization_node started, listening on %s/mapping/state",
                      base)

        # A street drawn across another one reads as an intersection that does
        # not exist. This should never fire (tests/test_layout.py guards it),
        # but a bad city layout would surface here rather than silently.
        crossings = graph_layout.find_crossings(
            self._edge_list(), self.pos, self.edge_rad
        )
        if crossings:
            rospy.logwarn("Layout: streets drawn crossing each other: %s",
                          crossings)

    # ------------------------------------------------------------------
    # Graph construction
    # ------------------------------------------------------------------

    def _build_graph(self):
        """
        Builds the drawing graph from the shared city definition.

        Only structure lives here; visited/gate state is applied from
        /mapping/state so this node never has to second-guess the mapping node.
        """
        graph = nx.MultiGraph()

        for node, ports in self.city.items():
            graph.add_node(node)

            for port, (neighbour, neighbour_port) in ports.items():
                key = city_map.edge_key(node, port, neighbour, neighbour_port)

                if graph.has_edge(node, neighbour, key=key):
                    continue

                graph.add_edge(
                    node, neighbour, key=key,
                    ports={node: port, neighbour: neighbour_port},
                    gate_id=None, gate_colour=None, visited=False,
                )

        return graph

    def _compute_node_positions(self):
        """Uses the layout hint from city.json when present, else a spring layout."""
        if all(node in self.layout for node in self.G.nodes()):
            return {node: tuple(self.layout[node]) for node in self.G.nodes()}

        rospy.logwarn("City file has no complete layout; falling back to spring layout")
        return nx.spring_layout(self.G, seed=42)

    def _compute_edge_rads(self):
        """
        Fans out parallel streets so both connections between two nodes stay
        visible (A and B are joined twice on the test track).

        The bow is deliberately small: a drawn street that swings wide can
        cross another one, and a crossing reads as an intersection that does
        not exist. See graph_layout.DEFAULT_SPREAD.
        """
        groups = {}

        for u, v, key, data in self.G.edges(keys=True, data=True):
            groups.setdefault(tuple(sorted([u, v])), []).append(
                (key, min(data["ports"].values()))
            )

        return graph_layout.compute_edge_rads(groups)

    def _edge_list(self):
        """(edge_key, u, v) for every street, for the layout helpers."""
        return [(key, u, v) for u, v, key in self.G.edges(keys=True)]

    # ------------------------------------------------------------------
    # State handling
    # ------------------------------------------------------------------

    def _cb_state(self, msg):
        try:
            self.latest_state = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("Could not decode mapping/state JSON")
            return

        self._apply_edge_state(self.latest_state.get("edges", {}))
        self.street_times = self.latest_state.get("timing", {}).get(
            "street_times", {}
        )
        self.dirty = True

    def _apply_edge_state(self, edges):
        """Copies the mapping node's edge table onto the drawing graph."""
        for u, v, key, data in self.G.edges(keys=True, data=True):
            reported = edges.get(key)

            if reported is None:
                continue

            data["visited"] = reported.get("visited", False)
            data["gate_id"] = reported.get("gate_id")
            data["gate_colour"] = reported.get("gate_colour")

    def _gate_hex(self, gate_id):
        entry = self.gate_config.get(gate_id)
        return entry["hex"] if entry else self.unknown_gate_hex

    # ------------------------------------------------------------------
    # Drawing helpers
    # ------------------------------------------------------------------

    def _draw_curved_edge(self, ax, u, v, rad, colour, width, linestyle="-",
                          zorder=1):
        """
        Draws a street as an explicit Bezier sampled in data coordinates.

        Deliberately not FancyArrowPatch/arc3: that builds its curve in display
        coordinates, so the axis aspect ratio warps the bow. The drawn line then
        no longer matches the geometry the layout tests check, labels drift off
        their own street, and streets can cross where the maths says they do
        not. Sampling graph_layout.curve_points here means what is drawn is
        exactly what is tested.
        """
        points = graph_layout.curve_points(self.pos[u], self.pos[v], rad,
                                           samples=60, trim=True)

        ax.plot(
            [p[0] for p in points], [p[1] for p in points],
            color=colour, linewidth=width, linestyle=linestyle,
            zorder=zorder, solid_capstyle="round",
        )

    def _bezier_point(self, p0, p1, t, rad):
        """Point on the drawn street, so labels and markers track their curve."""
        return graph_layout.bezier_point(p0, p1, t, rad)

    # ------------------------------------------------------------------
    # Main drawing
    # ------------------------------------------------------------------

    def _draw_to_image(self):
        fig, ax = plt.subplots(figsize=(11, 7))

        state = self.latest_state or {}
        graph_state = state.get("graph", {})
        plan_state = state.get("plan", {})
        mission = state.get("mission", {})

        current_edge_key = graph_state.get("current_edge_key")
        approaching_node = graph_state.get("approaching_node")

        route = plan_state.get("route_edge_keys", [])
        # A street can be driven more than once in one route, so every step
        # number it carries is collected.
        route_positions = {}
        for index, key in enumerate(route, start=1):
            route_positions.setdefault(key, []).append(index)

        # Gate-run progress, from what the bot has actually *seen*. Not from
        # `remaining_gates`: that shrinks when a gate's street is entered,
        # because that is when the next leg has to be planned -- but entering a
        # street is not evidence of having passed the gate on it, and a map
        # that says otherwise is claiming something it does not know.
        gate_order = mission.get("gate_order") or []
        done_gates, next_gate, _pending = gate_detection.gate_run_progress(
            gate_order, mission.get("gates_sighted")
        )

        phase = mission.get("phase", "?")
        ax.set_title(f"Duckiebot Mapping Graph  --  {phase}", fontsize=16)

        # Anchors of everything the info box must not land on top of. Filled in
        # as the streets are drawn, then used to choose the box's corner.
        label_points = []

        nx.draw_networkx_nodes(self.G, self.pos, node_size=1800,
                               node_color="#4C9BE8", edgecolors="black",
                               linewidths=2, ax=ax)
        nx.draw_networkx_labels(self.G, self.pos, font_size=18,
                                font_color="white", font_weight="bold", ax=ax)

        for u, v, key, data in self.G.edges(keys=True, data=True):
            # Canonical endpoint order. The sign of `rad` bows the curve
            # relative to the u -> v direction, so if the order varied the same
            # street would bow to the opposite side and could cross a
            # neighbour. networkx does not promise an order; sorting does.
            u, v = sorted((u, v))
            rad = self.edge_rad.get(key, 0.0)

            is_current = key == current_edge_key
            is_visited = data.get("visited", False)
            gate_id = data.get("gate_id")

            # Coverage decides the colour of the street itself, always. Being
            # the current street is drawn *around* it as a halo rather than
            # replacing it: the two say different things, and when the bot
            # stops on the last street at the end of mapping, "every street is
            # green" is exactly the thing worth being able to see.
            if is_visited:
                colour, width = VISITED_COLOUR, 3
            else:
                colour, width = UNVISITED_COLOUR, 2

            if is_current:
                self._draw_curved_edge(ax, u, v, rad, CURRENT_COLOUR,
                                       width + 6, zorder=0)

            self._draw_curved_edge(ax, u, v, rad, colour, width)

            # (b) the chosen path, laid over the map
            if key in route_positions:
                self._draw_curved_edge(ax, u, v, rad, ROUTE_COLOUR, width + 3,
                                       linestyle=(0, (4, 3)), zorder=2)

                marker = self._bezier_point(self.pos[u], self.pos[v], 0.30, rad)
                ax.text(
                    marker[0], marker[1],
                    ",".join(str(step) for step in route_positions[key]),
                    fontsize=10, fontweight="bold", color="white",
                    ha="center", va="center", zorder=9,
                    bbox=dict(boxstyle="circle,pad=0.3", fc=ROUTE_COLOUR,
                              ec="white", lw=1.5),
                )

            label = str(key)
            ports = data.get("ports", {})
            if u in ports and v in ports:
                label += f"\n{u}{ports[u]} <-> {v}{ports[v]}"
            # Measured drive time: intersection exit to stopped at the next red
            # line. This is what the planner charges for the street, so seeing
            # it on the map is how a bad measurement gets noticed.
            street_time = self.street_times.get(key)
            if street_time is not None:
                label += f"\n{street_time:.1f} s"
            if gate_id is not None:
                colour_name = data.get("gate_colour") or "?"
                label += f"\nGate {gate_id} ({colour_name})"
            # Both facts, not one or the other: a street can be driven *and* be
            # the one the bot is on.
            marks = [name for name, on in (("visited", is_visited),
                                           ("CURRENT", is_current)) if on]
            if marks:
                label += "\n" + ", ".join(marks)

            label_pos = self._bezier_point(self.pos[u], self.pos[v], 0.5, rad)
            label_points.append(label_pos)
            ax.text(label_pos[0], label_pos[1], label, fontsize=9,
                    ha="center", va="center", zorder=5,
                    bbox=dict(boxstyle="round,pad=0.35", fc="white",
                              ec=CURRENT_COLOUR if is_current else colour,
                              lw=2, alpha=0.95))

            if gate_id is not None:
                # Well clear of the label box, which sits at t=0.5 and is several
                # lines tall on a street that has a gate.
                gate_pos = self._bezier_point(self.pos[u], self.pos[v], 0.80, rad)

                # During the run, a gate's marker says where it is *in the
                # order*: done, next, or still to come. The gate's own colour is
                # kept as the fill so it stays identifiable either way.
                if gate_id in done_gates:
                    edge_colour, edge_width = VISITED_COLOUR, 3.5
                    tag = f"#{gate_id} PASSED"
                    tag_edge = VISITED_COLOUR
                elif gate_id == next_gate:
                    edge_colour, edge_width = ROUTE_COLOUR, 3.5
                    tag = f"#{gate_id} NEXT"
                    tag_edge = ROUTE_COLOUR
                else:
                    edge_colour, edge_width = "black", 1.5
                    tag = f"#{gate_id}"
                    tag_edge = "black"
                    if gate_id in gate_order:
                        tag = f"#{gate_id} ({gate_order.index(gate_id) + 1})"

                ax.scatter([gate_pos[0]], [gate_pos[1]], s=240, marker="s",
                           c=self._gate_hex(gate_id), edgecolors=edge_colour,
                           linewidths=edge_width, zorder=7)
                ax.text(gate_pos[0], gate_pos[1] + 0.18, tag,
                        fontsize=9, ha="center", va="bottom", zorder=8,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white",
                                  ec=tag_edge, lw=1.5, alpha=0.9))

        # (c) where the bot thinks it is going next
        if approaching_node in self.pos:
            ax.scatter([self.pos[approaching_node][0]],
                       [self.pos[approaching_node][1]],
                       s=2600, facecolors="none", edgecolors=NEXT_NODE_COLOUR,
                       linewidths=4, zorder=6)

        info_box = self._draw_info_box(ax, state, graph_state, plan_state,
                                       mission)
        legend = self._draw_legend(ax)

        ax.axis("off")

        xs = [p[0] for p in self.pos.values()]
        ys = [p[1] for p in self.pos.values()]
        ax.set_xlim(min(xs) - 1.6, max(xs) + 1.6)
        ax.set_ylim(min(ys) - 2.4, max(ys) + 2.4)

        plt.tight_layout()
        fig.canvas.draw()

        # Only meaningful once the limits are set and the figure has been laid
        # out: before that, data coordinates do not map to the final canvas.
        if self._reserve_space_for_boxes(fig, ax, (info_box, legend),
                                         list(self.pos.values()) + label_points):
            fig.canvas.draw()

        width, height = fig.canvas.get_width_height()
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        image = image.reshape((height, width, 3))

        plt.close(fig)

        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    def _draw_info_box(self, ax, state, graph_state, plan_state, mission):
        lines = [
            f"Phase: {mission.get('phase')}",
            f"Mode: {state.get('active_mode')}",
            f"Position: {graph_state.get('current_edge_key')} "
            f"-> node {graph_state.get('approaching_node')} "
            f"(port {graph_state.get('entry_port')})",
            f"Available: {graph_state.get('available_directions')}",
            f"Next turn: {plan_state.get('next_turn')}",
            f"Plan: {plan_state.get('note')}",
        ]

        route = plan_state.get("route_edge_keys") or []
        if route:
            lines.append(f"Route: {' -> '.join(route)}")
            lines.append(f"Estimated: {plan_state.get('estimated_cost')} s")

        gate_order = mission.get("gate_order") or []
        if gate_order:
            done, next_gate, pending = gate_detection.gate_run_progress(
                gate_order, mission.get("gates_sighted")
            )
            lines.append(f"Gates: {len(done)}/{len(gate_order)} seen {done}, "
                         f"next {next_gate}")

            # Where the route is heading, which runs ahead of what has been
            # seen -- the two differing is normal, not a fault.
            routing_to = mission.get("remaining_gates") or []
            if routing_to != pending:
                lines.append(f"       routing to {routing_to}")

            elapsed = mission.get("gate_run_elapsed")
            estimate = mission.get("gate_run_estimate")
            if elapsed is not None and estimate:
                lines.append(f"Run: {elapsed:.1f}s elapsed of ~{estimate:.0f}s")

        unvisited = graph_state.get("unvisited_edges")
        if unvisited:
            lines.append(f"Unvisited: {', '.join(unvisited)}")

        timing = state.get("timing", {})
        turn_times = timing.get("turn_times") or {}
        if turn_times:
            lines.append("Turns: " + ", ".join(
                f"{name} {value:.1f}s" for name, value in sorted(turn_times.items())
            ))

        # After mapping the bot is picked up and placed for the gate run, so
        # this is the number the operator actually needs off the screen.
        for rank, option in enumerate(state.get("start_recommendations") or [], 1):
            if rank == 1:
                lines.append("Best gate-run start positions:")
            lines.append(f"  {rank}. start_edge:={option['start_edge_arg']} "
                         f"({option['cost']:.0f}s)")

        if mission.get("halted"):
            lines.append(">> HOLDING AT RED LINE <<")

        if mission.get("localization_ok") is False:
            lines.append("!! LOCALIZATION LOST !!")

        # Returned rather than dropped: _reserve_space_for_boxes() has to
        # measure it once the figure has been laid out.
        return ax.text(
            *INFO_BOX_XY, "\n".join(lines), transform=ax.transAxes,
            fontsize=9, va="bottom", ha="left", zorder=10, family="monospace",
            bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="black",
                      alpha=0.92),
        )

    def _reserve_space_for_boxes(self, fig, ax, boxes, feature_points):
        """
        Grows the y range downwards until the bottom boxes clear the map.

        The info box and the legend are opaque and pinned to the bottom
        corners, so anything under them is simply not on the picture. That was
        free on the 3-node practice track, where the bottom of the canvas was
        empty; the challenge city fills the canvas and the info box lands on an
        intersection. Moving the box to another corner does not help -- it is
        about a third of the canvas tall, so every corner is occupied.

        So the space is made instead of hunted for: the axes keep the same
        drawing, extended far enough below the lowest node or street label that
        the boxes sit in blank space. Returns True when the limits changed, in
        which case the caller has to draw again.

        `feature_points` are data coordinates: node centres and label anchors.
        """
        if not feature_points:
            return False

        renderer = fig.canvas.get_renderer()
        to_axes = ax.transAxes.inverted()

        # How far up the canvas the boxes reach, as an axes fraction.
        box_top = 0.0
        for box in boxes:
            extent = box.get_window_extent(renderer)
            box_top = max(box_top, to_axes.transform(extent.get_points())[1][1])

        needed = box_top + BOX_CLEARANCE

        lowest = min(
            to_axes.transform(ax.transData.transform(point))[1]
            for point in feature_points
        )

        if lowest >= needed or needed >= 1.0:
            return False

        # Solve for the lower limit that puts the lowest feature at `needed`:
        # (y - low) / (high - low) == needed.
        low, high = ax.get_ylim()
        lowest_data = low + lowest * (high - low)
        ax.set_ylim((lowest_data - needed * high) / (1.0 - needed), high)

        return True

    def _draw_legend(self, ax):
        return ax.text(
            *LEGEND_XY,
            "Red halo = current street\n"
            "Green  = already driven\n"
            "Gray   = not driven yet\n"
            "Blue   = planned path\n"
            "Ring   = next intersection\n"
            "Square = gate found\n"
            "  green border = gate seen\n"
            "  blue border  = gate next\n"
            "N.N s  = measured drive time",
            transform=ax.transAxes, fontsize=9, va="bottom", ha="right",
            zorder=10, family="monospace",
            bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="black",
                      alpha=0.92),
        )

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.latest_state is not None and self.dirty:
                image = self._draw_to_image()

                if self.snapshot_path:
                    cv2.imwrite(self.snapshot_path, image)

                if not self.headless:
                    cv2.imshow(self.window_name, image)

                self.dirty = False

            if not self.headless:
                cv2.waitKey(1)

            rate.sleep()

        if not self.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        MappingVisualizationNode().run()
    except rospy.ROSInterruptException:
        pass
