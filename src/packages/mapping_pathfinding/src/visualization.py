#!/usr/bin/env python3

import os
import json
import cv2
import rospy
import numpy as np
import networkx as nx

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

from std_msgs.msg import String


# Same graph as in the mapping node:
# A1 -> B1
# A2 -> B4
# A3 -> B3
# A4 no connection
# B1 -> A1
# B2 no connection
# B3 -> A3
# B4 -> A2
CITY = {
    "A": {
        1: ("B", 1),
        2: ("B", 4),
        3: ("B", 3),
    },
    "B": {
        1: ("A", 1),
        3: ("A", 3),
        4: ("A", 2),
    },
}


class MappingVisualizationNode:
    def __init__(self):
        rospy.init_node("mapping_visualization_node")

        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.latest_state = None
        self.dirty = False

        self.G = self._build_graph(CITY)
        self.edge_rad = self._compute_edge_rads()

        self.pos = self._compute_node_positions()

        base = f"/{self.vehicle_name}"

        self.sub_state = rospy.Subscriber(
            f"{base}/mapping/state",
            String,
            self._cb_state,
            queue_size=1
        )

        self.window_name = "Duckiebot Mapping Graph"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1000, 650)

        rospy.loginfo("mapping_visualization_node started")
        rospy.loginfo("Subscribing to %s/mapping/state", base)

    # ---------------------------------------------------------------------
    # Graph construction from CITY
    # ---------------------------------------------------------------------
    def _build_graph(self, city):
        G = nx.MultiGraph()
        added_edges = set()

        for node, ports in city.items():
            G.add_node(node)

            for port, target in ports.items():
                next_node, next_port = target
                edge_key = self._edge_key(node, port, next_node, next_port)

                if edge_key in added_edges:
                    continue

                added_edges.add(edge_key)

                G.add_edge(
                    node,
                    next_node,
                    key=edge_key,
                    ports={
                        node: port,
                        next_node: next_port,
                    },
                    gate_id=None,
                    gate_color=None,
                    visited=False,
                )

        return G

    def _edge_key(self, node_a, port_a, node_b, port_b):
        side_1 = f"{node_a}{port_a}"
        side_2 = f"{node_b}{port_b}"
        return "__".join(sorted([side_1, side_2]))

    def _compute_node_positions(self):
        """
        For small graphs like A/B, a fixed layout is easier to read.
        For larger graphs, spring_layout is used automatically.
        """
        nodes = list(self.G.nodes())

        if set(nodes) == {"A", "B"}:
            return {
                "A": (0.0, 0.0),
                "B": (4.0, 0.0),
            }

        return nx.spring_layout(self.G, seed=42)

    def _compute_edge_rads(self):
        """
        Automatically computes different curvatures for parallel MultiGraph edges,
        using only the graph structure.

        Example with three parallel edges:
            -0.85, 0.0, +0.85
        """
        edge_groups = {}

        for u, v, key, data in self.G.edges(keys=True, data=True):
            pair = tuple(sorted([u, v]))
            edge_groups.setdefault(pair, []).append((u, v, key, data))

        edge_rad = {}

        for pair, edges in edge_groups.items():
            def sort_by_ports(edge):
                _, _, _, data = edge
                ports = data.get("ports", {})
                if not ports:
                    return 0
                return min(ports.values())

            edges = sorted(edges, key=sort_by_ports)
            count = len(edges)

            if count == 1:
                edge_rad[edges[0][2]] = 0.0
                continue

            spread = 1.7

            for idx, (_, _, key, _) in enumerate(edges):
                rad = -spread / 2.0 + idx * (spread / (count - 1))
                edge_rad[key] = rad

        return edge_rad

    # ---------------------------------------------------------------------
    # State update from /mapping/state
    # ---------------------------------------------------------------------
    def _cb_state(self, msg):
        try:
            self.latest_state = json.loads(msg.data)
            self._update_graph_from_state(self.latest_state)
            self.dirty = True
        except json.JSONDecodeError:
            rospy.logwarn("Could not decode mapping/state JSON")

    def _update_graph_from_state(self, state):
        """
        Applies visited and gate information from the mapping state.
        """
        move_result = state.get("move_result")

        if move_result and move_result.get("success"):
            new_edge = move_result.get("new_edge")

            if new_edge and len(new_edge) == 4:
                fn, fp, tn, tp = new_edge
                edge_key = self._edge_key(fn, fp, tn, tp)

                if self.G.has_edge(fn, tn, key=edge_key):
                    self.G[fn][tn][edge_key]["visited"] = True

        for gate in state.get("gates", []):
            edge_key = gate.get("edge_key")
            gate_id = gate.get("gate_id")
            gate_color = gate.get("gate_color")

            for u, v, key in self.G.edges(keys=True):
                if key == edge_key:
                    self.G[u][v][key]["gate_id"] = gate_id
                    self.G[u][v][key]["gate_color"] = gate_color

    # ---------------------------------------------------------------------
    # Drawing helpers
    # ---------------------------------------------------------------------
    def _draw_curved_edge(self, ax, u, v, rad, color, width):
        x1, y1 = self.pos[u]
        x2, y2 = self.pos[v]

        patch = FancyArrowPatch(
            (x1, y1),
            (x2, y2),
            connectionstyle=f"arc3,rad={rad}",
            arrowstyle="-",
            linewidth=width,
            color=color,
            shrinkA=45,
            shrinkB=45,
            zorder=1,
        )

        ax.add_patch(patch)

    def _bezier_point(self, p0, p1, t, rad):
        """
        Point on the same curve that FancyArrowPatch uses with arc3,rad.
        This keeps labels aligned with the correct edge.
        """
        p0 = np.array(p0, dtype=float)
        p1 = np.array(p1, dtype=float)

        midpoint = (p0 + p1) / 2.0
        direction = p1 - p0

        control = midpoint + rad * np.array([direction[1], -direction[0]])

        return (
            (1 - t) ** 2 * p0
            + 2 * (1 - t) * t * control
            + t ** 2 * p1
        )

    def _color_for_gate(self, gate_color):
        """
        Used if colors are added later.
        Yellow is used when no color is set.
        """
        if gate_color is None:
            return "#FFD400"

        color_map = {
            "red": "#E8483C",
            "green": "#2A9D8F",
            "blue": "#4C9BE8",
            "yellow": "#FFD400",
            "orange": "#F4A261",
            "purple": "#9B5DE5",
        }

        return color_map.get(str(gate_color).lower(), "#FFD400")

    # ---------------------------------------------------------------------
    # Main drawing function
    # ---------------------------------------------------------------------
    def _draw_to_image(self):
        """
        Draws the current graph state and returns an OpenCV BGR image.
        Parallel MultiGraph edges are separated automatically.
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title("Duckiebot Mapping Graph", fontsize=16)

        # Nodes
        nx.draw_networkx_nodes(
            self.G,
            self.pos,
            node_size=1800,
            node_color="#4C9BE8",
            edgecolors="black",
            linewidths=2,
            ax=ax
        )

        nx.draw_networkx_labels(
            self.G,
            self.pos,
            font_size=18,
            font_color="white",
            font_weight="bold",
            ax=ax
        )

        # Current state
        current_edge_key = None
        current_edge = None
        approaching_node = None
        available_directions = []

        if self.latest_state is not None:
            graph_state = self.latest_state.get("graph", {})
            current_edge_key = graph_state.get("current_edge_key")
            current_edge = graph_state.get("current_edge")
            approaching_node = graph_state.get("approaching_node")
            available_directions = graph_state.get("available_directions", [])

        # Draw edges
        edge_groups = {}

        for u, v, key, data in self.G.edges(keys=True, data=True):
            pair = tuple(sorted([u, v]))
            edge_groups.setdefault(pair, []).append((u, v, key, data))

        for pair, edges in edge_groups.items():
            def sort_by_ports(edge):
                _, _, _, data = edge
                ports = data.get("ports", {})
                if not ports:
                    return 0
                return min(ports.values())

            edges = sorted(edges, key=sort_by_ports)

            for u, v, key, data in edges:
                rad = self.edge_rad.get(key, 0.0)

                is_current = key == current_edge_key
                is_visited = data.get("visited", False)
                has_gate = data.get("gate_id") is not None

                if is_current:
                    edge_color = "#E8483C"
                    width = 5
                elif is_visited:
                    edge_color = "#2A9D8F"
                    width = 3
                else:
                    edge_color = "#BBBBBB"
                    width = 2

                self._draw_curved_edge(
                    ax=ax,
                    u=u,
                    v=v,
                    rad=rad,
                    color=edge_color,
                    width=width
                )

                label_pos = self._bezier_point(
                    self.pos[u],
                    self.pos[v],
                    0.5,
                    rad
                )

                label_x, label_y = label_pos[0], label_pos[1]

                label = str(key)

                ports = data.get("ports", {})
                if u in ports and v in ports:
                    label += f"\n{u}{ports[u]} ↔ {v}{ports[v]}"

                if has_gate:
                    label += f"\nGate {data.get('gate_id')}"

                if is_current:
                    label += "\nCURRENT"

                if is_visited and not is_current:
                    label += "\nvisited"

                ax.text(
                    label_x,
                    label_y,
                    label,
                    fontsize=10,
                    ha="center",
                    va="center",
                    zorder=5,
                    bbox=dict(
                        boxstyle="round,pad=0.35",
                        fc="white",
                        ec=edge_color,
                        lw=2,
                        alpha=0.95
                    )
                )

                # Draw the gate as an additional symbol on the edge
                if has_gate:
                    gate_pos = self._bezier_point(
                        self.pos[u],
                        self.pos[v],
                        0.62,
                        rad
                    )

                    gate_x, gate_y = gate_pos[0], gate_pos[1]

                    ax.scatter(
                        [gate_x],
                        [gate_y],
                        s=220,
                        marker="s",
                        c=self._color_for_gate(data.get("gate_color")),
                        edgecolors="black",
                        linewidths=1.5,
                        zorder=7
                    )

                    ax.text(
                        gate_x,
                        gate_y + 0.18,
                        f"#{data.get('gate_id')}",
                        fontsize=9,
                        ha="center",
                        va="bottom",
                        zorder=8,
                        bbox=dict(
                            boxstyle="round,pad=0.2",
                            fc="white",
                            ec="black",
                            alpha=0.9
                        )
                    )

        # Highlight approaching node
        if approaching_node in self.pos:
            ax.scatter(
                [self.pos[approaching_node][0]],
                [self.pos[approaching_node][1]],
                s=2600,
                facecolors="none",
                edgecolors="#FFD400",
                linewidths=4,
                zorder=6
            )

        # Info box
        reason = None
        last_turn = None
        active_mode = None
        tag_id = None

        if self.latest_state is not None:
            reason = self.latest_state.get("reason")
            last_turn = self.latest_state.get("last_turn_direction")
            active_mode = self.latest_state.get("active_mode")
            tag_id = self.latest_state.get("tag_id")

        info_lines = [
            f"Reason: {reason}",
            f"Mode: {active_mode}",
            f"Last turn: {last_turn}",
            f"Current edge: {current_edge}",
            f"Edge key: {current_edge_key}",
            f"Approaching node: {approaching_node}",
            f"Available: {available_directions}",
        ]

        if tag_id is not None:
            info_lines.append(f"Detected gate: {tag_id}")

        ax.text(
            0.02,
            0.02,
            "\n".join(info_lines),
            transform=ax.transAxes,
            fontsize=10,
            va="bottom",
            ha="left",
            zorder=10,
            bbox=dict(
                boxstyle="round,pad=0.45",
                fc="white",
                ec="black",
                alpha=0.92
            )
        )

        # Legend
        legend_text = (
            "Red = current edge\n"
            "Green = visited\n"
            "Gray = not visited\n"
            "Yellow ring = next node\n"
            "Square = detected gate"
        )

        ax.text(
            0.98,
            0.02,
            legend_text,
            transform=ax.transAxes,
            fontsize=10,
            va="bottom",
            ha="right",
            zorder=10,
            bbox=dict(
                boxstyle="round,pad=0.45",
                fc="white",
                ec="black",
                alpha=0.92
            )
        )

        ax.axis("off")

        # Automatically expand the visible layout area
        xs = [p[0] for p in self.pos.values()]
        ys = [p[1] for p in self.pos.values()]

        margin_x = 1.2
        margin_y = 3.0

        ax.set_xlim(min(xs) - margin_x, max(xs) + margin_x)
        ax.set_ylim(min(ys) - margin_y, max(ys) + margin_y)

        plt.tight_layout()

        fig.canvas.draw()
        width, height = fig.canvas.get_width_height()

        img_rgb = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img_rgb = img_rgb.reshape((height, width, 3))

        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

        plt.close(fig)

        return img_bgr

    # ---------------------------------------------------------------------
    # Main Loop
    # ---------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.latest_state is not None and self.dirty:
                img = self._draw_to_image()

                cv2.imshow(self.window_name, img)
                cv2.waitKey(1)

                self.dirty = False
            else:
                cv2.waitKey(1)

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = MappingVisualizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass