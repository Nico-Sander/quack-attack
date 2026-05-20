#!/usr/bin/env python3

import os
import json
import rospy
import networkx as nx

from std_msgs.msg import Int32, String
from custom_enums import DriveMode, TurnDirection


# Aktueller Graph:
# A1 -> B1
# A2 -> B4
# A3 -> B3
# A4 keine Verbindung
# B1 -> A1
# B2 keine Verbindung
# B3 -> A3
# B4 -> B2
CITY = {
    "A": {
        1: ("B", 1),
        2: ("B", 4),
        3: ("B", 3),
        # 4: keine Verbindung
    },
    "B": {
        1: ("A", 1),
        # 2: keine Verbindung
        3: ("A", 3),
        4: ("A", 2),
    },
}


# Port-Konvention:
# 1, 2, 3, 4 liegen zyklisch um die Kreuzung.
# OPPOSITE[entry_port] ist "geradeaus raus".
OPPOSITE = {
    1: 3,
    2: 4,
    3: 1,
    4: 2,
}

LEFT_OF = {
    1: 2,
    2: 3,
    3: 4,
    4: 1,
}

RIGHT_OF = {
    1: 4,
    4: 3,
    3: 2,
    2: 1,
}


# IDs von normalen Verkehrsschildern.
# Alles andere wird als Gate/Tor behandelt.
# Bei dir sind das aktuell die 52h13-Verkehrsschilder.
KNOWN_SIGN_IDS = {1, 2, 3, 4}


class GraphMap:
    def __init__(self, city_dict, start_edge):
        self.city = city_dict
        self.G = nx.MultiGraph()

        self.current_edge = None
        self.current_edge_key = None

        self._build_graph()
        self.start_on_edge(start_edge)

    def _build_graph(self):
        added_edges = set()

        for node, ports in self.city.items():
            self.G.add_node(node)

            for port, target in ports.items():
                next_node, next_port = target
                edge_key = self._edge_key(node, port, next_node, next_port)

                if edge_key in added_edges:
                    continue

                added_edges.add(edge_key)

                self.G.add_edge(
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

    def _edge_key(self, node_a, port_a, node_b, port_b):
        side_1 = f"{node_a}{port_a}"
        side_2 = f"{node_b}{port_b}"
        return "__".join(sorted([side_1, side_2]))

    def start_on_edge(self, edge):
        """
        edge = ("A", 1, "B", 1)

        Bedeutung:
        Bot fährt gerade von A Port 1 nach B Port 1.
        Der nächste Kreuzungsknoten ist also B.
        """
        from_node, from_port, to_node, to_port = edge
        edge_key = self._edge_key(from_node, from_port, to_node, to_port)

        if not self.G.has_edge(from_node, to_node, key=edge_key):
            raise ValueError(f"Startkante existiert nicht: {edge}")

        self.current_edge = edge
        self.current_edge_key = edge_key

    def current_node(self):
        """
        Knoten, den der Bot als nächstes erreicht.
        """
        return self.current_edge[2]

    def entry_port(self):
        """
        Port, über den der Bot in den aktuellen Knoten hineinfährt.
        """
        return self.current_edge[3]

    def direction_to_exit_port(self, direction):
        if not isinstance(direction, str):
            direction = direction.name

        entry = self.entry_port()

        if direction == "STRAIGHT":
            return OPPOSITE[entry]

        if direction == "LEFT":
            return LEFT_OF[entry]

        if direction == "RIGHT":
            return RIGHT_OF[entry]

        return OPPOSITE[entry]

    def is_direction_available(self, direction):
        if not isinstance(direction, str):
            direction = direction.name

        node = self.current_node()
        exit_port = self.direction_to_exit_port(direction)

        return node in self.city and exit_port in self.city[node]

    def available_directions(self):
        result = []

        for direction in ["LEFT", "STRAIGHT", "RIGHT"]:
            if self.is_direction_available(direction):
                result.append(direction)

        return result

    def move(self, direction):
        """
        Wird aufgerufen, nachdem eine Kreuzung wirklich fertig durchfahren wurde.
        Input: TurnDirection.LEFT / STRAIGHT / RIGHT oder String.
        """
        if not isinstance(direction, str):
            direction = direction.name

        node = self.current_node()

        if not self.is_direction_available(direction):
            return {
                "success": False,
                "reason": "direction_not_available",
                "node": node,
                "entry_port": self.entry_port(),
                "direction": direction,
                "available": self.available_directions(),
            }

        exit_port = self.direction_to_exit_port(direction)
        next_node, next_entry_port = self.city[node][exit_port]

        old_edge = self.current_edge
        new_edge = (node, exit_port, next_node, next_entry_port)
        new_edge_key = self._edge_key(node, exit_port, next_node, next_entry_port)

        if not self.G.has_edge(node, next_node, key=new_edge_key):
            return {
                "success": False,
                "reason": "edge_not_in_networkx_graph",
                "node": node,
                "exit_port": exit_port,
                "next_node": next_node,
                "next_entry_port": next_entry_port,
            }

        self.G[node][next_node][new_edge_key]["visited"] = True

        self.current_edge = new_edge
        self.current_edge_key = new_edge_key

        return {
            "success": True,
            "old_edge": old_edge,
            "new_edge": new_edge,
            "direction": direction,
            "exit_port": exit_port,
            "entry_port_next_node": next_entry_port,
            "available_next": self.available_directions(),
        }

    def record_gate(self, gate_id, color=None):
        """
        Schreibt Gate-Information auf die aktuell befahrene Kante.
        """
        if self.current_edge is None or self.current_edge_key is None:
            return False

        from_node, _, to_node, _ = self.current_edge

        if not self.G.has_edge(from_node, to_node, key=self.current_edge_key):
            return False

        edge_data = self.G[from_node][to_node][self.current_edge_key]

        # Idempotent: Wenn schon dasselbe Gate geloggt wurde, bleibt es stabil.
        if edge_data.get("gate_id") == gate_id:
            return True

        edge_data["gate_id"] = gate_id
        edge_data["gate_color"] = color

        return True

    def current_state(self):
        return {
            "current_edge": list(self.current_edge),
            "current_edge_key": self.current_edge_key,
            "approaching_node": self.current_node(),
            "entry_port": self.entry_port(),
            "available_directions": self.available_directions(),
        }

    def gate_edges(self):
        gates = []

        for u, v, key, data in self.G.edges(keys=True, data=True):
            if data.get("gate_id") is not None:
                gates.append({
                    "u": u,
                    "v": v,
                    "edge_key": key,
                    "gate_id": data.get("gate_id"),
                    "gate_color": data.get("gate_color"),
                })

        return gates


class MappingPathplanningNode:
    def __init__(self):
        rospy.init_node("mapping_pathplanning_node")

        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        start_edge = rospy.get_param("~start_edge", ["A", 1, "B", 1])

        start_edge = (
            str(start_edge[0]),
            int(start_edge[1]),
            str(start_edge[2]),
            int(start_edge[3]),
        )

        self.graph_map = GraphMap(
            CITY,
            start_edge=start_edge
        )

        self.active_mode = DriveMode.LANE_FOLLOWING
        self.last_turn_direction = TurnDirection.STRAIGHT

        self.last_logged_gate = None
        self.last_logged_gate_time = 0.0
        self.gate_cooldown = float(rospy.get_param("~gate_cooldown", 2.0))

        base = f"/{self.vehicle_name}"

        self.sub_turn = rospy.Subscriber(
            f"{base}/switch/turn_direction",
            Int32,
            self._cb_turn_direction,
            queue_size=1
        )

        self.sub_mode = rospy.Subscriber(
            f"{base}/switch/mode",
            Int32,
            self._cb_mode,
            queue_size=1
        )

        self.sub_sign = rospy.Subscriber(
            f"{base}/detect/sign",
            Int32,
            self._cb_sign,
            queue_size=1
        )

        self.pub_state = rospy.Publisher(
            f"{base}/mapping/state",
            String,
            queue_size=1,
            latch=True
        )

        rospy.loginfo(
            "mapping_pathplanning_node started. Start state: %s",
            self.graph_map.current_state()
        )

        self._publish_state(reason="start")

    def _cb_turn_direction(self, msg):
        try:
            self.last_turn_direction = TurnDirection(msg.data)
        except ValueError:
            rospy.logwarn("Invalid turn direction: %s", msg.data)
            self.last_turn_direction = TurnDirection.STRAIGHT

    def _cb_mode(self, msg):
        try:
            new_mode = DriveMode(msg.data)
        except ValueError:
            rospy.logwarn("Invalid drive mode: %s", msg.data)
            return

        crossing_finished = (
            self.active_mode == DriveMode.CROSSING_INTERSECTION
            and new_mode == DriveMode.LANE_FOLLOWING
        )

        if crossing_finished:
            result = self.graph_map.move(self.last_turn_direction)

            if result["success"]:
                rospy.loginfo(
                    "Graph move success: turn=%s, old_edge=%s, new_edge=%s",
                    result["direction"],
                    result["old_edge"],
                    result["new_edge"]
                )
            else:
                rospy.logwarn(
                    "Graph move failed: reason=%s, node=%s, direction=%s, available=%s",
                    result.get("reason"),
                    result.get("node"),
                    result.get("direction"),
                    result.get("available")
                )

            self._publish_state(reason="crossing_finished", move_result=result)

        self.active_mode = new_mode

    def _cb_sign(self, msg):
        tag_id = int(msg.data)

        if tag_id in KNOWN_SIGN_IDS:
            return

        now = rospy.Time.now().to_sec()

        if (
            self.last_logged_gate == tag_id
            and now - self.last_logged_gate_time < self.gate_cooldown
        ):
            return

        success = self.graph_map.record_gate(gate_id=tag_id)

        if success:
            self.last_logged_gate = tag_id
            self.last_logged_gate_time = now

            rospy.loginfo(
                "Gate %d logged on edge %s",
                tag_id,
                self.graph_map.current_edge_key
            )

            self._publish_state(reason="gate_detected", tag_id=tag_id)
        else:
            rospy.logwarn(
                "Could not log gate %d on current edge",
                tag_id
            )

    def _publish_state(self, reason, tag_id=None, move_result=None):
        state = {
            "reason": reason,
            "tag_id": tag_id,
            "last_turn_direction": self.last_turn_direction.name,
            "active_mode": self.active_mode.name,
            "graph": self.graph_map.current_state(),
            "gates": self.graph_map.gate_edges(),
            "move_result": move_result,
            "stamp": rospy.Time.now().to_sec(),
        }

        self.pub_state.publish(String(data=json.dumps(state)))


if __name__ == "__main__":
    try:
        node = MappingPathplanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass