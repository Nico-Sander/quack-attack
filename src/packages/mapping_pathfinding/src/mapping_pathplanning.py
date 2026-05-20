#!/usr/bin/env python3
"""
ROS-Node: Tor-Lokalisierung auf EINEM networkx-Graphen (graph_only.py).
Laufzeit-Mapping Tag->Kante, keine YAML-Eintraege, Tore duerfen pro Lauf variieren.

Subscribed:
  /<bot>/switch/turn_direction  Int32 -> TurnDirection   (welche Richtung)
  /<bot>/switch/mode            Int32 -> DriveMode        (Trigger fuers Abbiegen)
  /<bot>/detect/sign            Int32                     (AprilTag-ID)
Published:
  /<bot>/localization/pose      String (JSON)
"""
import os
import json
import rospy
from std_msgs.msg import Int32, String

from graph_only import (build_graph, start_on_edge, turn_at_node,
                        record_gate, current_edge)
from custom_enums import DriveMode, TurnDirection

KNOWN_SIGN_IDS = {1, 2, 3, 4}  # Verkehrsschilder laut 52DB.yaml -> kein Tor


class GraphLocalizationNode:
    def __init__(self):
        rospy.init_node("graph_localization_node")
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        # Stadt-Dict (Param). ROS liefert Listen -> Ports zu Tupeln machen.
        city_param = rospy.get_param("~city", {
            "A": {1: ["B", 1], 2: ["C", 2], 3: ["C", 1], 4: ["B", 2]},
            "B": {1: ["A", 1], 2: ["A", 4], 3: ["C", 4]},
            "C": {1: ["A", 3], 2: ["A", 2], 4: ["B", 3]},
        })
        city = {n: {int(p): (t[0], int(t[1])) for p, t in ports.items()}
                for n, ports in city_param.items()}

        # DER eine Graph
        self.G = build_graph(city)
        s = rospy.get_param("~start_edge", ["A", 1, "B", 1])
        start_on_edge(self.G, s[0], int(s[1]), s[2], int(s[3]))

        # zuletzt kommandierte Richtung + Modus-Tracking
        self.turn_direction = TurnDirection.STRAIGHT
        self.active_mode = DriveMode.LANE_FOLLOWING

        base = f"/{self._vehicle_name}"
        self.pub_pose = rospy.Publisher(
            f"{base}/localization/pose", String, queue_size=1, latch=True)

        self.sub_turn = rospy.Subscriber(
            f"{base}/switch/turn_direction", Int32, self._cb_direction, queue_size=1)
        self.sub_mode = rospy.Subscriber(
            f"{base}/switch/mode", Int32, self._cb_mode, queue_size=1)
        self.sub_sign = rospy.Subscriber(
            f"{base}/detect/sign", Int32, self._cb_sign, queue_size=1)

        rospy.loginfo("graph_localization_node up. Start edge: %s",
                      current_edge(self.G))

    # --- exakt euer Auswertungsmuster ---
    def _cb_direction(self, msg):
        try:
            self.turn_direction = TurnDirection(msg.data)
        except ValueError:
            self.turn_direction = TurnDirection.STRAIGHT

    def _cb_mode(self, msg):
        try:
            new_mode = DriveMode(msg.data)
        except ValueError:
            return
        # Beim Eintritt in die Kreuzungsdurchfahrt: Position fortschreiben
        if (new_mode == DriveMode.CROSSING_INTERSECTION
                and self.active_mode != DriveMode.CROSSING_INTERSECTION):
            turn_at_node(self.G, self.turn_direction.name)
            self._publish_pose(reason="turn")
            rospy.loginfo("Abbiegen %s -> Kante %s",
                          self.turn_direction.name, current_edge(self.G))
        self.active_mode = new_mode

    def _cb_sign(self, msg):
        tag_id = msg.data
        if tag_id in KNOWN_SIGN_IDS:
            return  # Verkehrsschild -> switch_control zustaendig

        cur = current_edge(self.G)
        if cur is None:
            return
        fn, fp, tn, tp = cur
        record_gate(self.G, tag_id, fn, fp, tn, tp, color=None)  # Farbe: s.u.
        self._publish_pose(reason="gate", tag=tag_id)
        rospy.loginfo("Tor %d auf Kante %s%d->%s%d", tag_id, fn, fp, tn, tp)

    def _publish_pose(self, reason, tag=None):
        fn, fp, tn, tp = current_edge(self.G)
        n_gates = sum(1 for _, _, d in self.G.edges(data=True)
                      if d.get("gate_tag") is not None)
        pose = {
            "from_node": fn, "from_port": fp,
            "to_node": tn, "to_port": tp,
            "reason": reason, "tag_id": tag,
            "discovered_gates": n_gates,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.pub_pose.publish(String(data=json.dumps(pose)))


if __name__ == "__main__":
    try:
        GraphLocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass