#!/usr/bin/env python3

"""
Renders an example picture of the visualization window without a Duckiebot.

The point is a figure for the documentation (and a way to eyeball a layout
change) on a machine that has no robot, no roscore and no camera. Layout claims
are only worth something when the image is actually looked at, and that should
not require the track.

Nothing here fakes the *drawing*: it drives the real GraphMap through a real
exploration, plans the gate run with the real planner, and hands the resulting
/mapping/state payload to the real visualization node. Only ROS itself is
stubbed out, because the node reaches for rospy in its constructor and there is
no master to talk to.

    python3 tests/render_example_visualization.py [-o docs/latex/img/visualization.png]

Needs matplotlib, cv2 and networkx. If the host has none of them, run it in the
container image, which does:

    docker run --rm -v "$PWD:/workspace" -w /workspace \
        --entrypoint python3 quack-attack-duckierace_env:latest \
        src/packages/mapping_pathfinding/tests/render_example_visualization.py
"""

import argparse
import json
import os
import sys
import types

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, os.path.abspath(SRC))


def install_ros_stubs(params):
    """
    Puts a minimal fake `rospy` and `std_msgs.msg` in sys.modules.

    Only what the visualization node touches at construction and draw time:
    parameters, a subscriber that is never fed, and the log calls. `params` maps
    the node's private parameter names to the values it should read.
    """
    rospy = types.ModuleType("rospy")

    def get_param(name, default=None):
        return params.get(name, default)

    def _log(fmt, *args):
        print("[rospy] " + (fmt % args if args else fmt), file=sys.stderr)

    rospy.init_node = lambda *a, **k: None
    rospy.get_param = get_param
    rospy.Subscriber = lambda *a, **k: None
    rospy.Publisher = lambda *a, **k: None
    rospy.loginfo = _log
    rospy.logwarn = _log
    rospy.logerr = _log
    rospy.is_shutdown = lambda: False
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.ROSInterruptException = RuntimeError

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self, data=""):
            self.data = data

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    sys.modules["rospy"] = rospy
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


def build_example_state():
    """
    A finished map part-way through the timed gate run.

    That moment shows everything the picture is meant to show at once: every
    street mapped, the gates found on them, a planned route with step numbers,
    and a believed position. Returns the /mapping/state payload.
    """
    import city_map
    import gate_detection  # noqa: F401  (imported by visualization; fail early)
    import planner

    city, _layout = city_map.load_city()
    gate_config = city_map.load_gate_config()

    config_path = os.path.join(SRC, "..", "config", "config.json")
    with open(os.path.abspath(config_path), "r") as handle:
        config = json.load(handle)

    # --- mapping phase: drive until every street has been seen ---------------
    start_edge = ("A", 1, "B", 1)
    graph = city_map.GraphMap(city, start_edge, gate_config=gate_config)
    costs = planner.TurnCosts.from_config(config)

    guard = 0
    while not graph.is_fully_explored() and guard < 50:
        guard += 1
        steps = planner.plan_exploration_step(
            city, graph.current_state_tuple(),
            graph.unvisited_edge_keys(), costs,
        )
        if not steps:
            break
        graph.move(steps[0].direction)

    # --- the gates that mapping found, one per street ------------------------
    # force=True seeds a known map; live sightings go through the filters in
    # gate_detection instead.
    found_gates = {"A1__B1": 5, "B3__C4": 9, "A3__C1": 8, "A2__C2": 7}
    for edge_key, gate_id in found_gates.items():
        graph.record_gate(gate_id, edge_key_override=edge_key, force=True)

    # --- street times measured during that run -------------------------------
    # Real runs feed these from run_timing; here they are plausible constants so
    # the estimate in the info box is not a flat guess.
    street_times = {
        "A1__B1": 5.1, "A2__C2": 6.4, "A3__C1": 4.7,
        "A4__B2": 5.6, "B3__C4": 7.2,
    }
    turn_times = {"LEFT": 2.6, "RIGHT": 1.3, "STRAIGHT": 2.1}

    costs = planner.TurnCosts.from_config(
        config, edge_durations=street_times, turn_durations=turn_times,
    )

    # --- gate run: repositioned by hand, announced order, part way through ---
    gate_order = [7, 5, 9]
    gates_sighted = [7]                 # tag 7 confirmed, 5 and 9 still ahead
    remaining_gates = [5, 9]

    graph.start_on_edge(("C", 2, "A", 2))

    gate_edge_keys = [graph.edge_key_for_gate(gate) for gate in remaining_gates]
    steps, _legs = planner.plan_gate_run(
        city, graph.current_state_tuple(), gate_edge_keys, costs,
        start_street_driven=False,
    )

    estimate = planner.total_cost(steps)

    return {
        "reason": "crossing_finished",
        "stamp": 0.0,
        "tag_id": None,
        "active_mode": "LANE_FOLLOWING",
        "last_turn_direction": "STRAIGHT",

        "mission": {
            "phase": "GATE_RUN",
            "gate_order": gate_order,
            "remaining_gates": remaining_gates,
            "localization_ok": True,
            "halted": False,
            "gate_run_estimate": estimate,
            "gate_run_result": None,
            "gates_sighted": gates_sighted,
            "gate_run_elapsed": 8.4,
        },

        "graph": graph.current_state(),
        "edges": graph.edges_state(),
        "gates": graph.gate_edges(),

        "timing": {
            "street_times": street_times,
            "turn_times": turn_times,
            "street_samples": {key: [value] for key, value in street_times.items()},
            "seed_turn_times": {},
            "default_street_time": costs.edge_duration,
            "rejected": [],
        },

        "start_recommendations": [],

        "plan": {
            "note": "gate run: routing to gate 5",
            "next_turn": steps[0].direction if steps else None,
            "route_edge_keys": planner.route_edge_keys(steps),
            "turns": planner.turn_sequence(steps),
            "steps": planner.steps_as_dicts(steps),
            "estimated_cost": estimate,
        },

        "move_result": {"success": True},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o", "--out",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "docs", "latex", "img", "visualization.png"),
        help="where to write the PNG",
    )
    parser.add_argument(
        "--dpi", type=int, default=200,
        help="canvas resolution; the node draws an 11x7 inch figure, so the "
             "default gives 2200x1400. Layout and font sizes are unaffected.",
    )
    args = parser.parse_args()
    out = os.path.abspath(args.out)

    install_ros_stubs({"~headless": True, "~snapshot_path": out})

    state = build_example_state()

    import cv2
    from std_msgs.msg import String
    import visualization

    # visualization.py already selected the Agg backend on import.
    import matplotlib
    matplotlib.rcParams["figure.dpi"] = args.dpi

    node = visualization.MappingVisualizationNode()
    node._cb_state(String(data=json.dumps(state)))

    image = node._draw_to_image()
    if not cv2.imwrite(out, image):
        raise SystemExit(f"Could not write {out}")

    print(f"wrote {out}  ({image.shape[1]}x{image.shape[0]})")
    print("route:", " -> ".join(state["plan"]["route_edge_keys"]),
          f"({state['plan']['estimated_cost']} s estimated)")


if __name__ == "__main__":
    main()
