# Current Implementation — Architecture & Capabilities

This document describes what the Duckietown Challenge-4 package currently does, how the pieces fit together, and where the implementation stands relative to the challenge description in `01-information.md` / `02-additional-information.md`.

## High-level architecture

The stack is a classic perception → decision → action pipeline in ROS 1, with the mapping layer sitting alongside it as a passive observer. All nodes communicate over topics namespaced under `/$VEHICLE_NAME`.

```
camera_node/image/compressed
        │
        ├──────────────► detect_lane.py (U-Net segmentation)
        │                   ├─► /detect/lane          (Float64, cross-track error)
        │                   ├─► /detect/lane_borders  (String/JSON, incl. red-line geometry)
        │                   └─► /debug/lane_*         (debug images)
        │
        └──────────────► detect_signs.py (AprilTag 52h13)
                            └─► /detect/sign          (Int32, tag id)

/detect/lane_borders ──► detect_intersection.py
                            └─► /detect/intersection  (Int32, IntersectionState)

/detect/intersection + /detect/sign ──► switch_control.py (behavior FSM)
                            ├─► /switch/mode            (Int32, DriveMode)
                            └─► /switch/turn_direction  (Int32, TurnDirection)

/switch/mode + /switch/turn_direction + /detect/lane ──► control_wheels.py
                            └─► lane_controller_node/car_cmd (Twist2DStamped)

/switch/mode + /switch/turn_direction + /detect/sign ──► mapping_pathplanning.py
                            └─► /mapping/state (String/JSON, latched)

/mapping/state ──► visualization.py (OpenCV window with the live graph)
camera + debug topics ──► dashboard.py (OpenCV debug dashboard)
```

An important architectural fact: information flows *into* the mapping node but never *out of it* back to the driving stack. `mapping_pathplanning.py` tracks where the robot is on the graph, but it does not influence where the robot goes. Turn decisions are made randomly in `switch_control.py`.

## Node-by-node description

### detect_lane.py — semantic lane perception

Runs a MobileNet-v2 U-Net (4 classes: background, white, yellow, red) on the bottom third of the camera image, resized to 192×192, TorchScript-traced for CPU inference. From the segmentation it computes the lane center as the midpoint between the median white x and median yellow x inside a horizontal search band, with fallbacks when a line is not visible and a sanity check for crossed lines. The search band is moved dynamically below the red stop line when one is visible, so the controller keeps steering on the lane segment in front of the line instead of looking across the intersection.

It publishes a normalized cross-track error in [-1, 1] on `/detect/lane`, and a JSON blob on `/detect/lane_borders` that contains the lane geometry plus red-line features: `red_detected`, `red_distance_y` (normalized y of the closest red pixel, i.e. proximity to the stop line) and `red_angle` (slope of a line fitted through the red pixels, used to square up to the stop line).

### detect_intersection.py — stop-line state machine

A thin logic node that maps the red-line geometry to an `IntersectionState`: `NO_INTERSECTION` when no red is detected, `APPROACHING_INTERSECTION` when red is visible, and `AT_INTERSECTION` once `red_distance_y` crosses the configured threshold (0.95). Published on `/detect/intersection`.

### detect_signs.py — AprilTag detection

Detects tags of the family `tagStandard52h13` at 10 Hz and publishes the id of the *largest* (closest) detection on `/detect/sign`, with a small per-tag cooldown. The database `52DB.yaml` currently only contains the intersection signs (ids 1–4: 4-way, T, left-T, right-T). Gate tags (ids 5–13 per the challenge) are not in the DB; the node handles unknown ids gracefully and still publishes them, logging the type as "unknown".

### switch_control.py — behavior FSM

The central decision node. It runs a four-state drive-mode machine:

`LANE_FOLLOWING → APPROACHING_STOP_LINE → STOPPED → CROSSING_INTERSECTION → LANE_FOLLOWING`

Transitions are driven by the intersection state and timers from `config.json` (stop duration 3 s, per-direction crossing durations, and a red-line ignore window after crossing so the just-crossed stop line does not immediately retrigger).

When an intersection sign (ids 1–4) is seen while driving or approaching, the node looks up the allowed turn directions for that sign type and **locks in a random choice**. That choice is published continuously on `/switch/turn_direction` and consumed by both the wheel controller and the mapping node. If the sign was missed, it defaults to STRAIGHT with a warning. There is no input from the map or from any planner — this is the single biggest gap relative to the challenge.

A minor code-hygiene note: it imports `IntersectionState` from `detect_intersection` instead of `custom_enums` (works, but pulls in the whole node module).

### control_wheels.py — low-level control

Translates the commanded `DriveMode` into wheel commands at 10 Hz. In `LANE_FOLLOWING` / `APPROACHING_STOP_LINE` it runs a PID on the cross-track error (with a slower gain set and reduced speed when approaching), plus a clever "red-line squaring" behavior: as the stop line gets close, steering is blended from lane-centering toward aligning perpendicular to the red line, using `red_angle`. In `STOPPED` it outputs zero. In `CROSSING_INTERSECTION` it executes an **open-loop, timed** maneuver in two phases (drive straight for a per-direction duration, then apply a fixed v/omega turn arc) using parameters from `config.json`. It also forces the Duckiebot FSM to `LANE_FOLLOWING` at startup so its commands are actually applied.

### mapping_pathplanning.py — graph position tracking & gate recording

Despite the file name, this node currently does *mapping and localization on the graph only* — there is no path planning in it.

The `GraphMap` class builds a `networkx.MultiGraph` from a **hard-coded** `CITY` dictionary (currently a reduced two-node test map A↔B with three parallel edges, not the real competition map). Edges carry a stable string key derived from the sorted port pair (e.g. `A1__B1`), the port mapping, a `gate_id`/`gate_color` slot, and a `visited` flag. The port convention follows the challenge sheet: ports 1–4 are arranged cyclically, `OPPOSITE` gives the straight-ahead port, `LEFT_OF`/`RIGHT_OF` give the left/right exits relative to the entry port. U-turns are simply not representable via these tables, matching the "no U-turns" constraint.

Position is tracked as the **directed edge currently being driven**, `(from_node, from_port, to_node, to_port)`, seeded via the ROS param `~start_edge` (default `["A", 1, "B", 1]`) — consistent with the requirement that the start pose is given. Localization is pure graph dead-reckoning: when the node observes the drive-mode transition `CROSSING_INTERSECTION → LANE_FOLLOWING`, it applies the last commanded turn direction to the current entry port, looks up the resulting exit in `CITY`, marks the new edge as visited and advances the position. If the turn is not available in the graph or the edge lookup fails, it logs a warning and *stays where it was* (no recovery/resync mechanism).

Gate handling: any `/detect/sign` id that is not in `KNOWN_SIGN_IDS = {1,2,3,4}` is treated as a gate and stamped onto the *currently driven edge* (with a 2 s per-tag cooldown, idempotent if the same gate is already on that edge). No color is recorded (the parameter exists but nothing supplies it), and there is no guard against a gate tag being visible early/late and thus being stamped onto the wrong edge.

Every relevant event (start, gate detected, crossing finished) publishes a full JSON state on `/mapping/state` (latched): current edge and key, approaching node, entry port, available directions, all recorded gates, the last move result and the reason for the update.

### visualization.py — live graph window

Subscribes to `/mapping/state` and renders the graph with matplotlib into an OpenCV window: nodes, curved parallel edges with port labels, edge coloring (red = current edge, green = visited, gray = unvisited), a yellow ring around the node being approached, gate markers with id and color, plus an info box (mode, last turn, current edge, available directions) and a legend. It rebuilds the same `CITY` graph **duplicated as a second hard-coded copy** — a maintenance hazard, since the two copies must be kept in sync manually.

This covers challenge requirement (a) "the created map" and (c) "where the bot thinks it is" reasonably well. Requirement (b) "the chosen path" cannot be shown yet because no path is ever computed.

### dashboard.py — perception debug dashboard

An independent OpenCV dashboard stitching the AI search window, the three class masks and the raw camera feed with a translucent semantic overlay. Purely for perception debugging; unrelated to the graph.

### graph_only.py — standalone prototype

A self-contained, ROS-free earlier prototype of the same graph ideas (build graph from city dict, record gates on port-exact edges, track the current edge, apply turns, draw). Its logic has been superseded by `GraphMap` in `mapping_pathplanning.py`; it is useful as a reference/demo script only. Note it uses a slightly different port geometry helper (`exit_port` via a CCW list) than the mapping node's `LEFT_OF`/`RIGHT_OF` tables — both encode the same convention (1 and 3 opposite, cyclic order), but only the mapping node's version is live.

### util.py / *.json configs

`util.py` provides a per-node parameter loading + live-update mechanism over `/update_parameters`, used by an older iteration of the stack (`control_lane_node.json`, `detect_lane.json` with HSV thresholds and crop points predate the U-Net approach). The current nodes instead read the central `config.json`. This is legacy and partly dead configuration.

## What the system can do today

Driving: robust lane following with dynamic red-line handling, controlled stopping at intersections, squared-up alignment to the stop line, and timed open-loop crossings in three directions with per-direction tuning.

Perception: semantic lane/stop-line segmentation, intersection-sign recognition (ids 1–4) with sign-type → allowed-directions mapping, and gate-tag detection (any other id).

Mapping: given a start edge, it tracks the robot's directed position on the port-annotated graph purely from commanded turns, records gates on the edge where they were seen (direction-independent, matching "one tag spans the whole street"), and exposes/visualizes everything live.

In effect, the robot currently performs a *random walk* through the city while passively building the gate map — a workable basis for the mapping phase, but nothing more.

## Known gaps and weaknesses (summary)

The detailed work plan is in `plan.md`; in short: there is no path planning of any kind and no way for a plan to command turns (turns are random); there is no notion of the two challenge phases (explore/map vs. timed gate run) or of a target gate sequence; the exploration is unsystematic, so full edge coverage — needed to guarantee all gates are found — is not ensured; the `CITY` map is a hard-coded two-node test graph duplicated in two files rather than the real map loaded from one place; gate colors and the required visiting order are not represented anywhere; the chosen path cannot be displayed; and graph localization has no recovery if a turn fails, a sign is missed (silent STRAIGHT fallback can desynchronize the graph position from reality), or a gate tag is attributed to the wrong edge.
