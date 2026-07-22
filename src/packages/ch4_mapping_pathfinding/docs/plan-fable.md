# Plan — From Current State to Challenge 4

Goal per the challenge sheets: the Duckiebot gets the city as a graph, drives around, maps the gates (AprilTags 5–13) onto edges, and then drives through the colored gates **in the correct order**, where only the gate run is timed, the start edge is free (and may differ between phases), U-turns are forbidden, and the gate route must not be hard-coded. The map, the chosen path and the believed position must be made visible.

The existing stack already covers driving, perception and graph localization. What is missing is everything that turns the random walk into purposeful behavior. The work splits into seven steps, roughly in dependency order.

## 1. Consolidate the map into a single source of truth

Move the `CITY` dictionary out of `mapping_pathplanning.py` and `visualization.py` (currently two diverging hard-coded copies) into one config file (e.g. `config/city.json` or a ROS param), loaded by both nodes. Enter the real competition graph from the challenge sheet (nodes C, A, B with the port numbering shown in `examplecitygraph.png`) instead of the reduced A↔B test map. While at it, fix the minor import of `IntersectionState` from `detect_intersection` to `custom_enums`, and decide whether `graph_only.py` and the legacy `util.py`/HSV configs are kept as reference or removed.

Acceptance: both nodes start from the same file; swapping the map requires editing exactly one place.

## 2. Close the control loop: planner commands turns

This is the architectural keystone. Today `switch_control.py` picks turns randomly; the mapping node only watches. Invert this:

Add a topic, e.g. `/{veh}/plan/turn_command` (Int32, `TurnDirection`), published by `mapping_pathplanning.py` and subscribed by `switch_control.py`. While a planner command is available and fresh, `switch_control` uses it instead of the random choice; the sign-based random choice remains only as a fallback (useful during bring-up and as a safety net). The mapping node knows the current entry port and the graph, so it can also validate that the commanded direction actually exists — this replaces the current failure mode where a random turn can be impossible in the graph.

Timing matters here: the turn must be locked in before `STOPPED` ends. Simplest robust scheme: the mapping node publishes the desired next turn continuously (latched) as soon as its position updates, so the value is always ready when `switch_control` transitions to `CROSSING_INTERSECTION`. Additionally, the mapping node should be able to say "no preference" (mapping phase, if random exploration is kept) versus "mandatory" (gate run).

Acceptance: with a trivial planner that always says LEFT, the bot demonstrably turns left at every intersection where left exists.

## 3. Introduce the two challenge phases

Add an explicit mission state to the mapping node (and expose it in `/mapping/state`): `MAPPING` and `GATE_RUN`, plus a transition trigger (ROS param, service call, or simply "all target gates found"). The start edge for each phase is a parameter (already exists as `~start_edge`; add the ability to re-seed the position when the gate run starts from a different, known edge — the sheet explicitly allows different start points).

## 4. Systematic exploration for the mapping phase

Gates sit on edges and one tag spans both driving directions, so traversing every edge once (in either direction) guarantees finding every gate. Replace the random walk with a coverage strategy:

Practical and sufficient: greedy nearest-unvisited-edge — at each intersection, plan (using the path search from step 5) to the closest edge still marked unvisited, drive it, repeat; terminate when all edges are visited or when all expected gates (ids 5–13 that are actually placed) have been recorded early. The mapping phase is not timed, so optimality (Chinese-Postman style routing) is unnecessary; correctness of coverage is what counts. The `visited` flag on edges already exists and is maintained.

Edge case to respect: coverage planning must honor the no-U-turn constraint, so it needs the same state-space search as step 5.

## 5. Path planning with the no-U-turn constraint (the core algorithm)

Because U-turns are forbidden, the reachable next edges depend on how the current edge was entered. Plan therefore not over nodes but over **directed edge states** `(node, entry_port)` — exactly the representation `GraphMap` already uses for its position:

From state "arriving at node N via port p", the successors are the states reached by LEFT/STRAIGHT/RIGHT via `LEFT_OF`/`OPPOSITE`/`RIGHT_OF`, restricted to ports that exist in `CITY[N]`. Run Dijkstra (or plain BFS if unweighted) on this state graph. Since the gate run is timed, weight each transition with an estimated duration: edge travel time plus a per-maneuver cost taken from `config.json` (`turn_durations` already tells you LEFT ≈ 2.4 s, STRAIGHT ≈ 2.0 s, RIGHT ≈ 1.2 s, plus the fixed 3 s stop). This naturally prefers routes with fewer and cheaper turns — the sheet explicitly asks for a good path, not just any path.

The gate route itself is a concatenation: current position → gate 1 edge → gate 2 edge → … in the required order. Each gate edge may be entered from either end (the tag is direction-independent), so for each leg take the cheaper of the two directed variants, and the exit state of one leg is the start state of the next. Because the gates are only known at runtime from the recorded map, this automatically satisfies "the path through the gates must not be hard-coded".

Output of the planner: an ordered list of `TurnDirection` commands, one per upcoming intersection, consumed by step 2 and re-derived after every completed move (replanning each intersection is cheap on this graph size and self-heals after localization corrections).

Acceptance: unit tests off-robot — given the real city dict, arbitrary start edge and gate assignment, the planner returns a turn sequence that a simulated `GraphMap.move()` walk executes to visit the gates in order without ever needing a U-turn.

## 6. Gate order and colors

Open point to clarify with the organizers, then implement: how is "the correct order" defined — by a given color sequence, by ascending tag id, or announced on site? Two pieces of work regardless of the answer:

Make the target sequence a runtime input (ROS param / config, e.g. `gate_order: [pink, green, blue]` or a list of tag ids) rather than anything baked into code. If the order is specified by *color* while detection yields *ids*, the id→color association must come from somewhere: either a provided mapping, or lightweight color detection during the mapping phase (sample the hue around the detected tag / the gate structure in the camera image when the tag is close, and pass it into the already-existing `record_gate(gate_id, color=…)` slot, which is currently always called without a color).

Also add a small guard against wrong-edge attribution: only record a gate when the tag detection area exceeds a threshold (the tag is genuinely close) and ideally not while in `CROSSING_INTERSECTION`, so a tag glimpsed across an intersection is not stamped onto the wrong edge.

## 7. Visualization of the chosen path + robustness

Dashboard/visualization: `/mapping/state` should additionally carry the planned route (as the list of upcoming directed edges / edge keys), and `visualization.py` should highlight it (e.g. blue dashed overlay with sequence numbers) alongside the already-working map display and live position marker. That completes requirements (a), (b) and (c) in one window. Optionally add the mission phase and the gate target sequence to the info box.

Robustness items, in descending priority: handle a failed `GraphMap.move()` (currently the position silently freezes) — at minimum stop and flag, better: attempt resync by matching the next observed sign/gate against the graph; remove the silent STRAIGHT fallback for missed signs during the gate run (the planner's command should be used, which also makes missed signs harmless, since per the sheet all direction information is derivable from the graph anyway); optionally extend `52DB.yaml` with entries for ids 5–13 so logs are readable.

## Suggested order of execution

Steps 1 and 2 first (small, unblock everything). Step 5 can be developed and unit-tested completely off-robot in parallel, since `GraphMap` is pure Python. Then step 4 (exploration = planner + "nearest unvisited edge" target selection), step 3 (phase switch), step 6 (order/colors, pending the organizer clarification), and step 7 last. The riskiest real-world item is not the algorithmics but the reliability of the open-loop crossings — every mis-executed turn desynchronizes the graph position — so budget test time on the track for turn-duration tuning and for the resync behavior.
