# ch4_mapping_pathfinding

Challenge 4: explore the whole city and record every gate (MAPPING, untimed), then drive the announced gate order (GATE_RUN, timed).

## Nodes

| Node | Does |
| --- | --- |
| `detect_lane.py` | U-Net segmentation → lane error, stop-line geometry, which lines are visible |
| `detect_intersection.py` | Logic only: stop-line geometry → NO / APPROACHING / AT intersection |
| `detect_signs.py` | AprilTag detection → nearest tag ID; area-gated so gates on other streets are rejected |
| `switch_control.py` | State machine; obeys the planner's turn command, falls back to its own choice if the planner goes silent |
| `control_wheels.py` | PID lane following, slow approach, open-loop arc through the intersection |
| `mapping_pathplanning.py` | Owns the mission: graph dead-reckoning, gate recording, replanning after every move; serves `~start_gate_run` |
| `visualization.py` | Live map/path/position window |
| `dashboard.py` | Debug window: camera, masks, detected tags (off by default) |

ROS-free, unit-tested libraries: `city_map.py` (graph + live map state), `planner.py` (Dijkstra over directed `(node, entry_port)` states, so U-turns are inexpressible), `run_timing.py` (measures real street/turn times during mapping and feeds them to the cost model), `gate_detection.py` (size + streak guards before a gate is written permanently), `crossing.py`, `graph_layout.py`, `mission_setup.py`.

`mission_tui.py` is a menu-driven front end that keeps **one** roslaunch alive across both phases (restarting would throw away the map).

## Configuration

### `config/config.json` — tuning, read at node startup
| Section / parameter | Description |
| --- | --- |
| `control_wheels.pid.p/i/d`, `max_vel` | Lane-following gains and forward speed. |
| `control_wheels.pid.p_slow/d_slow` | Softer gains used while approaching the stop line. |
| `control_wheels.pid.d_filter_tau` | Low-pass time constant on the D term. |
| `control_wheels.approach_speed_multiplier` | Fraction of `max_vel` driven on approach. |
| `control_wheels.intersection.straight_before_turn.v/omega` | Roll-in command before the turn starts. |
| `control_wheels.intersection.durations.<DIR>` | Seconds of that roll-in per direction. |
| `control_wheels.intersection.initial_turn.<DIR>.v/omega` | The open-loop arc per turn direction. |
| `control_wheels.publish_rate` | Wheel-command publish rate in Hz. |
| `detect_intersection.stop_y_threshold` | How far down the image the red line must be to count as reached. |
| `detect_lane.lane_search_y_ratio` | Height of the horizontal search band, as a fraction from the top. |
| `detect_lane.crossing_search_y_ratio` | Search band while stopped/crossing — bottom of the image only. |
| `detect_lane.target_im_size` | Square input size the camera frame is resized to. |
| `detect_lane.hsv_red_*` | The two HSV bands (red wraps around hue 0) for the red mask. |
| `detect_lane.frame_skip` | Process one frame in N — the effective control rate. |
| `detect_lane.report_timing_every` | Seconds between timing log lines. |
| `detect_signs.gate_min_area` | Minimum tag area in px for a gate to count as being on the current street (calibrated; the launch arg overrides it). |
| `planner.turn_durations.<DIR>` | **Measured** turn times used as route costs — deliberately not the crossing timeouts below. |
| `planner.default_street_time` | Assumed seconds per street until the mapping run measures the real one. |
| `switch_control.exit_confidence_mode` | Which lines end a crossing: `both` or `yellow_only`. |
| `switch_control.timers.stop_duration` | Seconds to stand still at the stop line. |
| `switch_control.timers.min_blind_duration` | Minimum blind time before lane re-acquisition may end a crossing. |
| `switch_control.timers.red_line_ignore_duration` | Seconds after a crossing during which red lines are ignored. |
| `switch_control.timers.turn_durations.<DIR>` | Crossing **timeouts**, not turn lengths. |
| `switch_control.loop_rate` | State-machine loop rate in Hz. |

### Other files in `config/`
| File | Description |
| --- | --- |
| `city.json` | The city graph (9 intersections, 15 streets) — single source of truth for every node. |
| `city_practice.json` | The small practice track; pass via `city_path:=`. |
| `gates.json` | Gate tag ID → colour, cosmetic (logs and display only). |

`control_lane_node.json` and `detect_lane.json` are leftovers from the parameter-server workflow and are not read by any node in this package.

## Launch

```shell
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch
```

**General**

| Argument | Default | Description |
| --- | --- | --- |
| `veh` | `$VEHICLE_NAME` (else `default_robot`) | Robot name; all topics are namespaced under it. |
| `city_path` | `config/city.json` | The city graph to use. |
| `gates_path` | `config/gates.json` | Gate tag ID → colour mapping. |

**Mission**

| Argument | Default | Description |
| --- | --- | --- |
| `mission_phase` | `MAPPING` | `MAPPING` (drive every street) or `GATE_RUN` (timed run). |
| `start_edge` | `A,4,D,2` | Where the bot is placed, as `from_node,from_port,to_node,to_port`. |
| `gate_run_start_edge` | *(empty)* | Different start street for the gate run; empty = continue where mapping ended. |
| `gate_order` | *(empty)* | The gate tag IDs in the announced order, e.g. `7,5,11`. |
| `strict_gate_order` | `false` | Route around gates that are not due yet, when possible. |
| `auto_start_gate_run` | `false` | Start the gate run automatically once every street has been driven. |

**Tuning**

| Argument | Default | Description |
| --- | --- | --- |
| `gate_cooldown` | `2.0` | Seconds before the same gate tag is recorded again. |
| `command_timeout` | `1.0` | How long a planner turn command stays trustworthy (republished at 10 Hz). |
| `sign_cooldown` | `0.1` | Per-tag detection cooldown in `detect_signs`. |
| `gate_min_area` | `-1` | Override the gate area threshold in px; `-1` = use `config.json`. |
| `gate_confirm_frames` | `3` | Consecutive sightings before a gate is written to the map (writes are permanent). |
| `map_gates_while_approaching` | `true` | Also record gates while approaching a stop line. |

**Debugging**

| Argument | Default | Description |
| --- | --- | --- |
| `force_turn` | `NONE` | `LEFT`/`STRAIGHT`/`RIGHT` forces every turn and bypasses the planner. |
| `driving` | `true` | `false` runs perception, mapping and planning without moving. |
| `wait_for_signs` | `true` | Hold still until `detect_signs` is live (~5 s to build the 52h13 decode table). |
| `visualization` | `true` | Live map/path/position window. |
| `dashboard` | `false` | Perception debug dashboard (needs X11). |

```shell
# mapping phase
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch start_edge:=A,4,D,2

# timed gate run, order announced on site as tag IDs
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    mission_phase:=GATE_RUN start_edge:=C,4,F,2 gate_order:=7,5,11

# start the gate run on a running stack instead of relaunching
rosservice call /$VEHICLE_NAME/mapping_pathplanning_node/start_gate_run
```

### `simulate.launch` — full mission without the robot

Runs the mapping node, the visualization and a fake driver that obeys `/plan/turn_command`. No camera, no motors, no Duckiebot — only a roscore.

```shell
roslaunch ch4_mapping_pathfinding simulate.launch
roslaunch ch4_mapping_pathfinding simulate.launch mission_phase:=GATE_RUN gate_order:=7,5,6
```

Takes the mission arguments plus `gates` (edge:tag pairs to place), `step_time`, `moves`, `obey_planner` and `halt_timeout`.

### Menu-driven front end

```shell
rosrun ch4_mapping_pathfinding mission_tui.py
```

The manual equivalent of both phases is in `../docs/workflow.md`.
