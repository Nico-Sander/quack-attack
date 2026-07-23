# ch2_intersection_handling

Challenge 2: read the intersection sign, stop at the red line, turn.

## Nodes

| Node | Does |
| --- | --- |
| `detect_lane_node.py` | U-Net segmentation (white/yellow/red) → lane error, stop-line distance and angle, which lines are visible |
| `detect_signs_node.py` | AprilTag (tagStandard52h13) detection → tag ID of the **largest** (= nearest) tag in frame |
| `detect_intersection_node.py` | Logic only: stop-line geometry → NO / APPROACHING / AT intersection |
| `switch_control_node.py` | State machine + turn choice: tag ID → sign type (`52DB.yaml`) → allowed directions → random pick, locked on approach |
| `control_wheels_node.py` | PID lane following, softer gains on approach, open-loop arc through the intersection |
| `dashboard_node.py` | Debug window: masks + detected tags over the camera feed (off by default) |

`crossing.py` (ROS-free, unit tested) decides when the crossing ends: lane re-acquired, not "the time is up". `custom_enums.py` holds the shared enums.

## Configuration

Tuning is in **`config/config.json`** (read once at node startup). The sign database is `src/52DB.yaml`.

### `control_wheels`
| Parameter | Description |
| --- | --- |
| `pid.p`, `pid.i`, `pid.d` | Lane-following gains. |
| `pid.p_slow`, `pid.d_slow` | Softer gains used while approaching the stop line. |
| `pid.d_filter_tau` | Low-pass time constant on the D term. |
| `pid.max_vel` | Forward speed while lane following, in m/s. |
| `approach_speed_multiplier` | Fraction of `max_vel` driven while approaching the stop line. |
| `intersection.straight_before_turn.v/omega` | Command used to roll into the intersection before the turn starts. |
| `intersection.durations.LEFT/RIGHT/STRAIGHT` | Seconds of that roll-in per direction (straight needs none). |
| `intersection.initial_turn.<DIR>.v/omega` | The open-loop arc driven for each turn direction. |
| `publish_rate` | Wheel-command publish rate in Hz. |

### `detect_lane`
| Parameter | Description |
| --- | --- |
| `lane_search_y_ratio` | Height of the horizontal search band, as a fraction from the top. |
| `crossing_search_y_ratio` | Search band while stopped/crossing — bottom of the image only. |
| `target_im_size` | Square input size the camera frame is resized to. |
| `hsv_red_lower1/upper1`, `hsv_red_lower2/upper2` | The two HSV bands (red wraps around hue 0) for the red mask. |
| `frame_skip` | Process one frame in N — the effective control rate. |
| `report_timing_every` | Seconds between timing log lines. |

### `detect_intersection`
| Parameter | Description |
| --- | --- |
| `stop_y_threshold` | How far down the image the red line must be (0–1) to count as reached. |

### `switch_control`
| Parameter | Description |
| --- | --- |
| `exit_confidence_mode` | Which lines end a crossing: `both` (safe default) or `yellow_only`. |
| `timers.stop_duration` | Seconds to stand still at the stop line. |
| `timers.min_blind_duration` | Minimum blind time before lane re-acquisition may end the crossing. |
| `timers.red_line_ignore_duration` | Seconds after a crossing during which red lines are ignored, so the same line cannot re-trigger. |
| `timers.turn_durations.LEFT/RIGHT/STRAIGHT` | **Timeouts**, not turn lengths — the crossing normally ends earlier. |
| `loop_rate` | State-machine loop rate in Hz. |

## Launch

```shell
roslaunch ch2_intersection_handling ch2_intersection_handling.launch
```

| Argument | Default | Description |
| --- | --- | --- |
| `veh` | `$VEHICLE_NAME` (else `default_robot`) | Robot name; all topics are namespaced under it. |
| `driving` | `true` | `false` runs perception and the state machine without moving. |
| `wait_for_signs` | `true` | Hold still until `detect_signs` is live (~5 s to build the 52h13 decode table). |
| `sign_cooldown` | `0.1` | Seconds before the same tag ID is published again. |
| `force_turn` | `NONE` | `LEFT`/`STRAIGHT`/`RIGHT` forces every turn and ignores the sign. |
| `dashboard` | `false` | Start the debug dashboard (needs X11). |

```shell
# bring-up without motors
roslaunch ch2_intersection_handling ch2_intersection_handling.launch driving:=false

# tune the turn manoeuvre without hunting for the right intersection
roslaunch ch2_intersection_handling ch2_intersection_handling.launch force_turn:=LEFT

# check the segmentation and which tag is being read
roslaunch ch2_intersection_handling ch2_intersection_handling.launch driving:=false dashboard:=true
```
