# ch1_lane_following

Challenge 1: follow the lane, stop 3 s at every red line, drive on.

## Nodes

| Node | Does |
| --- | --- |
| `detect_lane_node.py` | U-Net segmentation (white/yellow/red) → lane centre, cross-track error, red-line geometry |
| `detect_intersection_node.py` | Logic only, no images: stop-line geometry → `at stop line` + `red line visible` |
| `switch_control_node.py` | State machine: LANE_FOLLOWING → AT_STOP_LINE → CROSSING → CROSSING_CLEARING |
| `control_lane_node.py` | PID on the lane error → wheel command; zeros while stopped, blind command while crossing |
| `dashboard_node.py` | Debug window: segmentation masks next to the raw camera feed (off by default) |

`custom_enums.py` holds the shared `DriveMode`; the model lives in `models/`.

## Configuration

All tuning is in **`config/config.json`** (read once at node startup — restart the node after editing).

### `control_lane`
| Parameter | Description |
| --- | --- |
| `pid.p`, `pid.i`, `pid.d` | Gains turning the lane error into a turn rate. |
| `pid.d_filter_tau` | Low-pass time constant on the D term, so the rate stays stable across loop rates. |
| `pid.max_vel` | Forward speed while lane following, in m/s. |
| `crossing.v`, `crossing.omega` | Blind command driven during CROSSING; `omega` is measured drift compensation, not zero. |
| `publish_rate` | Wheel-command publish rate in Hz. |

### `detect_lane`
| Parameter | Description |
| --- | --- |
| `lane_search_y_ratio` | Height of the horizontal search band in the image, as a fraction from the top. |
| `crossing_search_y_ratio` | Search band used while stopped/crossing — bottom of the image only, so the far side of the line is ignored. |
| `target_im_size` | Square input size the camera frame is resized to for the network. |
| `hsv_red_lower1/upper1`, `hsv_red_lower2/upper2` | The two HSV bands (red wraps around hue 0) for the red mask. |
| `frame_skip` | Process one frame in N — this is the effective control rate (30 Hz camera). |
| `report_timing_every` | Seconds between per-frame timing log lines. |

### `detect_intersection`
| Parameter | Description |
| --- | --- |
| `stop_y_threshold` | How far down the image the red line must be (0–1) before the robot stops for it. |

### `switch_control`
| Parameter | Description |
| --- | --- |
| `timers.stop_duration` | Seconds to stand still at the stop line. |
| `timers.crossing_duration` | Seconds of blind driving over the line (~22 cm at 0.18 m/s). |
| `timers.min_clearing_duration` | Minimum clearing time, covering stale perception messages about the line just left. |
| `timers.max_clearing_duration` | Give-up timeout if red never disappears from the image. |
| `loop_rate` | State-machine loop rate in Hz. |

## Launch

```shell
roslaunch ch1_lane_following ch1_lane_following.launch
```

| Argument | Default | Description |
| --- | --- | --- |
| `veh` | `$VEHICLE_NAME` (else `default_robot`) | Robot name; all topics are namespaced under it. |
| `driving` | `true` | `false` runs perception and the state machine without moving the robot. |
| `dashboard` | `false` | Start the debug dashboard (needs X11). |

```shell
# bring-up without motors: hold the bot over a red line to walk all four states
roslaunch ch1_lane_following ch1_lane_following.launch driving:=false

# check the segmentation, e.g. when the red mask causes phantom stops
roslaunch ch1_lane_following ch1_lane_following.launch driving:=false dashboard:=true
```
