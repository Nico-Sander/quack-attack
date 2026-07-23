# ch3_obstacle_avoidance

Challenge 3: follow the lane and steer around duckies. Purely reactive follow-the-gap — camera and time only, no mapping or odometry memory.

## Nodes

| Node | Does |
| --- | --- |
| `detect_lane_node.py` | U-Net segmentation → lane error (`/detect/lane`) and left/right line positions (`/detect/lane_borders`); flags head-on lines |
| `detect_obstacle_node.py` | YOLOv11 duckie detection → bounding boxes on `/detect/duckie_BB` |
| `control_lane_node.py` | Follow-the-gap controller, FSM CRUISE / AVOID / ESCAPE_ROTATE; **the only node that moves the robot** |
| `dashboard_node.py` | Debug window: camera, masks and the planner overlay (blocked/free spans, chosen gap, FSM state) |

Startup safety: the controller holds still until it has seen `/detect/lane`, `/detect/lane_borders` **and** `/detect/duckie_BB` at least once — an absent detector is indistinguishable from "road clear". The robot is never commanded `v == 0 and omega == 0`: with no forward path it rotates in place instead of freezing.

Unlike ch1/ch2/ch4 these nodes are **not** launched inside a `veh` namespace; they build their topics from `$VEHICLE_NAME` themselves.

## Configuration

Two JSON files under **`config/`**, read at node startup. `control_lane_node` additionally exposes every parameter over **dynamic_reconfigure** (`cfg/ControlLane.cfg`) — use `rosrun rqt_reconfigure rqt_reconfigure` to tune live, then write the value back into the JSON to make it permanent.

### `config/control_lane_node.json` → `parameters.controller`
| Parameter | Description |
| --- | --- |
| `v_cruise` | Forward speed with a clear road. |
| `v_avoid` | Forward speed while steering around a duckie. |
| `v_min` | Creep floor, just above the friction stall limit. |
| `k_steer` | Proportional steering gain — this is the lane P-gain. |
| `omega_rotate` | In-place rotation rate in ESCAPE_ROTATE; must break static friction. |
| `omega_max` | Hard cap on `|omega|` for any command. |
| `lane_margin` | Hard inset from each detected line, defining the steerable corridor. |
| `react_ymax` | How close a duckie must be before it counts at all — the "reacts too early" knob. |
| `front_slow_ymax` | Front-duckie closeness where speed starts ramping down to `v_min` (keep above `react_ymax`). |
| `front_block_ymax` | Front-duckie closeness that counts as blocked → ESCAPE_ROTATE. |
| `escape_min_dwell` | Minimum seconds held in ESCAPE_ROTATE (anti-chatter). |
| `avoid_min_dwell` | Seconds AVOID is held once committed to a gap — the "drive through it confidently" knob. |
| `escape_relax_after` | Seconds of fruitless rotation before gap requirements are progressively relaxed. |
| `lane_hold_frames` | Invalid border readings before a line is treated as gone — this is line memory. |
| `duckie_hold_time` | Seconds a duckie's span survives after detections stop (above ~1.5 expect phantoms). |
| `yellow_anchor` | Assumed yellow-line x when only white is visible; sets the U-turn radius. |
| `white_anchor` | Assumed white-line x when only yellow is visible. |
| `camera_width_k` | **Measured**, not tuned: cm → normalized image width at range. |
| `robot_width_cm` | **Measured**, not tuned: robot width at its widest point. |
| `safety_margin_cm` | Clearance left when passing an obstacle — the real tuning knob. |
| `gap_min_cm` | Extra lateral slack the robot's centre needs before a gap counts as passable. |

### `config/detect_obstacle_node.json` → `parameters`
| Parameter | Description |
| --- | --- |
| `model.confidence_threshold` | Minimum YOLO confidence for a detection to be published. |
| `model.process_every_n_frames` | Run inference on one frame in N. |
| `model.input_size` | YOLO input resolution in pixels. |
| `obstacle_region.x_min`, `x_max`, `y_min` | Fallback image region searched for duckies when no lane borders are available. |

`detect_lane_node.py` has no JSON file; its constants (`lane_search_y_ratio`, `head_on_spread`, duckie-mask hold/margin) sit at the top of the node.

## Launch

```shell
roslaunch ch3_obstacle_avoidance ch3_obstacle_avoidance.launch
```

| Argument | Default | Description |
| --- | --- | --- |
| `lane` | `true` | Start lane segmentation. |
| `detector` | `true` | Start YOLO duckie detection. |
| `controller` | `true` | `false` = dry run: the node still plans and feeds the dashboard, but publishes no drive command. |
| `controller_node` | `true` | Start `control_lane_node` at all — rarely changed, prefer `controller:=false`. |
| `dashboard` | `true` | Camera debug dashboard (needs X11). |

```shell
# tuning / dry run: full stack, dashboard fully populated, robot stationary
roslaunch ch3_obstacle_avoidance ch3_obstacle_avoidance.launch controller:=false
```

Longer write-ups (design, parameters, debugging) are in `../docs/`.
