# Parameter Reference — ch4_mapping_pathfinding

Every knob in the package, in one place. For the race-day command sequence see
[`workflow.md`](workflow.md).

Three kinds:

1. **[Launch args](#1-launch-args)** — what you pass on the command line. Start here.
2. **[Config files](#2-config-files)** — the map, the gates, and the driving/timing constants.
3. **[Node params not exposed as launch args](#3-node-params-not-exposed-as-launch-args)** — defaults that are fine as-is.

Everything in section 1 is set like this:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch <arg>:=<value> ...
```

---

## 1. Launch args

### Mission

| Arg | Default | Values | What it does |
|---|---|---|---|
| `mission_phase` | `MAPPING` | `MAPPING`, `GATE_RUN` | Which half of the challenge to run. `MAPPING` explores every street; `GATE_RUN` drives the announced order. |
| `start_edge` | `A,4,D,2` | `from,port,to,port` | Where the bot is placed — **always at an intersection exit**. `A,4,D,2` = placed at A port 4, driving towards D port 2, so **D is the next intersection** and street `A4__D2` is driven in full. Must be a street the loaded city has, or the mapping node refuses to start. Changing this default means changing it in `mission_setup.default_config()` too — see the note there. |
| `gate_order` | *(empty)* | e.g. `7,5,11` | Gate tag IDs in the order announced on site. Can also be set after mapping, with `rosparam` — see [`workflow.md`](workflow.md). Passing it at launch makes the bot recommend a start position when mapping ends. |
| `gate_run_start_edge` | *(empty)* | `from,port,to,port` | Where the bot was placed for the gate run. Setting it also means "I moved it", so it drives off along the street instead of crossing. Empty = carry on from where mapping stopped. |
| `strict_gate_order` | `false` | `true`/`false` | See [the open question](#strict_gate_order--the-open-question) below. |
| `auto_start_gate_run` | `false` | `true`/`false` | Roll straight into the gate run when mapping finishes, instead of holding. Off by default because the bot is normally repositioned between the phases; mainly for simulation. |

### Debugging / bring-up

| Arg | Default | Values | What it does |
|---|---|---|---|
| `force_turn` | `NONE` | `NONE`, `LEFT`, `STRAIGHT`, `RIGHT` | Forces every turn, bypassing the planner. Use to verify the planner → switch_control command path end to end. |
| `driving` | `true` | `true`/`false` | `false` leaves out `control_wheels`, so nothing moves. Perception, mapping and planning still run — good for a bench test. |
| `wait_for_signs` | `true` | `true`/`false` | Holds the robot still until `detect_signs` reports it is detecting. **Leave this on.** `detect_signs` is the slowest node to come up — see [Startup order](#startup-order-and-why-the-bot-waits) — and without it the bot pulls away before gate detection is live. `false` only makes sense when `detect_signs` is not running at all. |
| `visualization` | `true` | `true`/`false` | The live map/path/position window. |
| `dashboard` | `false` | `true`/`false` | Perception debug dashboard (camera + segmentation masks). |

### Tuning

| Arg | Default | What it does |
|---|---|---|
| `gate_cooldown` | `2.0` s | Before the same gate tag is recorded again. Mostly redundant now that the map refuses overwrites; it still stops a re-seen gate re-triggering replans. |
| `command_timeout` | `1.0` s | How long a planner turn command stays trustworthy. The mapping node republishes at 10 Hz, so anything older means it stopped talking and `switch_control` falls back to its own choice. |
| `sign_cooldown` | `0.1` s | Per-tag detection cooldown in `detect_signs`. |

### Gate mapping

The three knobs that decide which gate lands on which street. See
[Which gate belongs to which street](#which-gate-belongs-to-which-street) for
why they exist.

| Arg | Default | What it does |
|---|---|---|
| `gate_min_area` | `-1` = use config | Overrides the tag-size threshold. The calibrated value is **`config/config.json` → `detect_signs.gate_min_area` = 1500 px²**; pass a positive value here only to sweep it. |
| `gate_confirm_frames` | `3` | Consecutive accepted sightings before a gate is written. At 10 Hz that is 0.3 s. |
| `map_gates_while_approaching` | `true` | Whether gates are recorded while approaching a stop line. Mapping is *always* off while stopped and while crossing. |

### Plumbing

| Arg | Default | What it does |
|---|---|---|
| `veh` | `$(env VEHICLE_NAME)` | Robot name; all topics live under `/<veh>`. |
| `city_path` | `config/city.json` | The map — the **challenge city** (9 intersections, 15 streets). Point this at `config/city_practice.json` for the small practice track, or at any other city file. |
| `gates_path` | `config/gates.json` | Gate tag ID → colour mapping. |

---

## 2. Config files

### `config/city.json` — the map

**The single source of truth.** Both the mapping node and the visualization read
it; changing the city means editing this file and nothing else.

It currently holds the **challenge city**: 9 intersections (`A`–`I`), 15
streets, transcribed from `config/challenge_graph_as_received.txt`. Note that
**A and B are joined by two separate streets** (`A1__B3` and `A2__B2`), so that
pair can carry two gates and a placement on it has to name the right one.
The old 3-node practice track lives on as `config/city_practice.json` and is
what the offline test suite runs against.

- `nodes`: `node → port → [neighbour, neighbour_port]`. Must be symmetric — if
  A port 1 leads to B port 3, then B port 3 must lead back to A port 1.
  Validated on startup; a mistake raises rather than misbehaving quietly.
- `layout`: optional `{node: [x, y]}` drawing hint. Without it the
  visualization falls back to a spring layout.

Port convention: ports 1–4 run cyclically around an intersection, so 1 is
opposite 3 and 2 is right of 1. Node names must contain **no digits** (edge keys
concatenate node and port, so `A1__B1` would otherwise be ambiguous).

### `config/gates.json` — gate colours

Cosmetic only. The gate order is announced as tag IDs, never derived from
colour; these just make logs and the visualization readable. IDs 5–13, each with
a `colour` name and a `hex` for drawing, plus `unknown_gate_hex` as fallback.

### `config/config.json` — driving and timing

Shared by the driving nodes **and the planner's cost model**, so tuning the
timings for driving automatically retunes route planning.

| Key | Default | Notes |
|---|---|---|
| `switch_control.exit_confidence_mode` | `"both"` | What counts as "back in a lane" and ends a crossing: `"both"` (white **and** yellow) or `"yellow_only"`. `"both"` is safer — mid-turn the camera often catches a single stray line. |
| `switch_control.timers.min_blind_duration` | `0.8` s | How long a crossing ignores lane markings at the start. The road being *left* is still in view for the first moment of a turn. |
| `switch_control.timers.stop_duration` | `3.0` s | Stop at the line. Also a planner cost term. |
| `switch_control.timers.turn_durations.{LEFT,STRAIGHT,RIGHT}` | `2.5 / 2.5 / 1.4` s | **A timeout, not a duration.** A crossing normally ends earlier, when lane markings are seen again. Also the planner's turn costs, which is why routes prefer right turns. |
| `switch_control.timers.red_line_ignore_duration` | `2.5` s | Ignore the just-crossed stop line so it does not retrigger. |
| `control_wheels.pid.*` | — | Lane-following PID. `p_slow`/`d_slow` apply while approaching a stop line and are deliberately set equal to `p`/`d`. |
| `control_wheels.pid.d_filter_tau` | `0.08` s | Low-pass time constant for the derivative term. The error is a noisy segmentation median; without this the D term is unusable at 30 Hz. Larger = smoother but laggier. |
| `control_wheels.intersection.durations.{LEFT,RIGHT,STRAIGHT}` | `0.40 / 0.35 / 0.0` s | Straight segment before the turn arc begins. |
| `control_wheels.intersection.initial_turn.{DIR}.{v,omega}` | — | The turn arc. Only needs to be roughly right: the crossing ends on seeing a lane, not on completing a precise arc. |
| `detect_intersection.stop_y_threshold` | `0.95` | How close the red line must be to count as *at* the intersection. |
| `detect_lane.lane_search_y_ratio` | `0.15` | How far ahead to look for lane lines while driving. |
| `detect_lane.crossing_search_y_ratio` | `0.99` | Search band while stopped or crossing — pinned to the bottom of the image, so the bot does not steer to roads on the far side of the intersection. |
| `detect_lane.frame_skip` | `1` | Process one camera frame in N. The camera is 30 Hz, so `1` gives a 30 Hz control loop — **this is the effective control rate**, since the PID runs once per frame published here. `2`/`3` drop it to 15/10 Hz if the CPU ever can't keep up. See the timing section below. |
| `detect_lane.report_timing_every` | `10.0` s | How often the lane pipeline logs how long a frame takes. `0` disables. |
| `control_wheels.publish_rate` | `30` Hz | How often wheel commands go out. Independent of the PID rate, which is driven by lane messages. |
| `switch_control.loop_rate` | `30` Hz | State machine tick. Bounds how late a crossing exit can be acted on. |
| `planner.turn_durations.{LEFT,STRAIGHT,RIGHT}` | `2.7 / 3.2 / 0.9` s | **Measured on the track.** Time from pulling away at the red line to being out of the intersection. Only a seed: once the mapping run has observed a turn, the observed time is used instead. Deliberately *not* the `switch_control` values, which are timeouts. |
| `planner.default_street_time` | `4.0` s | Charged for a street that has never been driven. After a mapping run every street has a real time and this stops mattering. |
| `detect_signs.gate_min_area` | `900` px² | **Measured from a rosbag.** Minimum detected tag area for a gate to be mapped onto the street the bot is driving; below it the tag is on another street. `detect_signs` reads this at startup — the launch arg of the same name only overrides it when set positive. See the calibration section below; 1400 was too high and missed a gate. |
| `detect_lane.hsv_red_*` | — | HSV gate on the model's red class. Pixels predicted red that are not red in HSV are discarded, which removes phantom stop lines. Widen if real stop lines are missed; tighten if the bot stops on open road. |

### Where the planner's numbers come from

A move costs `stop + turn + street`, and all three are measured off the same
drive-mode transitions, so they add up to the whole with no gap or overlap:

```
STOPPED --------> CROSSING ------> LANE_FOLLOWING -----------> STOPPED
        (turn starts)      (turn ends,          (arrived at the next
                            street starts)        red line)
        |<--- turn time --->|<------- street time ------------->|
```

Street times are measured during the mapping run and shown on the visualization
under each street. Turn times are measured too, and fall back to the config
seeds for any direction the run never performed (a five-street coverage route
can easily never turn left). Repeated measurements of one street are reduced
with the **median**, so a single disturbed lap does not distort the model.

The first street of a run is a special case: the bot is placed at an
intersection exit, so there is no crossing to start its clock, and node startup
is too early — `detect_lane` loads its network first and the bot stands still
for several seconds. That clock starts on the first wheel command that actually
moves it.

---

## 3. Node params not exposed as launch args

Reasonable defaults; override with `rosparam` or by adding a `<param>` if ever needed.

| Node | Param | Default |
|---|---|---|
| `detect_signs` | `~sign_db_path` | `src/52DB.yaml` |
| `detect_signs` | `~image_topic` | `/<veh>/camera_node/image/compressed` |
| `detect_signs` | `~output_topic` | `/<veh>/detect/sign` — closest tag ID, unthresholded, for `switch_control` |
| `detect_signs` | `~detections_topic` | `/<veh>/detect/sign_detections` — every tag with area, corners and the accept decision |
| `switch_control` | `~apriltags_db_path` | `src/52DB.yaml` |
| `mapping_visualization` | `~headless` | `false` — render without opening a window |
| `mapping_visualization` | `~snapshot_path` | `""` — also write every frame to this file |

`headless` + `snapshot_path` are how the map gets captured without a screen —
useful for a report, or for checking the layout over SSH:

```bash
rosparam set /$VEHICLE_NAME/mapping_visualization_node/headless true
rosparam set /$VEHICLE_NAME/mapping_visualization_node/snapshot_path /workspace/map.png
```

---

## Which gate belongs to which street

The camera sees further than one street. Approaching or stopped at an
intersection, gates on the streets *beyond* it are in frame, and taking one of
those at face value stamps it onto the street the bot is on — which was
observed on the track overwriting a gate that had already been mapped
correctly.

Four guards, applied in this order:

1. **Only while driving.** Gate mapping is off in `STOPPED` and
   `CROSSING_INTERSECTION`, where the camera is pointed across the
   intersection by definition. `APPROACHING_STOP_LINE` still counts by default
   (`map_gates_while_approaching`) — the bot is on its own street there, and a
   gate near the far end of it is legitimately its own.
2. **Big enough** (`gate_min_area`). Area falls off with the square of the
   distance, so it separates "on my street" from "over there" bluntly but very
   effectively.
3. **Seen repeatedly** (`gate_confirm_frames`). One frame can misdetect, and
   the write is permanent, so a single bad frame must not be able to cause it.
4. **First one wins, permanently.** A street keeps the first gate written to
   it, and a gate stays on the first street it was found on. Both refusals are
   logged (`Ignoring gate 9 on A1__B1: already mapped as gate 5`).

Guard 4 is the important one: the *first* sighting of a street's gate happens
while driving towards it, from closer up and better framed than any later
glimpse across an intersection. Later sightings can only be worse, so there is
nothing to gain from ever letting one win.

### Calibrating `gate_min_area`

**Set to 900 px²**, in `config/config.json` → `detect_signs.gate_min_area`.

This was first set to 1400 by eye on the dashboard, and the rosbag of the first
full mapping run (`run_0658.bag`) showed that was too high — one gate was
missed. The bag gives much better evidence than the dashboard does, because it
holds every sighting with its area and the street the bot was on at the time:

| | worst case in the run |
|---|---|
| Tag on **another** street, seen while driving | **501 px²** (tag 8 seen from `A2__C2`) |
| Tag on **this** street, first accepted sighting | **1426 px²** (gate 9 on `B3__C4`) |

1400 sat right at the top of that gap, so a gate only counted for the last
fraction of a second before the bot passed it. On the short streets that left
too few frames to confirm. 900 sits in the middle: still 1.8x above the worst
false positive, but it accepts a gate roughly a second earlier.

Redo this only if the camera, the gate size or the track changes — and prefer
a rosbag over the dashboard. Run with the dashboard:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch dashboard:=true driving:=false
```

Every detected tag is outlined on the camera feed with its ID and area:

- **green** — accepted, big enough to be mapped onto the current street
- **red** — below threshold, ignored

Carry the bot down a street towards its gate, and stand it at a stop line where
a gate on another street is visible. The gate ahead should be green well before
the bot reaches it; the one across the intersection should be red. Put the
threshold between the two areas you read off. Too high loses real gates; too
low is what caused the problem in the first place.

To sweep values without editing the config:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    dashboard:=true driving:=false gate_min_area:=2500
```

`detect_signs` logs which value it ended up with at startup, so there is never
any doubt about which one is in force.

---

## `strict_gate_order` — the open question

The requirements do not say whether **driving through a gate before it is due**
breaks the order. Both readings are defensible:

- **Lenient** (`strict_gate_order:=false`, the default) — only the order in
  which gates are *targeted* matters. Faster, since the route takes the
  shortest legal path.
- **Strict** (`strict_gate_order:=true`) — the route avoids gates that are not
  yet due. Costs time, and on a small city it is not always possible; when a
  detour does not exist the planner falls back to the direct route for that leg
  rather than refusing to plan.

**This needs an answer from the organizers.** The gate run is the only timed
part, so guessing wrong is either a time penalty or a rule violation. Both
behaviours are implemented and tested — it is a one-word change on the command
line once you know.

---

## Runtime control

The full sequence is in [`workflow.md`](workflow.md). In short:

```bash
N=/$VEHICLE_NAME/mapping_pathplanning_node

# Announce the order and where the bot has been placed, then start the run
rosparam set $N/gate_order          "7,5,6"
rosparam set $N/gate_run_start_edge "C,1,A,3"
rosservice call $N/start_gate_run

# What the planner is doing (map, timings, plan, recommendations)
rostopic echo /$VEHICLE_NAME/mapping/state

# The commanded turn: -1=hold at the line, 0=none, 1=LEFT, 2=STRAIGHT, 3=RIGHT
rostopic echo /$VEHICLE_NAME/plan/turn_command
```

`-1` (hold) is distinct from `0` (no opinion) on purpose: `0` hands the bot to
`switch_control`'s random fallback and it drives on, which is exactly what must
*not* happen while it is waiting to be picked up.


---

## Startup order, and why the bot waits

Two nodes have an expensive constructor, and **the slower one is not the one
you would guess**. Measured in the container (2026-07-22):

| Node | What it does before it can work | Cost |
|---|---|---|
| `detect_lane` | `import torch` (3.2 s) + load and JIT-trace the U-Net (1.2 s) | **~4.5 s** |
| `detect_signs` | build the `tagStandard52h13` quick-decode table | **~5.1 s alone, 5.7 s under launch contention** |

The AprilTag cost is not the detection — a `detect()` call on a frame is under
a millisecond. It is the one-off decode table: 48714 codes × 52 bits with 2-bit
error correction. It is not tunable from `pupil_apriltags`, so it is a fixed
part of every launch.

Motion has always been gated on `detect_lane` **by accident**: `control_wheels`
leaves `v` at 0 until the first `/detect/lane` message, which cannot arrive
before the network is loaded. Nothing gated it on `detect_signs`, so the bot
pulled away roughly a second before gate detection was live and drove ~0.2 m of
its first street blind. A gate already in frame at the placement could be
passed, or shrink out of the camera's view, in that window — see
[current-state.md](current-state.md#found-on-the-robot).

So `detect_signs` now publishes a **latched** `Bool` on
`/<veh>/detect/sign_ready`, and `control_wheels` holds every wheel command at
zero until it arrives (`wait_for_signs`, on by default). The flag means all
three of:

1. the detector is built,
2. a camera frame has actually been through it,
3. something is subscribed to `/detect/sign_detections` — the mapping node
   connects at its own pace, and a message published before that TCP connection
   is up is dropped, not queued.

Only the third implies the other two, which is why the flag waits for it. The
release is one-way: a later dropped message must not re-freeze a bot that is
mid-crossing.

What you should see in the log, in this order:

```
detect_signs started (52h13, 13 tags, detector built in 5.7s, ...)
Holding still: waiting for detect_signs to come up          # control_wheels
detect_signs ready after 6.1s -- gate detection is live
Sign detection is live -- releasing the wheels
Run started on A1__B1 -- timing from here                    # mapping node
```

If the bot never moves, that first warning is the one to look for: it names the
node it is waiting for. The mission clock is unaffected — it starts on the
first non-zero wheel command, which now cannot happen too early.

---

## Timing and latency

Measured on the laptop container against the real robot (July 2026):

| Stage | Measured |
|---|---|
| Camera publish rate | 30 Hz, ~40 KB/frame, 1.1 MB/s |
| Camera frame age on arrival | ~39 ms |
| Lane pipeline (decode → inference → masks) | **20 ms** (inference 16 ms of it) |
| Pipeline capability | ~49 Hz |
| `/detect/lane` output | **30 Hz** (`frame_skip: 1`), confirmed live |
| `switch/mode`, wheel commands | 30 Hz |

**Inference is not a bottleneck** — 20 ms against a 33 ms budget at 30 Hz, and
the node runs offboard on the laptop's 12 cores. It logs this periodically and
warns if a frame ever exceeds the budget, which is the symptom to watch for if
driving degrades:

```
lane pipeline 20 ms/frame (budget 33 ms at 1-in-1)
```

End-to-end, a steering decision is based on an image roughly **75 ms old** on
average (39 transit + 20 compute + ~17 command publish). At 0.19 m/s that is
about 1.4 cm of travel.

### Why 30 Hz

The control rate is set by `frame_skip`, **not** by `publish_rate`: the PID runs
once per lane message, so publishing wheel commands faster than the PID updates
just repeats the same value. `frame_skip: 1` is what actually gives 30 Hz
steering — 3× more frequent corrections and ~65 ms less latency than the old
10 Hz, which matters most on tight curves.

The catch is the derivative term. The error is a quantised segmentation median,
and differentiating it at 30 Hz amplifies that noise. So the D term is low-pass
filtered (`pid.d_filter_tau`), which both makes it usable at 30 Hz and makes the
loop rate-robust — `frame_skip` can be changed without the steering blowing up.
Tune the PID (Step 3) at this rate, since it is the rate you will race at. If
the CPU is ever contended, `frame_skip: 2` drops cleanly to 15 Hz.
