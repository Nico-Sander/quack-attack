# Test Plan — what to run, in order

Each step isolates one failure mode and assumes the previous one passed. Steps
0–1 need no robot. Parameters are documented in [`parameters.md`](parameters.md).

Record a rosbag for every step from 3 onwards — see [Rosbags](#rosbags) at the
bottom for the exact command and what I can get out of one.

---

## Step 0 — Rebuild and smoke test *(no robot, ~5 min)*

The image gained explicit pip dependencies (`networkx`, `matplotlib`,
`pupil-apriltags`, `PyYAML`). `pupil-apriltags` was previously hand-installed in
the running container and would have vanished on the next rebuild.

```bash
docker compose build duckierace_env # <- passed
./start.sh <vehicle> # <- passed
./attach_tmux.sh # <-passed
```

In a pane:

```bash
python3 -m pytest /workspace/src/packages/ch4_mapping_pathfinding/tests -q # <-passed
python3 -c "import networkx, matplotlib, yaml, pupil_apriltags; print('deps ok')" # <-passed
```

**Pass:** 69 tests pass, imports succeed. <-passed

---

## Step 1 — ROS simulation *(no robot, ~5 min)*

Runs the whole mission over real ROS with a fake driver. Catches anything wrong
in the ROS layer before the robot is involved.

> **Use a private roscore.** The container's `ROS_MASTER_URI` points at the
> robot, and the fake driver publishes `/switch/mode` — on the real master that
> would command the actual Duckiebot.

```bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=127.0.0.1
roscore -p 11312 &
sleep 3

roslaunch ch4_mapping_pathfinding simulate.launch \
    gate_order:=7,5,6 auto_start_gate_run:=true step_time:=0.4
```

**Pass:** the visualization window shows the bot exploring every street, gates
appearing on streets, then the blue dashed path during the gate run, ending in
`Gate run complete`. Every move should say `(planner)`, never `(random)`, and
there should be **no WARN or ERROR lines**.
-> passed

The launch stays alive after the mission finishes so the final map and route
remain on screen — Ctrl-C when you have finished looking.

Expect this INFO line near the phase switch:

```
Every street driven; waiting for gate(s) [7] to come into view before starting the run
```
-> passed

That is normal and usually lasts a fraction of a second. Coverage completes the
moment the last street is *entered*, but its gate is only seen while driving
it, so the run correctly waits rather than planning against an incomplete map.
If it persists it escalates to a WARN naming the gates — that one *is* worth
investigating (gate not on the track, or an ID announced wrongly).
-> passed

Worth also running `obey_planner:=false` — the fake driver then turns at random
and the mapping node must still track position correctly. That simulates
mis-executed turns, which is the failure mode most likely on the real track.
Expect `Localization lost` here: that is the point of the run. Note the planner
then stands down for good, since `localization_ok` never recovers — the resync
work in `current-state.md` §7 is what would fix that.
-> passed, as expected

---

## Step 2 — Bench test with the real robot *(robot powered, wheels off the ground, ~10 min)*

First contact with real perception. **No motors.**

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch driving:=false
```

Carry the bot around the track by hand, or hold tags in front of the camera.

**First check the namespace.** The startup line now reports it:

```
mapping_pathplanning_node started in MAPPING on edge A1__B1, namespace /donald
```

If it says `/default_robot`, `VEHICLE_NAME` was not set and nothing will reach
the robot — start the container with `./start.sh <vehicle>`. Note `.env` still
carries `VEHICLE_NAME=trick`; `start.sh` overrides it, but a plain
`docker compose up` would silently namespace everything under `/trick`.
-> passed

**Check:**
- `rostopic echo /$VEHICLE_NAME/detect/sign` — intersection tags 1–4 and gate tags 5–13 are detected
-> passed
- `rostopic echo /$VEHICLE_NAME/plan/turn_command` — publishing steadily (1/2/3, not 0)
-> passed
- the visualization window renders the map with A/B/C in a triangle
-> map is drawn, but layout is funky, streets are crossing each other, does not reflect the actual layout

**Pass:** tags detected, turn command non-zero, map drawn.
-> passed

---

## Step 3 — Lane following first *(driving, ~20 min)*

**The approach phase is no longer special.** Red-line squaring is gone, the
approach uses the same PID gains as normal driving and only slows down a
little, and the crossing now ends when the bot sees a lane again rather than
when a timer expires. So there is much less to tune, and what is left is
ordinary lane following.

Before tuning, confirm the pipeline is keeping up. `detect_lane` logs this
every 10 s:

```
lane pipeline 20 ms/frame (budget 33 ms at 1-in-1)
```

A WARN here (`falling behind, images are stale`) means every steering decision
is being made on an old image, and no amount of PID tuning will fix that. See
the timing section in [`parameters.md`](parameters.md).

Tune lane following on open road, away from intersections:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch force_turn:=STRAIGHT
```

| Symptom | Knob (`config/config.json` → `control_wheels`) |
|---|---|
| Weaves / oscillates | lower `pid.p`, raise `pid.d` |
| Cuts corners, slow to re-centre | raise `pid.p` |
| Too fast to control | lower `pid.max_vel` |
| Overshoots the stop line | lower `approach_speed_multiplier` |

`pid.p_slow`/`d_slow` are deliberately set equal to `p`/`d`. Keep them equal
unless there is a clear reason: separate approach gains were what made the
approach behave differently from the rest of driving in the first place.

**Pass:** the bot holds its lane for a full lap and stops at red lines without
overshooting. Being square at the line no longer matters much — the next step
explains why.

---

## Step 3b — Crossings *(driving, ~15 min)*

Crossings are now closed loop: the arc runs until **lane markings are seen
again**, with `turn_durations` demoted to a timeout.

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch force_turn:=RIGHT
# then STRAIGHT, then LEFT
```

Watch the log. Every crossing prints why it ended:

```
Crossing done after 1.42s (lane_reacquired)     <- good
Crossing timed out after 2.50s without re-acquiring a lane   <- bad
```

**Pass: crossings end with `lane_reacquired`, not `timeout`.** Consistent
timeouts mean the exit detection is not working and the crossing has silently
gone back to being open loop — the exact fragility this change removes.

| Symptom | Knob |
|---|---|
| Always `timeout` | check `/detect/lane_borders` reports `white_detected`/`yellow_detected` at all |
| Exits mid-turn, too early | raise `timers.min_blind_duration` |
| Exits into the wrong lane | keep `exit_confidence_mode: "both"` |
| Never completes the turn before timing out | raise `turn_durations.<DIR>` (it is only a ceiling) |
| Turn arc badly wrong | `intersection.initial_turn.<DIR>.omega` |

The arc only needs to be roughly right now — close enough that the bot ends up
somewhere a lane is visible. Do not spend time perfecting it.

> `turn_durations` also feeds the planner's cost model. As a timeout it is an
> upper bound rather than the real crossing time, so route time estimates will
> read high. Ordering (RIGHT cheaper than LEFT) is what the planner actually
> uses, so this is harmless.

---

## Step 4 — Command path *(driving, ~10 min)*

Plan step 2's acceptance criterion.

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch force_turn:=LEFT
```

**Pass:** the bot turns left at every intersection where left exists. Logs show
`Turning LEFT (planner)`, never `falling back to random`.

---

## Step 4b — Calibrate the gate distance threshold *(no driving, ~10 min)*

> **Done: 1500 px²**, stored in `config/config.json` →
> `detect_signs.gate_min_area`. Nothing to do here unless the camera, the gates
> or the track change. The rest of this section is the procedure, kept for that
> case.

The bot must only map gates that are on the street it is driving, and the test
for that is how big the tag looks.

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    dashboard:=true driving:=false
```

Every detected tag is outlined on the camera feed with its ID and pixel area:
**green** = accepted and mappable, **red** = too far away, ignored.

Two readings, by hand:

1. Stand the bot on a street, pointed at that street's gate from a normal
   driving distance. Note the area. **Should be green.**
2. Stand it at a stop line where a gate on another street is visible across the
   intersection. Note that area. **Should be red.**

If the split is wrong, sweep the threshold without editing anything:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    dashboard:=true driving:=false gate_min_area:=<value>
```

Once a value works, write it into `config/config.json` →
`detect_signs.gate_min_area` so it applies without the arg.

**Pass:** the gate on the current street goes green well before the bot reaches
it, and gates across an intersection stay red. If no single number separates
them, say so — the fallback is `map_gates_while_approaching:=false`, which stops
mapping as soon as a stop line is in sight.

---

## Step 5 — Mapping run *(driving, ~15 min)*

The real thing, untimed.

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch start_edge:=A,4,D,2
```

Set `start_edge` to where the bot actually is: `A,4,D,2` means *driving from A
port 4 towards D port 2*, so D is the next intersection.

Place the bot **at the exit of A port 4**, not part-way down the street — the
street has to be driven in full for its time to mean anything.

**Pass:** all 15 streets turn green in the visualization, each showing a
measured time in seconds; every gate appears on the street it is physically on;
the `MAPPING COMPLETE` summary block in the log; and the bot **stops at the red
line and stays there**. Expect ~16 crossings from `A,4,D,2` on the challenge
city (~5 on the practice track, from `A,1,B,1` with
`city_path:=…/config/city_practice.json`).

Check the measured times against a stopwatch on one street. They should also be
plausible relative to each other — a street twice as long should read roughly
twice as long. A street reading far too short means a spurious stop-line
detection; far too long means a missed one.

**Watch for:** a gate landing on the wrong street, which is now the main
remaining mapping risk. The log tells you which way it went wrong:

```
Gate 7 (blue) recorded on A3__C1 (area 6820 px)          <- a street learned its gate
Ignoring gate 9 on A3__C1: already mapped as gate 7      <- a wrong sighting refused
```

A stream of `Ignoring …` lines is the guard doing its job. But if the gate that
*stuck* is on the wrong street, the first sighting was already wrong — raise
`gate_min_area` (step 4b) rather than assuming the rest will sort itself out,
because nothing overwrites a mapped gate afterwards.

If a gate is never recorded at all, the threshold is too high: watch the
dashboard for a gate that stays red all the way down its street.

**Count the gates against the streets before moving on.** The `MAPPING
COMPLETE` block lists them; a street with no gate is either a street with no
gate on it, or a gate that was missed — and the map cannot tell you which. This
has already happened once (`current-state.md` §4). The bag settles it:

```bash
python3 - <<'PY'
import json, rosbag
bag = rosbag.Bag("run_XXXX.bag"); seen = {}
for _t, msg, _ in bag.read_messages(topics=["/<veh>/detect/sign_detections"]):
    for d in json.loads(msg.data)["detections"]:
        e = seen.setdefault(d["tag_id"], [0, 0, 0.0])
        e[0] += 1; e[1] += d["accepted"]; e[2] = max(e[2], d["area"])
for k, (n, acc, area) in sorted(seen.items()):
    print(f"tag {k:<3} seen {n:4d}  accepted {acc:4d}  max area {area:.0f}")
PY
```

A tag with a large max area but few accepted sightings is a gate that nearly
made it — lower `gate_min_area` until it clears the confirmation streak.

---

## Step 6 — Gate run *(driving, timed, ~10 min)*

**Leave the mapping launch running** — the map, gates and timings live in that
node. Pick the bot up, place it at an intersection exit (the log recommends
one), then give it the order and the placement. Full command sequence in
[`workflow.md`](workflow.md):

```bash
N=/$VEHICLE_NAME/mapping_pathplanning_node
rosparam set $N/gate_order          "7,5,6"
rosparam set $N/gate_run_start_edge "C,1,A,3"
rosservice call $N/start_gate_run
```

Or from a cold start with a known map:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    mission_phase:=GATE_RUN start_edge:=C,4,F,2 gate_order:=7,5,6
```

**Pass:** the bot drives off along its street (it must **not** perform a turn
manoeuvre — it is no longer at an intersection), the gates are driven in the
announced order, the blue dashed path is visible, and it finishes with
`Gate run complete` and holds.

Record the wall-clock time and compare against the estimate in the service
reply. After a mapping run the cost model is built from measured times, so the
two should now be close — a large gap points at the cost model rather than at
the driving.

Worth doing twice: once from the recommended placement, once from a different
one, to confirm the recommendation is actually the faster of the two.

---

## Rosbags

Record from step 3 onwards. This is small enough to keep for every run:

```bash
rosbag record -O run_$(date +%H%M).bag \
  /$VEHICLE_NAME/mapping/state \
  /$VEHICLE_NAME/plan/turn_command \
  /$VEHICLE_NAME/switch/mode \
  /$VEHICLE_NAME/switch/turn_direction \
  /$VEHICLE_NAME/detect/sign \
  /$VEHICLE_NAME/detect/sign_detections \
  /$VEHICLE_NAME/detect/intersection \
  /$VEHICLE_NAME/detect/lane \
  /$VEHICLE_NAME/detect/lane_borders \
  /$VEHICLE_NAME/lane_controller_node/car_cmd
```

Add `/$VEHICLE_NAME/camera_node/image/compressed` when debugging perception —
but it dominates the file size, so leave it out for routine runs.

From a bag I can reconstruct: every turn commanded vs. executed, where the
believed position diverged from reality and at exactly which crossing, which
gate was recorded on which street and how long the tag was visible first, and
how the real crossing durations compare to the configured ones. That last one
turns step 3 from trial-and-error into measurement.

**If a run goes wrong, the bag plus the console log is enough for me to work
from — no need to reproduce it live.**
