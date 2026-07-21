# Current State — mapping_pathfinding

Living document. Reflects the package as of **2026-07-21**.

Related: **[`test-plan.md`](test-plan.md) — what to run on the robot, in order**,
**[`parameters.md`](parameters.md) — every launch arg and config knob in one
place**, [`plan-fable.md`](plan-fable.md) (the work plan being executed),
[`current_implementation-fable-analysis.md`](current_implementation-fable-analysis.md)
(architecture analysis of the state *before* this work — now partly outdated).

---

## 1. Where we are

| Plan step | Status | Note |
|---|---|---|
| 1. Single source of truth for the map | **Done** | `config/city.json`, loaded by every node |
| 2. Planner commands turns | **Done** | `/plan/turn_command`, untested on the robot |
| 3. Two mission phases | **Done** | `MAPPING` / `GATE_RUN` / `DONE` |
| 4. Systematic exploration | **Done** | Greedy nearest-unvisited-street |
| 5. Path planning, no U-turns | **Done** | Dijkstra over `(node, entry_port)` |
| 6. Gate order and colours | **Done** | Order = announced tag IDs; colours cosmetic |
| 7. Path visualization + robustness | **Mostly done** | Path drawn; resync beyond "flag and stand down" still open |

**209 tests pass.** Steps 0-5 of [`test-plan.md`](test-plan.md) are done on the
robot: **lane following, intersection driving and the full mapping run all
work**, with every gate mapped to the right street. What is left is the timed
gate run. Track testing has found three bugs the offline tests could not (see
*Found on the robot* below).

**[`workflow.md`](workflow.md) is the race-day command sequence.**

---

## 2. Architecture

The control loop is now closed — the mapping node plans and commands, where it
used to only observe:

```
camera ─┬─► detect_lane.py ──► /detect/lane, /detect/lane_borders
        └─► detect_signs.py ─┬─► /detect/sign            (closest tag ID)
                             └─► /detect/sign_detections (all tags + area)

/detect/lane_borders ──► detect_intersection.py ──► /detect/intersection

/detect/intersection ─┐
/detect/sign ─────────┴─► switch_control.py ──► /switch/mode
                              ▲                 /switch/turn_direction
                              │                        │
                    /plan/turn_command                 │
     /detect/sign_detections  │                        ▼
              └───► mapping_pathplanning.py ◄──────────┘
                    │  (plans the route, commands the turn)
                    └─► /mapping/state ──► visualization.py
```

Two sign topics because the two consumers need different things.
`switch_control` reads intersection signs and must see them at any size, so
`/detect/sign` stays unthresholded. Gate mapping needs the tag's *area* — how
far away it is — so it reads `/detect/sign_detections`, which carries every
detection with its corners and an accept/reject decision. The dashboard draws
that same topic, so the size threshold is applied in exactly one place.

`switch_control` takes the planner's command when it is fresh, and otherwise
falls back to its old sign-based random choice — so a dead mapping node
degrades to the previous behaviour instead of stopping the robot.

### Files

| File | Role |
|---|---|
| `config/city.json` | **The map.** Nodes, ports, layout hints |
| `config/gates.json` | Gate tag ID → colour (cosmetic) |
| `config/config.json` | Timings; also the planner's cost model |
| `src/city_map.py` | ROS-free: port geometry, loading/validation, `GraphMap` |
| `src/planner.py` | ROS-free: Dijkstra over directed-edge states |
| `src/graph_layout.py` | ROS-free: drawing geometry + crossing detection |
| `src/crossing.py` | ROS-free: when to end an intersection crossing |
| `src/gate_detection.py` | ROS-free: which gate sightings may be believed |
| `src/run_timing.py` | ROS-free: measuring street and turn times |
| `src/mission_setup.py` | ROS-free: menu validation and command building |
| `src/mission_tui.py` | Menu front end for the whole mission |
| `src/mapping_pathplanning.py` | The mission: localization, gates, planning, commands |
| `src/switch_control.py` | Behaviour FSM; consumes the turn command |
| `src/visualization.py` | Live map + path + position window |
| `src/dashboard.py` | Perception debug window; draws tag boxes |
| `launch/mapping_pathfinding.launch` | Everything, with args |
| `tests/` | 209 offline tests |

`src/graph_only.py` was **deleted** — a superseded prototype that carried a
third copy of the city dict. It is in git history if ever needed.

---

## 3. Key design decisions

**Planning happens over `(node, entry_port)`, not over nodes.** Because U-turns
are forbidden, where the bot may go next depends on how it arrived. That state
also identifies the street just driven, which is what lets the planner target
individual streets. U-turns are impossible to even express: no direction table
maps a port onto itself.

**The control loop runs at 30 Hz.** `frame_skip: 1` — the perception pipeline
is 20 ms/frame against a 33 ms budget, offboard on the laptop, so there is
headroom. The rate is set here, not by `publish_rate`, because the PID runs once
per lane frame. The derivative term is low-pass filtered so it stays stable at
this rate and `frame_skip` can be changed without retuning from scratch.
Measured latency photon-to-command ~75 ms.

**Crossings are closed loop.** The manoeuvre ends when lane markings are seen
again, not when a timer expires; `turn_durations` is only a timeout. This is
what makes the stack tolerant of the bot not being square at the stop line, and
it is why the approach phase has no special handling any more: no red-line
squaring, the same PID gains as normal driving, and only a mild slowdown. The
group that got this reliable first had already reached the same conclusion by
tuning — their `p_angle` was 0.21 against our 2.5, effectively squaring off.

**Port handedness is taken from the driver's seat.** For a bot entering
through port p, port p+1 is on its right and p-1 on its left, so entering
through port 1 gives right = port 2, straight = port 3, left = port 4. This was
originally implemented mirrored and only showed up on the robot; see §4.

**Edge keys are direction-independent** (`A1__B1` from either end), which is
what makes a gate "span the whole street" fall out for free.

**A mapped gate is never overwritten.** The first gate written to a street
stays, and a gate stays on the first street it was found on. The camera sees
past the intersection, so later sightings are the ones taken from further away
and at a worse angle — there is nothing to gain from letting one win. In front
of that write sit three filters: gate mapping is off while stopped and while
crossing, the tag must be big enough to be on this street (`gate_min_area`),
and it must be seen several frames running (`gate_confirm_frames`). See
[`parameters.md`](parameters.md#which-gate-belongs-to-which-street).

The trade-off is deliberate: making the first write permanent means a *wrong*
first write is permanent too. That is why the threshold and the confirmation
streak exist. `gate_min_area` is **900 px²**, set from rosbag evidence rather
than by eye (§4), and lives in `config/config.json` with every other tuned
constant; the launch arg of the same name only overrides it when set positive,
so the config stays the single source.

**A gate is "passed" when its tag is seen, not when its street is entered.**
Two different questions get two different answers, on purpose:

- *Routing* ticks a gate off on entering its street (`remaining_gates`),
  because that is the moment the leg to the following gate has to be planned.
- *Reporting* — the map, the scored time — waits until the tag has actually
  been confirmed on that street (`gates_sighted`). Entering a street is not
  evidence of having passed the gate on it, and a display that says otherwise
  is claiming something the bot does not know.

The two diverging mid-street is normal, and the info box shows both.

The fallback matters as much as the rule: if the street is driven end to end
and the tag was never caught, the gate is credited anyway on reaching the red
line. Without it a single missed detection would leave a gate pending forever
and the run would never report a result — worse than crediting one street late.

**The route is replanned after every crossing.** Cheap on this graph, and it
means a corrected position immediately produces a corrected route.

**The cost model is measured, not assumed.** A move costs
`stop + turn + street`, and all three come off the same drive-mode transitions,
so they tile the move with no gap or overlap. The mapping run is untimed, so
measuring during it is free; the gate run is timed, and is planned against what
mapping learned. Repeated measurements of a street are reduced with the median.
Turn times fall back to the config seeds (`planner.turn_durations`, measured on
the track) for any direction the run never performed.

Those seeds are deliberately *not* `switch_control.timers.turn_durations`:
those are crossing **timeouts**, an upper bound the crossing normally ends well
before, and charging them would overcharge every turn.

**The timed run is reported against the same boundary the planner uses.** The
scored number is first movement to the last gate being passed, but the number
compared against the estimate is measured to the bot standing at the red line
beyond it — every leg the cost model charges ends with a stop, so comparing the
estimate to the scored number would make the planner look optimistic by exactly
one street each time.

**The two phases are separated by a hard stop.** When mapping finishes the bot
holds at the red line and does not move until the gate run is started, so it
can be picked up and placed. That needs a command distinct from "no opinion":
`TURN_COMMAND_HALT` (-1), because `TURN_COMMAND_NONE` (0) hands the bot to
`switch_control`'s random fallback and it drives away. Starting the run with a
`gate_run_start_edge` also means "I moved it", which resets `switch_control` to
lane following — it was parked at a red line, and after being carried there is
no intersection in front of it to cross.

**Gate order is announced as tag IDs** and passed in as `gate_order`. Nothing
about the route is hard-coded; it is derived at runtime from the map that was
built. Colours (`config/gates.json`) are for logs and display only.

---

## 4. Verified behaviour (offline)

The real track (`config/city.json`) is 3 nodes / 5 streets: **A** is the 4-way,
**B** and **C** are T-junctions. All 10 `(node, entry_port)` states are
reachable and all 5 streets drivable without a U-turn — there are no dead ends,
so the planner can never strand the bot.

Mapping from `A,1,B,1`:

```
turns : RIGHT -> STRAIGHT -> STRAIGHT -> STRAIGHT -> STRAIGHT
route : A4__B2 -> A2__C2 -> B3__C4 -> A1__B1 -> A3__C1
```

5 moves to cover 5 streets (4 is the theoretical floor — greedy, and the
mapping phase is untimed, so this is fine).

Gate run, gates 5/6/7 on `A1__B1`/`B3__C4`/`A3__C1`, announced order `[7,5,6]`:

```
turns : RIGHT -> LEFT -> STRAIGHT
route : A2__C2 -> A1__B1 -> B3__C4               (~27 s estimated)
```

Test coverage: port geometry and the no-U-turn invariant, city validation,
`GraphMap` moves/gates/coverage, gate-sighting filters (size, confirmation
streak, drive mode, and the never-overwrite lock), planner reachability from
every start state, **all 6 gate orderings from all 10 start states**, greedy
coverage from every start state, full mission simulations, and 2×2/3×3/4×5 grid
cities.

```bash
# host
.venv/bin/python -m pytest src/packages/mapping_pathfinding/tests -q
# in the container
python3 -m pytest /workspace/src/packages/mapping_pathfinding/tests -q
```

### ROS simulation (no robot needed)

`launch/simulate.launch` runs the mapping node, the visualization and a fake
driver (`tests/simulate_drive.py`) that obeys `/plan/turn_command` and fakes
gate detections. It exercises the real ROS layer — topics, timers, the phase
switch — without a camera, motors or a Duckiebot.

```bash
roslaunch mapping_pathfinding simulate.launch \
    gate_order:=7,5,6 auto_start_gate_run:=true step_time:=0.4
```

> **Run it against a private roscore.** The container's `ROS_MASTER_URI` points
> at the robot, and the fake driver publishes `/switch/mode` — on the real
> master that commands the actual Duckiebot.
>
> ```bash
> export ROS_MASTER_URI=http://localhost:11312
> export ROS_IP=127.0.0.1
> roscore -p 11312 &
> ```

This has already earned its keep twice:

1. `auto_start_gate_run` fired the moment the last street was *entered*, before
   that street's gate had been seen — so the run started against an incomplete
   map and fell back to a random turn. Fixed; pinned by
   `test_coverage_complete_does_not_imply_all_gates_found`.
2. While waiting for a not-yet-seen gate the planner stood down, handing the
   bot to the random fallback — which then commanded a turn the graph does not
   allow and **lost the position permanently**. In a 45 s run that produced 6
   localization losses. Now the planner keeps commanding while it waits,
   preferring streets with no gate recorded (where a missed one would be):
   0 losses, 0 random turns in the same scenario.
3. Streets were drawn crossing each other, which reads as an intersection that
   does not exist. Two causes: the curvature was far too large (1.7), and the
   renderer used matplotlib's `arc3`, which builds its curve in **display**
   coordinates — so the axis aspect ratio warped the bow, labels drifted off
   their own street, and the geometry no longer matched anything testable.
   Streets are now sampled explicitly in data coordinates from
   `graph_layout.py`, so what is drawn is exactly what `tests/test_layout.py`
   checks.

### A note on that last one

The first fix *looked* right — the crossing detector reported zero crossings —
but the rendered picture still crossed, because the detector was modelling a
curve matplotlib never drew. Layout claims are only worth anything when the
image is actually looked at: render one with `~headless` + `~snapshot_path`
rather than trusting the numbers.

### Rendering the map without a robot

`tests/render_example_visualization.py` writes a PNG of the visualization
window with no roscore, no camera and no Duckiebot. It stubs `rospy`, then
drives the real `GraphMap` through a real exploration and plans the gate run
with the real planner, so the picture is the actual renderer against an actual
plan — only ROS is faked. Used for the figure in `docs/latex/`, and it is the
cheapest way to look at a layout change.

```bash
# host, if matplotlib/cv2/networkx are installed
python3 src/packages/mapping_pathfinding/tests/render_example_visualization.py
# otherwise, in the image (needs no robot and no ROS master)
docker run --rm -v "$PWD:/workspace" -w /workspace --entrypoint python3 \
    quack-attack-duckierace_env:latest \
    src/packages/mapping_pathfinding/tests/render_example_visualization.py
```

It also surfaced a cosmetic issue worth knowing about before a demo: with every
street carrying a time and a gate, the label boxes of the two parallel streets
between A and C overlap. Nothing is wrong with the geometry — the streets
themselves stay clear of each other — but the two labels sit almost on top of
one another.

### Found on the robot

**Left and right were mirrored.** `LEFT_OF`/`RIGHT_OF` were swapped, so the
planner commanded the opposite turn to the one it meant. Caught in Step 2 by
driving towards B1 and seeing `Planned turn RIGHT is not available at B port 1`,
when a right turn there is physically fine and leads to B2 -> A4.

The offline tests could never have found this: the graph is internally
consistent either way, so every reachability, coverage and gate-run test passed
against mirrored geometry. Only the physical track defines which way is right.
`test_handedness_matches_the_track` now pins both observed cases.

Two things this changes: every route above (the turn *names* differ, though the
streets driven are the same shape of problem), and the gate run got cheaper —
3 moves instead of 4.

**Two bugs the workflow rehearsal caught before the track did.** Both were
found by driving the full two-phase sequence in simulation rather than by
reading the code:

1. After the gate run finished, the phase flipped to `DONE` *inside* a replan
   that had already decided not to halt — so the bot would have driven on after
   the timed run instead of stopping.
2. Starting the run after repositioning made `switch_control` perform a
   *crossing manoeuvre* from a standstill, because it was still parked in
   `STOPPED` at the red line it had held at. There is no intersection in front
   of a bot that has just been carried to a street's start. Hence
   `/plan/resume_driving`.

**A gate behind the bot counted as driven.** Mapping ended at the red line of
`B3__C4`, which carries gate 9; the announced order began with 9, and the run
marked it done before moving.

The cause is an ambiguity in the position representation that had been harmless
until then. A planner state `(node, entry_port)` means "just traversed the
street attached to `entry_port`" — and that is true both while *driving* that
street and while *stopped at the red line at its end*. Those are opposite
situations for a gate on it: ahead of the bot in the first, behind it in the
second. Everywhere the state was reached by actually driving, so the two never
had to be told apart; the hard stop introduced between the phases is what
created a position the bot arrives at without having driven the street *in this
run*.

Fixing the accounting exposed a second, hidden bug: the planner could not route
back onto the street it was standing on at all. Dijkstra starts with
`best[start] = 0`, so the start state can never be re-reached, and
`plan_to_edge` answered "you are already there" for that street. `require_move`
seeds the search from the start state's *successors* instead, leaving the start
state an ordinary unvisited node — which is what makes a lap back onto it
plannable.

Worth noting how the verification went: the first probe of the fix appeared to
show the bug still present, because it sampled the state 1.5 s after starting
the run, and the simulator crosses an intersection roughly every second. It was
reading a position two streets down a route that was already correct. Tracing
every state message instead of sampling one showed the fix working. On a
simulator that runs an order of magnitude faster than the robot, "check shortly
after" is not a timing assumption that survives.

**A gate was missed, and nobody noticed.** The first full mapping run looked
perfect — five streets green, four gates on the right streets, clean summary.
The rosbag showed `A3__C1` had no gate at all: tag 8 was seen there with a
healthy 2739 px², but `gate_min_area` was 1400, so it only counted for two
frames and never reached the three-frame confirmation streak.

The lesson is about *evidence*, not about the threshold. Eyeballing the
dashboard cannot tell 1400 from 900 — both look like "the gate goes green
before I reach it". The bag can, because it holds every sighting with its area
and the street the bot was on, so the two distributions can actually be
compared: worst wrong-street sighting 501 px², first right-street acceptance
1426 px². A threshold picked by eye landed at the top of that gap instead of
the middle of it. Record a bag for every run.

**Gate detection worked too well.** Approaching or stopped at an intersection,
the bot picked up a gate tag from a street beyond it and overwrote the gate it
had already mapped for the street it was on. The map then no longer matched the
track, which would have sent the gate run to the wrong street.

Also invisible offline, and for the same reason as the handedness bug: nothing
about the *graph* was wrong. The camera simply sees further than one street,
which is a fact about the track and the lens, not about the data structure. Now
guarded four ways (§3), verified over ROS by replaying the exact sequence —
map gate 5 while driving, then push a big, well-detected gate 9 while stopped,
while crossing, and while driving the same street again. Gate 5 survives all
three.

---

## 5. Running it

### Build / rebuild

Most of the time: **nothing to build.** The repo is bind-mounted at
`/workspace`, and `devel/.catkin` points at `/workspace/src`, so
`$(find mapping_pathfinding)` resolves to the source directory. Python files,
`config/*.json` and `launch/` are therefore picked up live — edit and re-run.

The workspace uses **`catkin_make`** (not `catkin build`), and `entrypoint.sh`
runs it automatically on every container start, then sources
`devel/setup.bash`. tmux panes get their environment from `.bashrc`, which the
Dockerfile writes — `docker exec` does not run the entrypoint, which is exactly
why the sourcing lives there too.

A **Docker image rebuild is only needed when the Dockerfile changes** (e.g. the
pip dependency layer), because `start.sh` recreates the container but does not
rebuild the image:

```bash
docker compose build duckierace_env
./start.sh <vehicle>
./attach_tmux.sh
```

### Launching

```bash
# Mapping phase
roslaunch mapping_pathfinding mapping_pathfinding.launch start_edge:=A,1,B,1

# Timed gate run (order announced on site)
roslaunch mapping_pathfinding mapping_pathfinding.launch \
    mission_phase:=GATE_RUN start_edge:=C,4,B,3 gate_order:=7,5,11

# Perception + planning, no motors
roslaunch mapping_pathfinding mapping_pathfinding.launch driving:=false

# Check the planner -> switch_control command path
roslaunch mapping_pathfinding mapping_pathfinding.launch force_turn:=LEFT
```

`start_edge:=A,1,B,1` means *driving from A port 1 towards B port 1*, so the
next intersection is B.

Switch phases at runtime:

```bash
rosservice call /$VEHICLE_NAME/mapping_pathplanning_node/start_gate_run
```

Useful topics:

```bash
rostopic echo /$VEHICLE_NAME/mapping/state        # map, path, position, mission
rostopic echo /$VEHICLE_NAME/plan/turn_command    # 0=none, 1=L, 2=S, 3=R
```

---

## 6. Open questions

1. **Does passing through a not-yet-due gate break the order?** Unknown how the
   judges score it. `strict_gate_order:=true` routes around future gates where
   possible; default is off because it costs time. **Needs an answer from the
   organizers.**
2. ~~Per-street travel times~~ — **done.** Measured during the mapping run and
   fed straight into the cost model (§3).

## 7. TODO — next steps

**On the robot, in order** (details and commands in [`test-plan.md`](test-plan.md)):

1. ~~Smoke test~~ — **done**.
2. ~~Lane following and crossings~~ — **done, both work.**
3. ~~Verify the command path~~ — **done.**
4. ~~Calibrate `gate_min_area`~~ — **done: 1500 px²**, in `config/config.json`.
5. ~~Mapping run~~ — **done. All gates identified correctly.**
6. **Gate run** — the next thing to run, and the last untested piece. Follow
   [`workflow.md`](workflow.md): mapping, hold, reposition, start. Compare the
   wall-clock time against the estimate, which is now built from measured
   street times rather than a flat guess.

**Code, in rough priority order:**

1. **Localization resync — the top code priority.** A failed move sets
   `localization_ok = False`, which is a **one-way latch**: the planner stands
   down for the rest of the run even though later moves still apply cleanly, so
   the mission is unrecoverable. Confirmed in simulation. Better: re-derive the
   position from the next observed intersection sign — a 4-way vs. a T-junction
   narrows the candidates sharply on a graph this small. Until then, one
   mis-executed turn on the track ends the run.
2. ~~Gate attribution guard~~ — **done.** `detect_signs` now publishes the
   detection area, and mapping is filtered by size, confirmation streak and
   drive mode, with the first gate on a street made permanent (§3).
3. **Exploration could be cheaper** (5 moves vs. a 4-move optimum). Untimed, so
   low priority.
4. **Legacy cleanup.** `src/util.py`, `config/control_lane_node.json` and
   `config/detect_lane.json` predate the U-Net perception and appear dead;
   confirm and remove.
5. **Launchers.** `launchers/*.sh` still `rosrun follow_lane …` and are stale
   now that a launch file exists.

---

## 8. Note for other packages (out of scope here)

`follow_lane` imports `tkinter` and `cv_bridge`, neither of which the Dockerfile
installs (`python3-tk`, `ros-noetic-cv-bridge`). Not touched, since this work is
scoped to `mapping_pathfinding` — but those nodes will fail in a clean image.
