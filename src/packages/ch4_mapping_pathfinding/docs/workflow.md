# Race-day workflow

Two phases with a **deliberate stop between them**, so the bot can be picked up
and placed for the timed run.

There are two ways to drive it: the **menu** (easier, recommended on the day)
or the **commands by hand** (what the menu runs). Both are below; the menu
prints every command it issues, so switching between them mid-session is fine.

Parameters are explained in [`parameters.md`](parameters.md); what to test and
in what order is [`test-plan.md`](test-plan.md).

---

## The menu

```bash
rosrun ch4_mapping_pathfinding mission_tui.py
```

It walks the phases in the order they happen: configure the mapping run, watch
it, and when it stops at the red line, configure and start the gate run from
the same menu. Enter accepts the value in brackets, so a default run is mostly
pressing Enter.

```
==================================================================
  Main menu
==================================================================
  1) Configure and run the MAPPING phase
  2) Show the city reference (placements, gate colours)
  q) Quit
```

What it does for you:

- **Checks the placement against the city** before launching. `A,1,C,1` is
  rejected with *"A port 1 leads to B port 1, not C port 1"* rather than being
  believed and quietly wrecking the map.
- **Reads the placement back** in words: *"at A port 1, driving along A1__B1
  towards B (entering port 2)"*.
- **Keeps one roslaunch alive across both phases.** This is the part worth
  getting right by hand too — the map, the gates and the measured street times
  live in the mapping node, and restarting it between phases throws them away.
- **Offers the recommended placement** as the default for the gate run.
- **Refuses to start** if a gate in the announced order was never mapped, and
  says which — before anything moves.
- **Warns about streets with no gate**, since the map cannot tell "no gate
  there" from "one was missed".
- Ctrl-C at any point shuts the mission down with SIGINT, so `control_wheels`
  stops the robot on the way out.

roslaunch's output goes to a log file (the path is shown) rather than the
screen, because a menu prompt buried in scrolling node output is unreadable.
For the usual view, in another tmux pane:

```bash
tail -f ~/mission_HHMMSS.log
```

### Rehearsing it without the robot

```bash
rosrun ch4_mapping_pathfinding mission_tui.py --simulate
```

Same menu, same phase switch, same commands — driven against the fake driver,
so the whole sequence can be practised before the track. Nothing moves.

---

## The commands by hand

The rest of this document. This is what the menu runs.

---

## The placement convention

**The bot always starts at an intersection exit**, pointed along the street it
is about to drive. It never starts mid-street.

`start_edge:=A,4,D,2` means: *placed at A's port 4 exit, driving towards D's
port 2.* So `D` is the next intersection, and the whole of street `A4__D2` is
driven — which is what makes its measured time meaningful.

This holds for **both** phases. The mapping run and the gate run each start this
way, and each is timed from the moment the wheels first turn.

---

## Phase 1 — mapping (untimed)

Place the bot at an intersection exit, then:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch start_edge:=A,4,D,2
```

If the gate order is already known, pass it now — the bot will then recommend
where to place it for the run:

```bash
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    start_edge:=A,4,D,2 gate_order:=7,5,6
```

The bot drives every street at least once, records which gate is on which
street, and times each street. It then **stops at the next red line and stays
there**. It will not move again until you tell it to.

You will see this when it is done:

```
==============================================================
MAPPING COMPLETE -- holding at this red line on A2__C2
Measured street times (seconds):
  A1__B1        4.90  (4.90)
  A2__C2        3.20  (3.20)
  A3__C1        4.45  (4.45)
  A4__B2        3.40  (3.40, 3.40)
  B3__C4        6.10  (6.10)
Turn times (seconds, * = measured):
  LEFT     * 2.60  (3 sample(s))
  STRAIGHT * 3.10  (1 sample(s))
  RIGHT    * 0.95  (1 sample(s))
Gates found (3):
  gate 5   on A1__B1 (pink)
  gate 7   on A3__C1 (blue)
  gate 6   on B3__C4 (green)
Best start positions for gate order [7, 5, 6]:
  1. start_edge:=C,1,A,3     12.1s, 2 crossing(s) (first gate is on this street)
  2. start_edge:=A,2,C,2     16.6s, 3 crossing(s)
  3. start_edge:=A,3,C,1     16.6s, 3 crossing(s) (first gate is on this street)
The bot will not move again until the gate run is started
==============================================================
```

The same information is on the visualization: measured seconds under each
street, and `>> HOLDING AT RED LINE <<` in the info box.

**Leave the launch running.** Everything learned — the map, the gates, the
timings — lives in that node. Restarting it throws all of it away.

### Reading the recommendation

Note the top two: `C,1,A,3` and `A,3,C,1` are the *same street* driven in
opposite directions, and they differ by 4.5 s. Both collect the first gate for
free; they differ in which intersection the bot ends up at, and therefore in
the whole rest of the route. Which way round you set the bot down is worth more
than it looks.

---

## Phase 2 — the gate run (timed)

1. **Pick the bot up** and place it at the recommended intersection exit,
   pointed along that street.
2. **Tell it the order and where it is**, then start:

```bash
N=/$VEHICLE_NAME/mapping_pathplanning_node

rosparam set $N/gate_order          "7,5,6"
rosparam set $N/gate_run_start_edge "C,1,A,3"
rosservice call $N/start_gate_run
```

Or as one line:

```bash
N=/$VEHICLE_NAME/mapping_pathplanning_node && \
  rosparam set $N/gate_order "7,5,6" && \
  rosparam set $N/gate_run_start_edge "C,1,A,3" && \
  rosservice call $N/start_gate_run
```

The service answers with the plan it committed to, and refuses rather than
guessing if something is wrong:

```
success: True
message: "Gate run started for [5, 6] from A3__C1 (16.4s estimated)"
```

`[5, 6]` rather than `[7, 5, 6]` is correct when the bot was placed on gate 7's
street: driving that street *is* passing that gate, which is exactly why the
recommendation put it there.

Setting `gate_run_start_edge` also tells the bot it has been **moved**, so it
drives off along the street instead of performing a turn manoeuvre — it was
parked at a red line, and there is no intersection in front of it any more.

> If you want it to carry on from where it stopped, without moving it, leave
> `gate_run_start_edge` unset. It will then cross the intersection it is
> standing at, exactly as it would have during mapping.
>
> Note what this costs: the bot is at the **end** of a street, so a gate on
> that street is *behind* it. If the announced order starts with that gate, the
> route has to drive a whole lap back onto the street to collect it. Starting
> the run there is correct but not free — on the test track that turned a 34 s
> route into a 61 s one. Repositioning is usually worth the walk.

While it runs, the visualization tracks progress: a gate turns **green** with a
`PASSED` tag once its tag has actually been *seen* on that street, the next one
still owed gets a **blue** `NEXT` border, and the info box counts them off with
the elapsed time against the estimate.

Being on a gate's street is not enough to turn it green — the bot has to have
driven far enough to see it. So `routing to [...]` in the info box runs ahead
of `seen [...]`, which is normal: the route to the following gate is planned as
soon as a street is entered, while the gate itself is only credited when
observed. If the tag is missed entirely, the gate is credited on reaching the
red line at the end of that street, so a detection miss cannot stall the run.

When the run finishes it stops at the red line beyond the last gate and holds,
reporting the time:

```
Passed gate 5 on A1__B1 -- 1 left
Passed gate 6 on B3__C4 -- 0 left
Street B3__C4 driven in 6.10s
==============================================================
GATE RUN COMPLETE
  gates driven : [7, 5, 6]
  TIME         : 24.6 s   (first movement -> last gate seen)
  at stop line : 30.7 s   (+6.1 s driving out the last street)
  planner said : 29.4 s   (+1.3 s, +4%)
==============================================================
```

Three numbers because they answer different questions:

- **TIME** is the scored one — wheels first turning to the last gate passed.
- **at stop line** is the same run measured to the bot standing still. Use this
  one against the estimate: every leg the cost model charges ends with a stop,
  so comparing the estimate to `TIME` would make the planner look optimistic by
  one street every time.
- **planner said** is what the measured cost model predicted. A large gap means
  the street times from mapping no longer describe the track — worth a re-map
  rather than a re-tune.

---

## If something is wrong

| Message | Meaning |
|---|---|
| `Gate(s) [9] not mapped yet` | The service refused: that gate was never seen. Check the announced IDs against `Gates found` above. |
| `No gate_order set` | `rosparam set $N/gate_order` was missed, or quoted wrongly — it needs to be a string: `"7,5,6"`. |
| `Invalid gate_run_start_edge` | Not a real street, or the wrong way round. It must be `from,port,to,port` and match `city.json`. |
| `Localization lost` | The bot turned somewhere the graph says it could not. The planner stands down; see `current-state.md` §7. |
| Bot drives on after mapping | It should hold. Check `rostopic echo $NS/plan/turn_command` reads `-1`. |

Useful while running:

```bash
NS=/$VEHICLE_NAME
rostopic echo $NS/mapping/state       # map, timings, plan, phase
rostopic echo $NS/plan/turn_command   # -1=hold, 0=none, 1=L, 2=S, 3=R
```

### Console output

The console is kept to the mission narrative — roughly four lines per
intersection, and nothing that repeats:

```
Turning RIGHT (planner)
Crossing done after 1.36s (lane_reacquired)
Moved RIGHT: now on A4__B2
Street A4__B2 driven in 2.86s
```

plus a line whenever a gate is recorded or passed. Anything that fires per
frame or per detection is at debug level. To bring it back for one node:

```bash
rosservice call /$VEHICLE_NAME/detect_signs_node/set_logger_level \
    "{logger: 'rosout', level: 'debug'}"
```

That is the one to reach for when a tag is not being picked up at all — though
a rosbag is better, since it holds every detection rather than a sample.

---

## Rehearsing the raw commands without the robot

`mission_tui.py --simulate` is the easier way to do this. To rehearse the
commands themselves:

```bash
# private roscore -- the container's ROS_MASTER_URI points at the real robot
export ROS_MASTER_URI=http://localhost:11312
export ROS_IP=127.0.0.1
roscore -p 11312 & sleep 3

# halt_timeout gives you 90 s to type the phase-2 commands
roslaunch ch4_mapping_pathfinding simulate.launch \
    gate_order:=7,5,6 halt_timeout:=90 step_time:=0.3
```

It maps, prints the same summary, and holds — then the phase-2 commands above
work verbatim (with `VEHICLE_NAME` set to whatever the sim runs under).
