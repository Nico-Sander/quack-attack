#!/usr/bin/env python3

"""Replay an exported run through GapPlanner, offline.

Why this exists: hand-tuning on the robot gives one noisy sample per session and
confounds every parameter with every other. GapPlanner is rospy-free and takes
time as an argument, so a recorded run can be pushed through it at any parameter
values in milliseconds - same input every time, which turns a parameter question
into a controlled experiment.

    python3 tools/replay.py crash.jsonl                       # what actually happened
    python3 tools/replay.py crash.jsonl --set gap_min_width=0.15
    python3 tools/replay.py crash.jsonl --sweep duckie_margin_gain=0.04,0.06,0.10
    python3 tools/replay.py crash.jsonl --contact              # per-frame near-miss detail

The replay mirrors control_lane_node.run(): messages are applied as they arrive,
then the planner is stepped at a fixed 10 Hz. It does NOT model the robot moving,
so a replay answers "what did the planner decide, given what it saw" - not "where
would the robot have ended up". That distinction matters: it can tell you the
planner believed a gap was clear when it was not, but it cannot tell you the
manoeuvre was undriveable.
"""

import argparse
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
CONFIG = os.path.join(HERE, "..", "config", "control_lane_node.json")

# Import GapPlanner without ROS present.
for name in ("rospy", "std_msgs", "std_msgs.msg",
             "duckietown_msgs", "duckietown_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
for attr in ("Float64", "String"):
    setattr(sys.modules["std_msgs.msg"], attr, object)
sys.modules["duckietown_msgs.msg"].Twist2DStamped = object
sys.path.insert(0, SRC)
import control_lane_node as cln  # noqa: E402

STEP_HZ = 10.0

# Measured on the course: bbox-bottom ymax -> distance from the robot's front, cm.
# Used only for reporting, so a replay can say "24 cm" instead of "ymax 0.66".
YMAX_TO_CM = [(0.462, 120.0), (0.470, 100.0), (0.503, 70.0), (0.545, 50.0),
              (0.638, 30.0), (0.823, 15.0), (1.000, 8.0)]


def ymax_cm(ymax):
    """Piecewise-linear interpolation of the measured curve."""
    pts = YMAX_TO_CM
    if ymax <= pts[0][0]:
        return pts[0][1]
    if ymax >= pts[-1][0]:
        return pts[-1][1]
    for (y0, d0), (y1, d1) in zip(pts, pts[1:]):
        if y0 <= ymax <= y1:
            f = (ymax - y0) / (y1 - y0) if y1 > y0 else 0.0
            return d0 + f * (d1 - d0)
    return pts[-1][1]


def load_params(overrides):
    with open(CONFIG) as fh:
        ctrl = json.load(fh)["parameters"]["controller"]
    params = {k: float(v["default"]) for k, v in ctrl.items()}
    for key, value in overrides.items():
        if key not in params:
            raise SystemExit(f"unknown parameter: {key}\nknown: {', '.join(sorted(params))}")
        params[key] = float(value)
    return params


def load_events(path):
    events = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    events.sort(key=lambda e: e["t"])
    return events


def replay(events, params):
    """Push events through a fresh planner at STEP_HZ. Returns per-step records."""
    planner = cln.GapPlanner(params)
    records = []

    got_lane = got_borders = got_duckies = False
    duckies_now = []
    idx = 0
    end = events[-1]["t"] if events else 0.0
    t = 0.0

    while t <= end:
        # Apply every message up to now, mirroring the callbacks.
        while idx < len(events) and events[idx]["t"] <= t:
            ev = events[idx]
            data = ev["data"]
            if ev["kind"] == "lane":
                planner.update_lane_error(data)
                got_lane = True
            elif ev["kind"] == "lane_borders":
                planner.update_lane_borders(
                    t,
                    data.get("yellow_x", 0.05), data.get("white_x", 0.95),
                    bool(data.get("yellow_valid", True)),
                    bool(data.get("white_valid", True)))
                # After update_lane_borders, which sets the yellow_seen/white_seen
                # flags this depends on.
                planner.set_lines_crossed(bool(data.get("lines_crossed", False)))
                got_borders = True
            elif ev["kind"] == "duckie_BB":
                ducks = data.get("duckies", [])
                if not ducks and data.get("detected", False) \
                        and str(data.get("class_name", "")).lower() == "duckie":
                    ducks = [data]
                planner.update_duckies(t, ducks)
                duckies_now = ducks
                got_duckies = True
            idx += 1

        if got_lane and got_borders and got_duckies:
            v, omega, dbg = planner.step(t)
            records.append(step_record(t, planner, params, v, omega, dbg, duckies_now))

        t += 1.0 / STEP_HZ

    return records


def step_record(t, planner, params, v, omega, dbg, duckies_now):
    """One replayed step, plus the clearance analysis that motivates all this."""
    drive_x = dbg.get("front_probe_x", 0.5)

    # Clearance between the robot's own footprint and the nearest RAW detection -
    # raw, not inflated, because the question is whether it would physically touch.
    # The footprint half-width is evaluated at each duckie's RANGE. An earlier
    # version used the fixed front_slice_half and therefore reported "clear" on a
    # bag that ended in a collision - the same bug this rewrite removes from the
    # planner, reproduced in the tool meant to detect it.
    # Clearance is tracked in CENTIMETRES, not image fraction. The same image gap
    # means very different physical clearances at different ranges, so ranking by
    # image fraction picks the wrong duckie: 0.037 of frame at 61cm is 5.9cm of room,
    # while 0.044 at 8cm is 0.9cm. Ranking by image width reported the far one as the
    # closer call and cleared a run that ended in a collision.
    clearance_cm = None
    clearance = None
    nearest = None
    for d in duckies_now:
        try:
            xmin, xmax = float(d["xmin"]), float(d["xmax"])
            ymax = float(d["ymax"])
        except (KeyError, TypeError, ValueError):
            continue
        half = cln.robot_half_image_static(
            params["robot_width_cm"], params["camera_width_k"], ymax_cm(ymax))
        if xmax < drive_x - half:
            gap = (drive_x - half) - xmax
        elif xmin > drive_x + half:
            gap = xmin - (drive_x + half)
        else:
            gap = -min(xmax - (drive_x - half), (drive_x + half) - xmin)
        dist_cm = ymax_cm(ymax)
        gap_cm = gap * dist_cm / params["camera_width_k"]
        if clearance_cm is None or gap_cm < clearance_cm:
            clearance_cm = gap_cm
            clearance = gap
            nearest = (ymax, xmin, xmax)

    return {
        "t": round(t, 2),
        "state": dbg["state"],
        "reason": dbg["reason"],
        "v": round(v, 3),
        "omega": round(omega, 3),
        "goal_x": round(dbg.get("lane_target_x", 0.5), 3),
        "target_x": round(dbg.get("target_x", 0.5), 3),
        "drive_x": round(drive_x, 3),
        "gap": dbg.get("selected_free_interval"),
        "gap_w": round((dbg["selected_free_interval"][1]
                        - dbg["selected_free_interval"][0]), 3)
                 if dbg.get("selected_free_interval") else 0.0,
        "n_spans": len(dbg.get("blocked_intervals", [])),
        "lane_source": dbg.get("lane_source", "?"),
        "clearance": round(clearance, 3) if clearance is not None else None,
        "clear_cm": round(clearance_cm, 1) if clearance_cm is not None else None,
        "near_cm": round(ymax_cm(nearest[0]), 1) if nearest else None,
    }


def is_contact(r, contact_cm):
    """Lateral overlap AND close enough to actually touch.

    Overlap alone is meaningless: a duckie 120 cm ahead overlaps the robot's column
    on every approach and is simply something to steer around. Only overlap at
    contact range is a hit, which is why this needs the distance curve rather than
    image coordinates alone - the same conflation that makes the image-space margin
    parameters untunable.
    """
    return (r["clear_cm"] is not None and r["clear_cm"] < 0
            and r["near_cm"] is not None and r["near_cm"] <= contact_cm)


def report(records, label, contact_cm=12.0):
    if not records:
        print(f"[{label}] no steps - sensors never all reported. "
              f"Is /detect/duckie_BB in the bag?")
        return None

    states = {}
    flips = 0
    prev = None
    for r in records:
        states[r["state"]] = states.get(r["state"], 0) + 1
        if prev is not None and r["state"] != prev:
            flips += 1
        prev = r["state"]

    contacts = [r for r in records if is_contact(r, contact_cm)]
    # In-path but not close: normal, this is what avoidance is for.
    in_path = [r for r in records
               if r["clear_cm"] is not None and r["clear_cm"] < 0
               and not is_contact(r, contact_cm)]
    # Within the calibration's error bars (~30% at close range) a sub-2cm pass is
    # indistinguishable from a touch, so it is called out separately.
    grazes = [r for r in records
              if r["clear_cm"] is not None and 0 <= r["clear_cm"] < 2.0
              and r["near_cm"] is not None and r["near_cm"] <= contact_cm * 2]
    # Closest approach considering only frames near enough to matter.
    near = [r for r in records
            if r["near_cm"] is not None and r["near_cm"] <= contact_cm * 2.5]
    worst = min(near, key=lambda r: r["clear_cm"]) if near else None

    dt = 1.0 / STEP_HZ
    print(f"\n=== {label} ===")
    print(f"  steps {len(records)}  ({len(records) * dt:.1f} s)   state flips: {flips}")
    for st, n in sorted(states.items(), key=lambda kv: -kv[1]):
        print(f"    {st:16s} {n * dt:6.1f} s  ({100.0 * n / len(records):4.1f}%)")
    print(f"  CONTACT frames (overlap within {contact_cm:.0f}cm): {len(contacts)}")
    print(f"  GRAZE frames  (passed within 2cm):          {len(grazes)}")
    print(f"  in-path but not close (normal avoidance):   {len(in_path)}")
    if worst:
        print(f"  closest approach: {worst['clear_cm']:+.1f} cm at "
              f"t={worst['t']}s, duckie ~{worst['near_cm']}cm ahead, "
              f"state={worst['state']}")
        print(f"                    reason={worst['reason']}")
    return {"contacts": len(contacts), "grazes": len(grazes),
            "in_path": len(in_path), "flips": flips,
            "worst": worst["clear_cm"] if worst else None}


def show_contacts(records, window=6, contact_cm=12.0):
    bad = [i for i, r in enumerate(records) if is_contact(r, contact_cm)]
    if not bad:
        print(f"\nNo contact frames (no overlap within {contact_cm:.0f}cm). "
              f"Showing the closest approach instead.")
        near = [r for r in records if r["clear_cm"] is not None]
        if not near:
            return
        first = records.index(min(near, key=lambda r: r["clear_cm"]))
    else:
        first = bad[0]
    lo, hi = max(0, first - window), min(len(records), first + window)
    print(f"\nFirst overlap at t={records[first]['t']}s - surrounding frames:\n")
    print(f"{'t':>6} {'state':14} {'drive_x':>8} {'gap_w':>6} {'clear_cm':>9} "
          f"{'dist_cm':>8} {'v':>6} {'omega':>7}  reason")
    print("-" * 96)
    for r in records[lo:hi]:
        mark = " << CONTACT" if is_contact(r, contact_cm) else ""
        print(f"{r['t']:>6} {r['state']:14} {r['drive_x']:>8} {r['gap_w']:>6} "
              f"{str(r['clear_cm']):>9} {str(r['near_cm']):>8} {r['v']:>6} "
              f"{r['omega']:>7}  {r['reason']}{mark}")


def measure_report(events, params, contact_cm=12.0):
    """Measure the clearance the robot ACTUALLY had, with no planner involved.

    Replaying with changed parameters is a counterfactual: the planner decides
    differently, but the recorded detections still come from the trajectory the
    robot really drove, so the two no longer correspond. To ask "how close did it
    actually pass" the planner must be taken out of the loop entirely.

    /debug/free_path_plan records front_probe_x per frame - the column the robot was
    genuinely steering toward - so pairing that with the concurrent detections gives
    the real clearance history, evaluated under the corrected range-aware geometry.
    """
    k = params["camera_width_k"]
    robot_half_cm = params["robot_width_cm"] / 2.0

    ducks = []
    rows = []
    for e in events:
        if e["kind"] == "duckie_BB":
            ducks = e["data"].get("duckies", [])
            continue
        if e["kind"] != "free_path_plan":
            continue
        plan = e["data"]
        drive_x = plan.get("front_probe_x")
        if drive_x is None:
            continue

        worst = None
        for d in ducks:
            try:
                xmin, xmax, ymax = float(d["xmin"]), float(d["xmax"]), float(d["ymax"])
            except (KeyError, TypeError, ValueError):
                continue
            dist_cm = ymax_cm(ymax)
            half = robot_half_cm * k / max(dist_cm, 6.0)
            if xmax < drive_x - half:
                gap = (drive_x - half) - xmax
            elif xmin > drive_x + half:
                gap = xmin - (drive_x + half)
            else:
                gap = -min(xmax - (drive_x - half), (drive_x + half) - xmin)
            gap_cm = gap * dist_cm / k
            if worst is None or gap_cm < worst[0]:
                worst = (gap_cm, dist_cm, xmin, xmax, plan.get("state", "?"))
        if worst:
            rows.append((e["t"], drive_x) + worst)

    if not rows:
        print("no free_path_plan messages with front_probe_x - was the bag recorded "
              "with a build that publishes it?")
        return

    near = [r for r in rows if r[3] <= contact_cm]
    hits = [r for r in near if r[2] < 0.0]
    graze = [r for r in near if 0.0 <= r[2] < 2.0]

    print(f"\n=== measured from the real run ({len(rows)} plan frames) ===")
    print(f"  frames with a duckie inside {contact_cm:.0f}cm : {len(near)}")
    print(f"  of those, lateral overlap (CONTACT)  : {len(hits)}")
    print(f"  of those, passed within 2cm (GRAZE)  : {len(graze)}")

    if near:
        w = min(near, key=lambda r: r[2])
        print(f"\n  closest pass at close range: {w[2]:+.1f} cm")
        print(f"    t={w[0]:.1f}s  duckie {w[3]:.0f}cm ahead at [{w[4]:.2f},{w[5]:.2f}]"
              f"  drive_x={w[1]:.3f}  state={w[6]}")

    focus = sorted(near, key=lambda r: r[2])[:12] if near else []
    if focus:
        print(f"\n  tightest {len(focus)} frames within {contact_cm:.0f}cm:")
        print(f"  {'t':>6} {'clear_cm':>9} {'dist_cm':>8} {'drive_x':>8} "
              f"{'duckie span':>14}  state")
        print("  " + "-" * 66)
        for t, dx, gap_cm, dist_cm, xmin, xmax, state in sorted(focus):
            mark = "  << CONTACT" if gap_cm < 0 else ("  << graze" if gap_cm < 2 else "")
            print(f"  {t:>6.1f} {gap_cm:>9.1f} {dist_cm:>8.0f} {dx:>8.3f} "
                  f"  [{xmin:.2f},{xmax:.2f}]  {state}{mark}")


def detections_report(events):
    """Summarise the RAW detection stream, independent of the planner.

    Motivation: if the robot touches a duckie the planner never saw, no parameter
    can help - the failure is upstream. This shows what the detector actually
    delivered, so "the planner chose badly" can be told apart from "the planner
    was blind".
    """
    ducks = [e for e in events if e["kind"] == "duckie_BB"]
    if not ducks:
        print("no duckie_BB messages in the export")
        return

    seen = []
    for e in ducks:
        for d in e["data"].get("duckies", []):
            try:
                seen.append((e["t"], float(d["ymax"]), float(d["xmin"]), float(d["xmax"])))
            except (KeyError, TypeError, ValueError):
                continue

    print(f"\nduckie_BB messages : {len(ducks)}")
    print(f"frames with >=1 duckie: {sum(1 for e in ducks if e['data'].get('duckies'))}")
    print(f"individual detections : {len(seen)}")
    if not seen:
        return
    ymax_max = max(s[1] for s in seen)
    print(f"closest detection ever: ymax={ymax_max:.3f}  (~{ymax_cm(ymax_max):.0f} cm)")
    print(f"  -> if that is far from contact range, the duckie left the frame "
          f"before impact")

    print("\n  t(s)  n  max_ymax   ~cm   x-span of closest")
    print("  " + "-" * 46)
    end = events[-1]["t"]
    bucket = 0.5
    t = 0.0
    while t <= end:
        window = [s for s in seen if t <= s[0] < t + bucket]
        n_msg = sum(1 for e in ducks if t <= e["t"] < t + bucket)
        if n_msg:
            if window:
                best = max(window, key=lambda s: s[1])
                print(f"  {t:5.1f} {len(window):2d}   {best[1]:6.3f} {ymax_cm(best[1]):5.0f}   "
                      f"[{best[2]:.2f},{best[3]:.2f}]")
            else:
                print(f"  {t:5.1f}  0        -     -   (nothing detected)")
        t += bucket


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("events", help="jsonl from bag_export.py")
    ap.add_argument("--set", action="append", default=[], metavar="K=V")
    ap.add_argument("--sweep", metavar="K=V1,V2,...")
    ap.add_argument("--contact-cm", type=float, default=12.0,
                    help="distance below which a lateral overlap counts as a hit")
    ap.add_argument("--measure", action="store_true",
                    help="measure the clearance the robot actually had, from the "
                         "recorded plan - no planner replay, no counterfactual")
    ap.add_argument("--detections", action="store_true",
                    help="summarise the raw detector stream, independent of the planner")
    ap.add_argument("--contact", action="store_true",
                    help="show frames around the first footprint overlap")
    args = ap.parse_args()

    overrides = {}
    for item in args.set:
        k, _, v = item.partition("=")
        overrides[k.strip()] = v.strip()

    events = load_events(args.events)
    if not events:
        raise SystemExit("no events")

    if args.detections:
        detections_report(events)
        return

    if args.measure:
        measure_report(events, load_params(overrides), args.contact_cm)
        return

    if args.sweep:
        key, _, values = args.sweep.partition("=")
        key = key.strip()
        rows = []
        for value in values.split(","):
            params = load_params({**overrides, key: value.strip()})
            res = report(replay(events, params), f"{key}={value.strip()}",
                         args.contact_cm)
            if res:
                rows.append((value.strip(), res))
        print(f"\n{'value':>10} {'CONTACT':>9} {'GRAZE':>7} {'in-path':>8} "
              f"{'flips':>7} {'closest cm':>11}")
        print("-" * 56)
        for value, res in rows:
            worst = f"{res['worst']:+.1f}" if res["worst"] is not None else "-"
            print(f"{value:>10} {res['contacts']:>9} {res['grazes']:>7} "
                  f"{res['in_path']:>8} {res['flips']:>7} {worst:>11}")
        return

    params = load_params(overrides)
    records = replay(events, params)
    report(records, "replay" + (f" ({', '.join(args.set)})" if args.set else ""),
           args.contact_cm)
    if args.contact:
        show_contacts(records, contact_cm=args.contact_cm)


if __name__ == "__main__":
    main()
