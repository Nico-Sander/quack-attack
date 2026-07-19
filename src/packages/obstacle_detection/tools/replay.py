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
                planner.set_lane_error_trusted(bool(data.get("valid", True)))
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
    half = params["front_slice_half"]

    # Clearance between the robot's own footprint and the nearest RAW detection -
    # raw, not inflated, because the question is whether it would physically touch.
    clearance = None
    nearest = None
    for d in duckies_now:
        try:
            xmin, xmax = float(d["xmin"]), float(d["xmax"])
            ymax = float(d["ymax"])
        except (KeyError, TypeError, ValueError):
            continue
        if xmax < drive_x - half:
            gap = (drive_x - half) - xmax
        elif xmin > drive_x + half:
            gap = xmin - (drive_x + half)
        else:
            gap = -min(xmax - (drive_x - half), (drive_x + half) - xmin)
        if clearance is None or gap < clearance:
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
    return (r["clearance"] is not None and r["clearance"] < 0
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
               if r["clearance"] is not None and r["clearance"] < 0
               and not is_contact(r, contact_cm)]
    # Closest approach considering only frames near enough to matter.
    near = [r for r in records
            if r["near_cm"] is not None and r["near_cm"] <= contact_cm * 2.5]
    worst = min(near, key=lambda r: r["clearance"]) if near else None

    dt = 1.0 / STEP_HZ
    print(f"\n=== {label} ===")
    print(f"  steps {len(records)}  ({len(records) * dt:.1f} s)   state flips: {flips}")
    for st, n in sorted(states.items(), key=lambda kv: -kv[1]):
        print(f"    {st:16s} {n * dt:6.1f} s  ({100.0 * n / len(records):4.1f}%)")
    print(f"  CONTACT frames (overlap within {contact_cm:.0f}cm): {len(contacts)}")
    print(f"  in-path but not close (normal avoidance):  {len(in_path)}")
    if worst:
        print(f"  closest approach: clearance {worst['clearance']:+.3f} at "
              f"t={worst['t']}s, duckie ~{worst['near_cm']}cm, "
              f"state={worst['state']}, gap_w={worst['gap_w']}")
        print(f"                    reason={worst['reason']}")
    return {"contacts": len(contacts), "in_path": len(in_path), "flips": flips,
            "worst": worst["clearance"] if worst else None}


def show_contacts(records, window=6, contact_cm=12.0):
    bad = [i for i, r in enumerate(records) if is_contact(r, contact_cm)]
    if not bad:
        print(f"\nNo contact frames (no overlap within {contact_cm:.0f}cm). "
              f"Showing the closest approach instead.")
        near = [r for r in records if r["near_cm"] is not None]
        if not near:
            return
        first = records.index(min(near, key=lambda r: r["clearance"]))
    else:
        first = bad[0]
    lo, hi = max(0, first - window), min(len(records), first + window)
    print(f"\nFirst overlap at t={records[first]['t']}s - surrounding frames:\n")
    print(f"{'t':>6} {'state':14} {'drive_x':>8} {'gap_w':>6} {'clear':>7} "
          f"{'cm':>6} {'v':>6} {'omega':>7}  reason")
    print("-" * 92)
    for r in records[lo:hi]:
        mark = " << CONTACT" if is_contact(r, contact_cm) else ""
        print(f"{r['t']:>6} {r['state']:14} {r['drive_x']:>8} {r['gap_w']:>6} "
              f"{str(r['clearance']):>7} {str(r['near_cm']):>6} {r['v']:>6} "
              f"{r['omega']:>7}  {r['reason']}{mark}")


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
        print(f"\n{'value':>10} {'CONTACT':>9} {'in-path':>8} {'flips':>7} {'closest':>9}")
        print("-" * 48)
        for value, res in rows:
            worst = f"{res['worst']:+.3f}" if res["worst"] is not None else "-"
            print(f"{value:>10} {res['contacts']:>9} {res['in_path']:>8} "
                  f"{res['flips']:>7} {worst:>9}")
        return

    params = load_params(overrides)
    records = replay(events, params)
    report(records, "replay" + (f" ({', '.join(args.set)})" if args.set else ""),
           args.contact_cm)
    if args.contact:
        show_contacts(records, contact_cm=args.contact_cm)


if __name__ == "__main__":
    main()
