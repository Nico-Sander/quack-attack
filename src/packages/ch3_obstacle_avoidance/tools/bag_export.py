#!/usr/bin/env python3

"""Export a tuning rosbag to newline-delimited JSON.

Deliberately dumb: this is the only piece that needs ROS, so it does nothing but
flatten (timestamp, topic, payload) and get out of the way. All the analysis lives
in replay.py, which is pure Python and therefore testable off the robot.

Run inside the container:

    python3 tools/bag_export.py crash.bag > crash.jsonl
"""

import argparse
import json
import sys

import rosbag

# Suffixes rather than full names, so the export does not care what the vehicle
# is called.
WANTED = (
    "/detect/lane",
    "/detect/lane_borders",
    "/detect/duckie_BB",
    "/debug/free_path_plan",
)


def kind(topic):
    for suffix in WANTED:
        if topic.endswith(suffix):
            return suffix.rsplit("/", 1)[-1]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    args = ap.parse_args()

    counts = {}
    t0 = None

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, stamp in bag.read_messages():
            name = kind(topic)
            if name is None:
                continue

            t = stamp.to_sec()
            if t0 is None:
                t0 = t

            # /detect/lane is a Float64; the rest are JSON in a String.
            if name == "lane":
                payload = float(msg.data)
            else:
                try:
                    payload = json.loads(msg.data)
                except Exception:
                    continue

            counts[name] = counts.get(name, 0) + 1
            sys.stdout.write(json.dumps({
                "t": round(t - t0, 4), "kind": name, "data": payload}) + "\n")

    for name, n in sorted(counts.items()):
        sys.stderr.write(f"{name:20s} {n:6d} messages\n")
    if t0 is not None:
        sys.stderr.write(f"{'duration':20s} {round(t - t0, 1):6} s\n")


if __name__ == "__main__":
    main()
