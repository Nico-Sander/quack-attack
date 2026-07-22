#!/usr/bin/env python3

"""
Deciding when a gate sighting is trustworthy enough to write onto the map.

ROS-free so it can be unit tested off-robot.

A gate is written onto a street *permanently* (see GraphMap.record_gate), which
makes a wrong sighting unrecoverable for the rest of the run. Two independent
guards stand in front of that write:

  size    A tag seen across an intersection, or far down a side street, belongs
          to a street the bot is not on. It is further away, so its detected
          area in the image is smaller -- area falls off with the square of the
          distance, which makes it a blunt but very effective discriminator.
          `min_area` is the line between "on my street" and "somewhere else".

  streak  A single frame can misdetect. Requiring the same gate on the same
          street in several consecutive detections costs a few hundred
          milliseconds and removes the one-bad-frame failure mode entirely.

The streak is keyed by (edge, gate), so it resets by itself when the bot moves
to another street or a different tag comes into view. It also decays on a
timeout, so sightings separated by a long gap are not silently accumulated into
a confirmation.

On top of both, gates are only mapped while the bot is driving a street at all
-- see mapping_allowed().
"""

from custom_enums import DriveMode


# Gate mapping is meaningless in these modes: stopped at the line and
# mid-crossing the camera points across the intersection, at streets the bot is
# not on. Anything seen there would be stamped onto the current street.
BLOCKED_MODES = (DriveMode.STOPPED, DriveMode.CROSSING_INTERSECTION)

# Minimum detected tag area, in pixels, for a gate to count as being on the
# street the bot is currently driving.
#
# This is only the fallback for when config.json cannot be read. The value in
# use lives in `config/config.json` -> detect_signs.gate_min_area; change it
# there. Both were measured on the track (2026-07-21): a gate on the current
# street reads well above this, one across an intersection well below.
DEFAULT_MIN_AREA = 1500.0

# Consecutive accepted sightings needed before a gate is written to the map.
DEFAULT_CONFIRM_FRAMES = 3

# A streak older than this is stale and starts over. detect_signs runs at 10 Hz,
# so this tolerates a couple of dropped frames but not a gap.
DEFAULT_STREAK_TIMEOUT = 0.6


def mapping_allowed(mode, allow_approaching=True):
    """
    Whether gate sightings should be believed in the given drive mode.

    Approaching a stop line the bot is still on its own street, so a gate near
    the far end of it is still legitimately its own -- that is what
    `allow_approaching` keeps. The area threshold is what rejects the tags
    visible across the intersection from there.
    """
    if mode in BLOCKED_MODES:
        return False

    if mode == DriveMode.APPROACHING_STOP_LINE and not allow_approaching:
        return False

    return True


def is_confident(area, min_area=DEFAULT_MIN_AREA):
    """Whether a detection is big enough -- i.e. close enough -- to trust."""
    return float(area) >= float(min_area)


def largest_accepted(detections, known_sign_ids=()):
    """
    Picks the gate to consider from one frame's detections.

    Intersection signs are not gates and are filtered out. Of the rest, the
    largest accepted detection wins: it is the closest tag, which on a street
    with one gate is the gate the bot is driving towards.

    Returns the detection dict, or None when the frame holds no usable gate.
    """
    candidates = [
        detection for detection in detections
        if detection.get("accepted")
        and int(detection.get("tag_id", -1)) not in known_sign_ids
    ]

    if not candidates:
        return None

    return max(candidates, key=lambda detection: float(detection.get("area", 0.0)))


def gate_run_progress(gate_order, gates_sighted):
    """
    Which gates of the announced order are done, and which is next.

    Deliberately driven by what the bot has actually *seen*, not by where it
    is. The mission's `remaining_gates` shrinks as soon as a gate's street is
    entered, because that is when the route to the following gate has to be
    planned -- but entering a street is not evidence of having passed the gate
    on it. For anything shown to a person, a gate is done when its tag has been
    confirmed on that street.

    Returns (done, next_gate, pending), all in the announced order.
    """
    order = list(gate_order or [])
    seen = set(gates_sighted or ())

    done = [gate for gate in order if gate in seen]
    pending = [gate for gate in order if gate not in seen]

    return done, (pending[0] if pending else None), pending


class GateConfirmer:
    """
    Counts consecutive sightings of one gate on one street.

    Usage: call confirm() for every accepted gate detection; it returns True
    once the same (edge, gate) pair has been seen `confirm_frames` times in a
    row. It keeps returning True while the streak holds, so the caller does not
    have to catch a single edge -- the write it guards is idempotent.
    """

    def __init__(self, confirm_frames=DEFAULT_CONFIRM_FRAMES,
                 streak_timeout=DEFAULT_STREAK_TIMEOUT):
        self.confirm_frames = max(1, int(confirm_frames))
        self.streak_timeout = float(streak_timeout)

        self._key = None
        self._count = 0
        self._last_seen = None

    @property
    def streak(self):
        return self._count

    @property
    def pending(self):
        """The (edge_key, gate_id) currently being counted, or None."""
        return self._key

    def reset(self):
        self._key = None
        self._count = 0
        self._last_seen = None

    def confirm(self, edge_key, gate_id, now):
        """
        Registers one accepted sighting. True once the streak is long enough.
        """
        if edge_key is None:
            self.reset()
            return False

        key = (edge_key, int(gate_id))
        stale = (self._last_seen is not None
                 and (now - self._last_seen) > self.streak_timeout)

        if key != self._key or stale:
            self._key = key
            self._count = 0

        self._count += 1
        self._last_seen = now

        return self._count >= self.confirm_frames
