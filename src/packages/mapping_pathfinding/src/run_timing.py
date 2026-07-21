#!/usr/bin/env python3

"""
Measuring how long the bot actually takes to drive each street and cross each
intersection.

ROS-free so it can be unit tested off-robot.

The gate run is the only timed part of the challenge, so the planner is only as
good as its idea of how long things take. A flat "4 seconds per street" cannot
tell a short street from a long one; these are the real numbers, measured
during the (untimed) mapping run and then handed to the cost model.

Everything is timed off the same signal -- the drive-mode transitions published
by switch_control -- so the pieces add up to the whole with no gap and no
overlap:

    STOPPED --------> CROSSING ------> LANE_FOLLOWING -----------> STOPPED
            (crossing starts)   (crossing ends,        (arrived at the next
                                 street starts)          red line)
            |<---- turn time --->|<------- street time ------------>|

A move therefore costs `stop_duration + turn_time + street_time`, which is
exactly what planner.TurnCosts charges.

One subtlety at the start of a run: the bot is placed at an intersection exit,
so its first street has no crossing in front of it to start the clock. Launching
is not a usable start either -- the perception node loads its network first and
the bot stands still for several seconds. The caller therefore starts the first
street on the first actual wheel command; see `start_street`.
"""

# A "street" driven in less time than this is not a street. Guards against a
# spurious stop-line detection right after a crossing, which would otherwise
# record a near-zero time and make the planner think that street is free.
MIN_PLAUSIBLE_STREET_TIME = 0.5

# Likewise, a crossing cannot plausibly take this little.
MIN_PLAUSIBLE_TURN_TIME = 0.2


def median(values):
    """Middle value, averaging the two middle ones for an even count."""
    ordered = sorted(values)
    count = len(ordered)

    if not count:
        return None

    middle = count // 2

    if count % 2:
        return ordered[middle]

    return (ordered[middle - 1] + ordered[middle]) / 2.0


class RunTimer:
    """
    Collects street and turn timings, and reports them as a planner cost model.

    Repeated measurements of the same street are kept and reduced with the
    median rather than replaced: mapping drives some streets more than once, and
    the median shrugs off a single run that was disturbed (a bad crossing, a
    hand on the bot) without needing an outlier rule.
    """

    def __init__(self, default_street_time=4.0, seed_turn_durations=None):
        self.default_street_time = float(default_street_time)
        self.seed_turn_durations = dict(seed_turn_durations or {})

        self.street_samples = {}
        self.turn_samples = {}

        self._street_key = None
        self._street_start = None
        self._turn_start = None

        # Samples thrown away for being implausible, kept for reporting so a
        # silently wrong measurement cannot hide.
        self.rejected = []

    # -- streets ---------------------------------------------------------

    def start_street(self, edge_key, now):
        """Clock starts as the bot leaves an intersection onto `edge_key`."""
        self._street_key = edge_key
        self._street_start = now

    def finish_street(self, now):
        """
        Clock stops when the bot is stopped at the next red line.

        Returns (edge_key, seconds) for a recorded sample, or None when there
        was nothing being timed or the sample was implausible.
        """
        if self._street_key is None or self._street_start is None:
            return None

        key, elapsed = self._street_key, now - self._street_start
        self._street_key = None
        self._street_start = None

        if elapsed < MIN_PLAUSIBLE_STREET_TIME:
            self.rejected.append(("street", key, elapsed))
            return None

        self.street_samples.setdefault(key, []).append(elapsed)

        return key, elapsed

    def abandon_street(self):
        """Drops the street being timed, e.g. when the position was lost."""
        self._street_key = None
        self._street_start = None

    @property
    def timing_street(self):
        return self._street_key

    # -- turns -----------------------------------------------------------

    def start_turn(self, now):
        """
        Clock starts as the bot pulls away from the red line.

        Deliberately takes no direction. switch_control publishes the drive mode
        before the turn direction, so at this instant the direction on the wire
        is still the *previous* crossing's -- the executed direction is only
        reliably known when the crossing ends, and that is where it is supplied.
        """
        self._turn_start = now

    def finish_turn(self, direction, now):
        """Returns (direction, seconds), or None if nothing/implausible."""
        if self._turn_start is None:
            return None

        elapsed = now - self._turn_start
        self._turn_start = None

        if elapsed < MIN_PLAUSIBLE_TURN_TIME:
            self.rejected.append(("turn", direction, elapsed))
            return None

        self.turn_samples.setdefault(direction, []).append(elapsed)

        return direction, elapsed

    # -- results ---------------------------------------------------------

    def street_time(self, edge_key):
        """Measured time for a street, or None if it has not been driven."""
        samples = self.street_samples.get(edge_key)
        return median(samples) if samples else None

    def edge_durations(self):
        """Measured street times, for planner.TurnCosts(edge_durations=...)."""
        return {
            key: median(samples)
            for key, samples in self.street_samples.items()
            if samples
        }

    def turn_durations(self):
        """
        Turn timings for the cost model: measured where available, else the
        seed from config.

        A mapping run does not necessarily perform every kind of turn -- a
        five-street coverage route can easily never turn left -- so the seed has
        to stay in place for whatever was not observed.
        """
        durations = dict(self.seed_turn_durations)

        for direction, samples in self.turn_samples.items():
            if samples:
                durations[direction] = median(samples)

        return durations

    def unmeasured(self, edge_keys):
        """Which of `edge_keys` still have no measurement."""
        return sorted(key for key in edge_keys if not self.street_samples.get(key))

    def as_dict(self):
        """Everything measured so far, for /mapping/state and the display."""
        return {
            "street_times": {
                key: round(median(samples), 2)
                for key, samples in sorted(self.street_samples.items())
                if samples
            },
            "street_samples": {
                key: [round(value, 2) for value in samples]
                for key, samples in sorted(self.street_samples.items())
            },
            "turn_times": {
                direction: round(median(samples), 2)
                for direction, samples in sorted(self.turn_samples.items())
                if samples
            },
            "seed_turn_times": dict(self.seed_turn_durations),
            "default_street_time": self.default_street_time,
            "rejected": [
                {"kind": kind, "what": what, "seconds": round(value, 3)}
                for kind, what, value in self.rejected
            ],
        }

    def summary_lines(self, edge_keys=()):
        """Human-readable table for the log at the end of the mapping run."""
        lines = ["Measured street times (seconds):"]

        for key in sorted(set(edge_keys) | set(self.street_samples)):
            samples = self.street_samples.get(key) or []

            if samples:
                detail = ", ".join(f"{value:.2f}" for value in samples)
                lines.append(f"  {key:<12} {median(samples):5.2f}  ({detail})")
            else:
                lines.append(f"  {key:<12}    --  (not measured, "
                             f"assuming {self.default_street_time:.1f})")

        measured = self.turn_durations()
        lines.append("Turn times (seconds, * = measured):")

        for direction in ("LEFT", "STRAIGHT", "RIGHT"):
            samples = self.turn_samples.get(direction) or []
            mark = "*" if samples else " "
            value = measured.get(direction)

            if value is not None:
                lines.append(f"  {direction:<9}{mark}{value:5.2f}"
                             f"  ({len(samples)} sample(s))")

        return lines
