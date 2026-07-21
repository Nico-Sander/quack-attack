#!/usr/bin/env python3

"""
Route planning on the city graph.

Because U-turns are forbidden, where the bot can go next depends on how it
entered the intersection. Planning therefore happens over *directed* states
`(node, entry_port)` -- "approaching `node`, having entered via `entry_port`" --
rather than over nodes. That state also identifies the street just traversed
(the one attached to `entry_port`), which is what lets the planner target
individual streets.

Transitions are weighted with an estimated duration so the search prefers
faster routes: the gate run is the only timed part of the challenge, and a
right turn is markedly cheaper than a left one.

This module is ROS-free and unit tested off-robot.
"""

import heapq

from city_map import DIRECTIONS, edge_key, exit_port


# Fallback timings, overridden from config.json via TurnCosts.from_config().
DEFAULT_STOP_DURATION = 3.0
DEFAULT_TURN_DURATIONS = {"LEFT": 2.4, "STRAIGHT": 2.0, "RIGHT": 1.2}
DEFAULT_EDGE_DURATION = 4.0


class TurnCosts:
    """Estimated seconds for each part of a move between two intersections."""

    def __init__(self, turn_durations=None, stop_duration=DEFAULT_STOP_DURATION,
                 edge_duration=DEFAULT_EDGE_DURATION, edge_durations=None):
        self.turn_durations = dict(DEFAULT_TURN_DURATIONS)
        self.turn_durations.update(turn_durations or {})

        self.stop_duration = stop_duration
        self.edge_duration = edge_duration
        # Optional per-street overrides; streets differ in length on a real track.
        self.edge_durations = dict(edge_durations or {})

    @classmethod
    def from_config(cls, config, edge_durations=None, turn_durations=None):
        """
        Builds the cost model from config.json.

        Turn durations come from the `planner` section, which holds times
        *measured on the track*. They are deliberately not taken from
        `switch_control.timers.turn_durations`: those are crossing **timeouts**,
        an upper bound the crossing normally ends well before, so using them
        would systematically overcharge every turn. `stop_duration` is shared,
        because the stop really does last exactly that long.

        `turn_durations` overrides the config, and is how live measurements from
        the mapping run replace the seeded values.
        """
        config = config or {}
        timers = config.get("switch_control", {}).get("timers", {})
        planner_config = config.get("planner", {})

        seeded = dict(planner_config.get("turn_durations")
                      or timers.get("turn_durations") or {})
        seeded.update(turn_durations or {})

        return cls(
            turn_durations=seeded,
            stop_duration=timers.get("stop_duration", DEFAULT_STOP_DURATION),
            edge_duration=planner_config.get("default_street_time",
                                             DEFAULT_EDGE_DURATION),
            edge_durations=edge_durations,
        )

    def cost(self, direction, traversed_edge_key):
        """Cost of crossing an intersection and driving the street beyond it."""
        return (
            self.turn_durations.get(direction, DEFAULT_TURN_DURATIONS["STRAIGHT"])
            + self.edge_durations.get(traversed_edge_key, self.edge_duration)
            + self.stop_duration
        )


class Step:
    """One planned intersection crossing."""

    __slots__ = ("direction", "from_state", "to_state", "edge_key", "cost")

    def __init__(self, direction, from_state, to_state, key, cost):
        self.direction = direction
        self.from_state = from_state
        self.to_state = to_state
        self.edge_key = key
        self.cost = cost

    def __repr__(self):
        return (
            f"Step({self.direction} @ {self.from_state[0]}{self.from_state[1]}"
            f" -> {self.edge_key})"
        )

    def __eq__(self, other):
        return isinstance(other, Step) and self.as_dict() == other.as_dict()

    def as_dict(self):
        return {
            "direction": self.direction,
            "from_state": list(self.from_state),
            "to_state": list(self.to_state),
            "edge_key": self.edge_key,
            "cost": round(self.cost, 3),
        }


class PlanningError(Exception):
    """Raised when no legal route exists."""


# ---------------------------------------------------------------------------
# State-space helpers
# ---------------------------------------------------------------------------

def traversed_edge_key(city, state):
    """The street the bot drove to arrive in `state`."""
    node, entry = state
    neighbour, neighbour_port = city[node][entry]
    return edge_key(node, entry, neighbour, neighbour_port)


def successors(city, state):
    """
    Legal moves out of `state`, as (direction, next_state, edge_key).

    Only ports that actually carry a street are returned, and the direction
    tables never map a port onto itself, so U-turns never appear.
    """
    node, entry = state
    result = []

    for direction in DIRECTIONS:
        port = exit_port(entry, direction)

        if port not in city.get(node, {}):
            continue

        neighbour, neighbour_port = city[node][port]
        result.append(
            (direction, (neighbour, neighbour_port),
             edge_key(node, port, neighbour, neighbour_port))
        )

    return result


def edge_goal_states(city, key):
    """
    The states that mean "just traversed the street `key`".

    A street can be driven from either end, so it has two such states -- which
    is exactly why gates are direction-independent.
    """
    states = [
        (node, port)
        for node, ports in city.items()
        for port in ports
        if traversed_edge_key(city, (node, port)) == key
    ]

    if not states:
        raise PlanningError(f"Unknown street: {key}")

    return set(states)


def all_states(city):
    return {(node, port) for node, ports in city.items() for port in ports}


# ---------------------------------------------------------------------------
# Search
# ---------------------------------------------------------------------------

def plan(city, start_state, goal_states, costs=None, forbidden_edge_keys=None,
         require_move=False):
    """
    Cheapest legal route from `start_state` to any state in `goal_states`.

    Returns a list of Steps, empty if the bot is already at a goal (i.e. it is
    currently driving the street it was asked to reach). Raises PlanningError
    when no route exists.

    `forbidden_edge_keys` blocks streets the route should not traverse; the
    start street itself is never blocked, since the bot is already on it.

    `require_move` forces a route of at least one crossing, and is what makes a
    street the bot has *already finished* drivable again. A state means "just
    traversed the street attached to entry_port", which is true both while
    driving that street and while stopped at the red line at its end -- so
    without this, asking to drive the street underneath the bot is answered
    with "you are already there" even when the bot is at the far end of it and
    the gate is behind it.
    """
    costs = costs or TurnCosts()
    goal_states = set(goal_states)
    blocked = set(forbidden_edge_keys or ())

    if not goal_states:
        raise PlanningError("No goal states given")

    if start_state in goal_states and not require_move:
        return []

    # (cost so far, tie-breaker, state) -- the counter keeps heapq from ever
    # having to compare states.
    counter = 0
    queue = []
    best = {}
    came_from = {}

    if require_move:
        # Seed with the successors instead of the start state. The start state
        # is then an ordinary unvisited node, so the search can come back round
        # to it -- which is exactly the route wanted when the street has to be
        # re-driven in the same direction.
        for direction, next_state, key in successors(city, start_state):
            if key in blocked:
                continue

            step_cost = costs.cost(direction, key)

            if step_cost < best.get(next_state, float("inf")):
                best[next_state] = step_cost
                came_from[next_state] = (start_state, direction, key, step_cost)
                counter += 1
                heapq.heappush(queue, (step_cost, counter, next_state))
    else:
        best[start_state] = 0.0
        queue.append((0.0, counter, start_state))

    while queue:
        cost_so_far, _, state = heapq.heappop(queue)

        if cost_so_far > best.get(state, float("inf")):
            continue

        if state in goal_states:
            return _reconstruct(came_from, state, stop_at=start_state)

        for direction, next_state, key in successors(city, state):
            if key in blocked:
                continue

            new_cost = cost_so_far + costs.cost(direction, key)

            if new_cost < best.get(next_state, float("inf")):
                best[next_state] = new_cost
                came_from[next_state] = (state, direction, key, new_cost - cost_so_far)
                counter += 1
                heapq.heappush(queue, (new_cost, counter, next_state))

    raise PlanningError(
        f"No route from {start_state} to any of {sorted(goal_states)}"
    )


def _reconstruct(came_from, goal_state, stop_at=None):
    """
    Walks the parent chain back into a list of Steps.

    `stop_at` terminates the walk at the start state. It matters only in
    require_move searches, where the start state can itself have a parent (the
    route loops back to it) and an unguarded walk would go round forever.
    """
    steps = []
    state = goal_state

    while state in came_from:
        previous, direction, key, cost = came_from[state]
        steps.append(Step(direction, previous, state, key, cost))
        state = previous

        if stop_at is not None and state == stop_at:
            break

    steps.reverse()
    return steps


def plan_to_edge(city, start_state, target_edge_key, costs=None,
                 forbidden_edge_keys=None, require_move=False):
    """Cheapest route that ends with the street `target_edge_key` traversed."""
    return plan(
        city,
        start_state,
        edge_goal_states(city, target_edge_key),
        costs=costs,
        forbidden_edge_keys=forbidden_edge_keys,
        require_move=require_move,
    )


# ---------------------------------------------------------------------------
# Mission-level planning
# ---------------------------------------------------------------------------

def plan_gate_run(city, start_state, gate_edge_keys, costs=None,
                  strict_order=False, start_street_driven=False):
    """
    Route through the gate streets in the given order.

    Each leg starts where the previous one ended, so the whole run is one
    continuous, U-turn-free route. With `strict_order` the route avoids driving
    through gates that are not yet due; if that makes a leg impossible, the
    constraint is dropped for that leg rather than failing outright.

    `start_street_driven` says the bot has *already finished* the street it is
    standing on -- it is at the red line at the far end, so anything on that
    street is behind it. The first leg then has to re-drive that street rather
    than count it as done. Only the first leg: every later one begins where the
    previous ended, which is a street the bot really has just driven.

    Returns (steps, legs) where `legs` holds the per-gate step counts and costs.
    """
    costs = costs or TurnCosts()
    remaining = list(gate_edge_keys)

    steps = []
    legs = []
    state = start_state

    for index, target in enumerate(remaining):
        upcoming = set(remaining[index + 1:]) if strict_order else set()
        require_move = start_street_driven and index == 0

        try:
            leg = plan_to_edge(city, state, target, costs=costs,
                               forbidden_edge_keys=upcoming,
                               require_move=require_move)
        except PlanningError:
            if not upcoming:
                raise
            # Detouring around later gates is impossible here -- take the
            # direct route rather than giving up on the run.
            leg = plan_to_edge(city, state, target, costs=costs,
                               require_move=require_move)

        steps.extend(leg)
        legs.append({
            "gate_edge_key": target,
            "steps": len(leg),
            "cost": round(sum(step.cost for step in leg), 3),
        })

        if leg:
            state = leg[-1].to_state

    return steps, legs


def all_directed_edges(city):
    """
    Every street in both directions, as (from_node, from_port, to_node, to_port).

    A directed edge is exactly a legal starting placement under the run
    convention: the bot is set down at an intersection exit, facing along the
    street towards the next intersection.
    """
    return sorted(
        (node, port, neighbour, neighbour_port)
        for node, ports in city.items()
        for port, (neighbour, neighbour_port) in ports.items()
    )


def recommend_start_edges(city, gate_edge_keys, costs=None, strict_order=False,
                          limit=3):
    """
    Where to place the bot for the fastest gate run.

    The run is timed from the moment it starts driving, and it starts at an
    intersection exit -- so the street it is placed on is driven in full and its
    time counts. The best placement is usually one whose own street is the first
    gate, which gets that gate for free.

    Returns a list of dicts sorted cheapest first, each with the placement as
    both a tuple and the `from,port,to,port` string the launch file takes.
    """
    costs = costs or TurnCosts()
    targets = list(gate_edge_keys)

    if not targets:
        return []

    options = []

    for edge in all_directed_edges(city):
        from_node, from_port, to_node, to_port = edge
        start_key = edge_key(from_node, from_port, to_node, to_port)

        try:
            steps, _legs = plan_gate_run(city, (to_node, to_port), targets,
                                         costs=costs, strict_order=strict_order)
        except PlanningError:
            continue

        # The street the bot is placed on is driven before the first
        # intersection, so its time is part of the run whether or not it
        # carries a gate.
        approach = costs.edge_durations.get(start_key, costs.edge_duration)

        options.append({
            "start_edge": list(edge),
            "start_edge_arg": f"{from_node},{from_port},{to_node},{to_port}",
            "start_edge_key": start_key,
            "first_gate_on_start_street": bool(targets and targets[0] == start_key),
            "steps": len(steps),
            "turns": turn_sequence(steps),
            "cost": round(approach + total_cost(steps), 3),
        })

    # Ties broken by fewer crossings, then by name, so the recommendation is
    # stable between runs rather than depending on dict ordering.
    options.sort(key=lambda option: (option["cost"], option["steps"],
                                     option["start_edge_arg"]))

    return options[:limit] if limit else options


def plan_exploration_step(city, start_state, unvisited_edge_keys, costs=None):
    """
    Route to the nearest street that has not been driven yet.

    Coverage is what matters during mapping (the phase is untimed), and since a
    gate tag spans the whole street, traversing every street once in either
    direction is enough to find every gate. Returns None when everything has
    been covered.
    """
    costs = costs or TurnCosts()
    targets = set(unvisited_edge_keys)

    if not targets:
        return None

    goal_states = set()

    for key in targets:
        try:
            goal_states |= edge_goal_states(city, key)
        except PlanningError:
            continue

    if not goal_states:
        return None

    try:
        return plan(city, start_state, goal_states, costs=costs)
    except PlanningError:
        return None


# ---------------------------------------------------------------------------
# Presentation helpers
# ---------------------------------------------------------------------------

def turn_sequence(steps):
    """The planned turns, in order -- what switch_control ultimately consumes."""
    return [step.direction for step in steps]


def route_edge_keys(steps):
    """Streets the route drives, in order -- used to highlight the path."""
    return [step.edge_key for step in steps]


def total_cost(steps):
    return round(sum(step.cost for step in steps), 3)


def steps_as_dicts(steps):
    return [step.as_dict() for step in steps]
