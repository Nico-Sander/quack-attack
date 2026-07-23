#!/usr/bin/env python3

"""
Mapping, localization and route planning on the city graph.

This node owns the mission. It tracks where the bot is on the graph, records
gates onto the streets they were seen on, and -- the part that closes the
control loop -- tells switch_control which way to turn at the next
intersection.

Two phases:
  MAPPING   drive every street at least once, so no gate can be missed (untimed)
  GATE_RUN  drive the gates in the announced order (timed)

Localization is graph dead-reckoning: the position advances by one street each
time switch_control reports a completed crossing. The route is replanned after
every move, which is cheap on a city this size and means the bot recovers on
its own once its believed position is corrected.
"""

import json
import os

import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerResponse

import city_map
import gate_detection
import planner
import run_timing
from city_map import CityMapError, GraphMap
from custom_enums import (
    TURN_COMMAND_HALT, TURN_COMMAND_NONE, DriveMode, MissionPhase, TurnDirection
)


CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../config/config.json"
)


def parse_list_param(value, cast=str):
    """
    Accepts either a real list or a comma-separated string.

    roslaunch args arrive as strings, so "A,1,B,1" and ["A", 1, "B", 1] must
    both work. A single value is also accepted, since roslaunch quietly turns
    a one-element list like "7" into the integer 7.
    """
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",") if part.strip()]
    elif isinstance(value, (list, tuple)):
        parts = list(value)
    else:
        parts = [value]

    return [cast(part) for part in parts]


class MappingPathplanningNode:
    def __init__(self):
        rospy.init_node("mapping_pathplanning_node")

        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        base = f"/{self.vehicle_name}"

        # -- configuration -------------------------------------------------
        city_path = rospy.get_param("~city_path", city_map.DEFAULT_CITY_PATH)
        gates_path = rospy.get_param("~gates_path", city_map.DEFAULT_GATES_PATH)

        self.city, _layout = city_map.load_city(city_path)
        self.gate_config = city_map.load_gate_config(gates_path)

        self.config = self._load_config()
        planner_config = self.config.get("planner", {})

        # Times measured during the mapping run replace the seeds from config.
        # The mapping phase is untimed, so measuring costs nothing; the gate run
        # is timed, and is planned against whatever mapping learned.
        self.timer = run_timing.RunTimer(
            default_street_time=planner_config.get("default_street_time", 4.0),
            seed_turn_durations=planner_config.get("turn_durations"),
        )
        self.costs = self._build_costs()

        rospy.loginfo("Loaded city '%s' with %d nodes and %d streets",
                      city_path, len(self.city),
                      len(city_map.all_edge_keys(self.city)))

        start_edge = self._parse_edge(
            rospy.get_param("~start_edge", ["A", 4, "D", 2])
        )
        self.gate_run_start_edge = rospy.get_param("~gate_run_start_edge", "")

        self.graph_map = GraphMap(self.city, start_edge,
                                  gate_config=self.gate_config)

        # -- mission ---------------------------------------------------------
        phase_name = str(rospy.get_param("~mission_phase", "MAPPING")).upper()
        try:
            self.phase = MissionPhase[phase_name]
        except KeyError:
            rospy.logwarn("Unknown mission_phase %r, falling back to MAPPING",
                          phase_name)
            self.phase = MissionPhase.MAPPING

        self.gate_order = parse_list_param(
            rospy.get_param("~gate_order", []), cast=int
        )
        self.strict_gate_order = bool(rospy.get_param("~strict_gate_order", False))
        self.auto_start_gate_run = bool(
            rospy.get_param("~auto_start_gate_run", False)
        )
        self.remaining_gates = list(self.gate_order)

        # Bring-up override: forces every turn, bypassing the planner. Used to
        # verify the command path end to end before trusting the route.
        force_turn = str(rospy.get_param("~force_turn", "NONE")).upper()
        self.force_turn = None
        if force_turn in ("LEFT", "STRAIGHT", "RIGHT"):
            self.force_turn = TurnDirection[force_turn]
            rospy.logwarn("force_turn=%s -- planner output is being ignored",
                          force_turn)

        # -- runtime state ---------------------------------------------------
        self.active_mode = DriveMode.LANE_FOLLOWING
        self.last_turn_direction = TurnDirection.STRAIGHT
        self.next_turn = None
        self.halt = False
        self.planned_steps = []
        self.localization_ok = True
        self.plan_note = "not planned yet"

        # Whether the street underneath the bot still has to be driven.
        #
        # The graph position cannot express this on its own: a state means
        # "just traversed the street attached to entry_port", which is true
        # both while driving that street and while stopped at the red line at
        # its end. Those are opposite situations for a gate on that street --
        # ahead of the bot in the first, behind it in the second. Starting
        # position is an intersection exit by convention, so it starts True.
        self.street_ahead = True
        self.start_recommendations = []

        # The first street of a run has no crossing in front of it to start its
        # clock -- the bot is placed at an intersection exit. Node startup is
        # not a usable substitute either: detect_lane loads its network first,
        # so the bot stands still for several seconds after launch. The clock
        # therefore starts on the first wheel command that actually moves it.
        self.motion_started = False

        # -- gate run stopwatch ----------------------------------------------
        # Wall-clock time of the timed run, kept separately from the per-street
        # measurements: those tile the route, this is the number that is scored.
        self.run_started_at = None
        self.gate_run_estimate = None
        self.gate_run_last_gate_at = None
        self.gate_run_reported = False
        # Published once the run is over, so anything watching /mapping/state
        # gets the result without having to scrape the console.
        self.gate_run_result = None

        # Gates whose tag has actually been confirmed on its street during the
        # run, in the order they were seen. Kept apart from `remaining_gates`,
        # which shrinks on *entering* a street because that is when the route
        # to the next gate has to be planned. Entering a street is not evidence
        # of having passed the gate on it, so anything reported to a person --
        # the map, the scored time -- uses this instead.
        self.gates_sighted = []

        # -- gate mapping ----------------------------------------------------
        # A gate is written to a street once and never overwritten, so these
        # guards decide what gets written at all. See gate_detection.py.
        self.gate_confirmer = gate_detection.GateConfirmer(
            confirm_frames=int(rospy.get_param(
                "~gate_confirm_frames", gate_detection.DEFAULT_CONFIRM_FRAMES
            ))
        )

        # Approaching the line, the bot is still on its own street, so gates
        # near the far end of it are still worth recording -- the area
        # threshold in detect_signs is what rejects the ones across the
        # intersection. Set false if that turns out not to be enough.
        self.map_gates_while_approaching = bool(
            rospy.get_param("~map_gates_while_approaching", True)
        )

        # Mostly redundant now that the map refuses overwrites, but it still
        # keeps a repeatedly re-seen gate from re-triggering replans.
        self.gate_cooldown = float(rospy.get_param("~gate_cooldown", 2.0))
        self.last_logged_gate = None
        self.last_logged_gate_time = 0.0

        # -- ROS interface ---------------------------------------------------
        self.pub_state = rospy.Publisher(
            f"{base}/mapping/state", String, queue_size=1, latch=True
        )
        self.pub_turn_command = rospy.Publisher(
            f"{base}/plan/turn_command", Int32, queue_size=1, latch=True
        )
        # Bumped when the bot has been physically picked up and put down at an
        # intersection exit. switch_control is parked in STOPPED at a red line
        # at that point, and without this it would perform a crossing manoeuvre
        # into a street rather than simply driving off. Latched and counted
        # rather than a one-shot event, so it cannot be missed.
        self.pub_resume = rospy.Publisher(
            f"{base}/plan/resume_driving", Int32, queue_size=1, latch=True
        )
        self.resume_counter = 0
        self.pub_resume.publish(Int32(data=self.resume_counter))

        rospy.Subscriber(f"{base}/switch/turn_direction", Int32,
                         self._cb_turn_direction, queue_size=1)
        rospy.Subscriber(f"{base}/switch/mode", Int32,
                         self._cb_mode, queue_size=1)
        # Gates come from the detection topic, not /detect/sign: it carries the
        # tag area, which is what tells a gate on this street apart from one
        # seen across an intersection.
        rospy.Subscriber(f"{base}/detect/sign_detections", String,
                         self._cb_sign_detections, queue_size=1)
        # Only to notice the moment the bot first moves; see motion_started.
        rospy.Subscriber(f"{base}/lane_controller_node/car_cmd", Twist2DStamped,
                         self._cb_car_cmd, queue_size=1)

        self.srv_start_gate_run = rospy.Service(
            "~start_gate_run", Trigger, self._srv_start_gate_run
        )

        # The command is republished continuously so switch_control can treat a
        # silent planner as "no opinion" instead of trusting a stale value.
        self.command_timer = rospy.Timer(rospy.Duration(0.1),
                                         self._publish_turn_command)

        self._replan(reason="start")

        rospy.loginfo("mapping_pathplanning_node started in %s on edge %s, "
                      "namespace /%s",
                      self.phase.name, self.graph_map.current_edge_key,
                      self.vehicle_name)

        if self.vehicle_name == "default_robot":
            rospy.logwarn("VEHICLE_NAME is not set, so topics live under "
                          "/default_robot and will not reach the robot. "
                          "Start the container with ./start.sh <vehicle>.")

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    def _load_config(self):
        try:
            with open(CONFIG_PATH, "r") as handle:
                return json.load(handle)
        except (IOError, ValueError) as error:
            rospy.logwarn("Could not read %s (%s); using default turn costs",
                          CONFIG_PATH, error)
            return {}

    def _build_costs(self):
        """
        Rebuilds the cost model from whatever has been measured so far.

        Called after every new measurement, so the route the planner picks is
        always based on the best timings available at that moment.
        """
        return planner.TurnCosts.from_config(
            self.config,
            edge_durations=self.timer.edge_durations(),
            turn_durations=self.timer.turn_durations(),
        )

    def _parse_edge(self, value):
        parts = parse_list_param(value)

        if len(parts) != 4:
            raise CityMapError(
                f"Edge parameter must have 4 elements, got {value!r}"
            )

        return (str(parts[0]), int(parts[1]), str(parts[2]), int(parts[3]))

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _replan(self, reason):
        """
        Recomputes the route from the current position and updates the turn
        command. Called after every move, so a corrected position immediately
        produces a corrected route.
        """
        self.planned_steps = []
        self.halt = False

        if self.force_turn is not None:
            self.next_turn = self.force_turn
            self.plan_note = "forced turn (bring-up override)"

        elif not self.localization_ok:
            self.next_turn = None
            self.plan_note = "localization lost -- planner stood down"

        elif self.phase == MissionPhase.DONE:
            # Halt rather than fall silent: a silent planner hands the bot to
            # the random fallback, and it would drive on after the timed run.
            self.next_turn = None
            self.halt = True
            self.plan_note = "mission complete -- holding"

        elif self.phase == MissionPhase.AWAITING_GATE_RUN:
            self.next_turn = None
            self.halt = True
            self.plan_note = "mapping complete -- waiting for the gate run"

        elif self.phase == MissionPhase.MAPPING:
            self._plan_exploration()

        elif self.phase == MissionPhase.GATE_RUN:
            self._plan_gate_run()

        # A planned turn the graph does not allow would desynchronise the
        # position on the next move, so it is dropped rather than sent.
        if (self.next_turn is not None
                and not self.graph_map.is_direction_available(self.next_turn)):
            rospy.logerr("Planned turn %s is not available at %s port %d -- "
                         "dropping command",
                         self.next_turn.name, self.graph_map.current_node(),
                         self.graph_map.entry_port())
            self.next_turn = None
            self.plan_note = "planned turn unavailable"

        self._publish_turn_command()
        self._publish_state(reason=reason)

    def _plan_exploration(self):
        unvisited = self.graph_map.unvisited_edge_keys()

        if not unvisited:
            # Every street has been entered. Do not cross another intersection:
            # drive out the street underneath the bot -- which is what times it
            # -- and stop at the red line at its end. _handle_arrived_at_line
            # turns that stop into the end of the mapping phase.
            self.next_turn = None
            self.halt = True
            self.plan_note = "every street driven -- stopping at the next red line"
            return

        steps = planner.plan_exploration_step(
            self.city, self.graph_map.current_state_tuple(), unvisited,
            costs=self.costs
        )

        if not steps:
            self.next_turn = None
            self.plan_note = "no route to any unvisited street"
            return

        self.planned_steps = steps
        self.next_turn = TurnDirection[steps[0].direction]
        self.plan_note = (
            f"exploring towards {steps[-1].edge_key} "
            f"({len(unvisited)} street(s) left)"
        )

    def _plan_gate_run(self):
        # The bot may already be standing on the next gate's street -- at the
        # start of the run, or after re-seeding the position. Tick those off
        # first, otherwise the run would plan zero steps and idle forever.
        self._update_gate_progress()

        if not self.remaining_gates:
            # The clock for "last gate passed" is stopped by the sighting, in
            # _mark_gate_sighted -- not here. This point is only the last gate's
            # street being *entered*, which is up to a whole street too early.
            self.phase = MissionPhase.DONE
            self.next_turn = None
            # Set here, not only in _replan's DONE branch: the phase changes
            # *during* a replan that already decided halt=False, so leaving it
            # to the next replan would let the bot drive on after the timed run.
            self.halt = True
            self.plan_note = "gate run complete"
            return

        edge_keys = []
        missing = []

        for gate_id in self.remaining_gates:
            key = self.graph_map.edge_key_for_gate(gate_id)

            if key is None:
                missing.append(gate_id)
            else:
                edge_keys.append(key)

        if missing:
            self.next_turn = None
            self.plan_note = f"gate(s) {missing} never mapped -- cannot plan run"
            rospy.logerr_throttle(10.0, "Cannot plan gate run: gate(s) %s were "
                                        "not found during mapping", missing)
            return

        try:
            steps, legs = planner.plan_gate_run(
                self.city, self.graph_map.current_state_tuple(), edge_keys,
                costs=self.costs, strict_order=self.strict_gate_order,
                # At a red line the current street is already behind the bot,
                # so a gate on it has to be reached by driving round to it.
                start_street_driven=not self.street_ahead,
            )
        except planner.PlanningError as error:
            self.next_turn = None
            self.plan_note = f"no legal gate route: {error}"
            rospy.logerr_throttle(10.0, "Gate run planning failed: %s", error)
            return

        self.planned_steps = steps
        self.plan_note = (
            f"gate run: {self.remaining_gates} "
            f"({planner.total_cost(steps):.1f}s estimated)"
        )

        if not steps:
            # Already on the last gate's street.
            self.next_turn = None
            return

        self.next_turn = TurnDirection[steps[0].direction]

    def _enter_gate_run(self):
        self.phase = MissionPhase.GATE_RUN
        self.remaining_gates = list(self.gate_order)

        # Restart the stopwatch. Both ways in come through here -- the service
        # and auto_start_gate_run -- and neither may inherit the mapping run's
        # clock.
        self.timer.abandon_street()
        self.gate_run_last_gate_at = None
        self.gate_run_reported = False
        self.gate_run_result = None
        self.gates_sighted = []

        if self.gate_run_start_edge:
            # A start edge means the bot has been picked up and set down at an
            # intersection exit, per the run convention. It is therefore no
            # longer at the red line switch_control thinks it is stopped at, and
            # has to resume by driving the street rather than by crossing.
            try:
                self.graph_map.start_on_edge(
                    self._parse_edge(self.gate_run_start_edge)
                )
                rospy.loginfo("Gate run re-seeded on edge %s",
                              self.graph_map.current_edge_key)
                # Set down at an intersection exit, so the whole street is
                # ahead again and a gate on it will genuinely be driven.
                self.street_ahead = True
                self._request_resume_driving()
            except CityMapError as error:
                rospy.logerr("Invalid gate_run_start_edge: %s", error)

        if self.gate_run_start_edge:
            # Repositioned and standing still: the clock starts when the wheels
            # first turn, the same way the mapping run's first street is timed.
            self.motion_started = False
            self.run_started_at = None
        else:
            # Carrying on from the red line it was already holding at, so it is
            # about to pull away and there is no first-motion edge to wait for.
            self.run_started_at = rospy.Time.now().to_sec()

        rospy.loginfo("Entering GATE_RUN for gates %s", self.remaining_gates)

    def _request_resume_driving(self):
        """Tells switch_control the bot has been moved and should just drive."""
        self.resume_counter += 1
        self.pub_resume.publish(Int32(data=self.resume_counter))
        rospy.loginfo("Bot was repositioned -- resuming in lane following, "
                      "not with a crossing")

    def _srv_start_gate_run(self, _request):
        """
        Starts the timed run, from wherever the bot has just been placed.

        The announced order and the placement are re-read from the parameters
        here rather than only at launch, so both can be set after mapping has
        finished -- which is the whole point of holding at the red line. See
        docs/workflow.md for the exact commands.
        """
        self.gate_order = parse_list_param(
            rospy.get_param("~gate_order", self.gate_order), cast=int
        )
        self.gate_run_start_edge = rospy.get_param(
            "~gate_run_start_edge", self.gate_run_start_edge
        )
        self.strict_gate_order = bool(
            rospy.get_param("~strict_gate_order", self.strict_gate_order)
        )

        if not self.gate_order:
            return TriggerResponse(
                success=False,
                message="No gate_order set -- announce the order first "
                        "(rosparam set ~gate_order '7,5,6')"
            )

        unmapped = [gate for gate in self.gate_order
                    if self.graph_map.edge_key_for_gate(gate) is None]

        if unmapped:
            return TriggerResponse(
                success=False,
                message=f"Gate(s) {unmapped} not mapped yet"
            )

        if self.gate_run_start_edge:
            try:
                self._parse_edge(self.gate_run_start_edge)
            except (CityMapError, ValueError) as error:
                return TriggerResponse(
                    success=False,
                    message=f"Invalid gate_run_start_edge: {error}"
                )

        self._enter_gate_run()
        self._replan(reason="gate_run_started")

        # The estimate covers the whole route including driving out the last
        # street and stopping, which is what _report_gate_run compares against.
        self.gate_run_estimate = planner.total_cost(self.planned_steps)

        return TriggerResponse(
            success=True,
            message=f"Gate run started for {self.remaining_gates} from "
                    f"{self.graph_map.current_edge_key} "
                    f"({self.gate_run_estimate:.1f}s estimated)"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _cb_turn_direction(self, msg):
        """The turn switch_control actually committed to, used for the move."""
        try:
            self.last_turn_direction = TurnDirection(msg.data)
        except ValueError:
            rospy.logwarn("Invalid turn direction: %s", msg.data)

    def _cb_car_cmd(self, msg):
        """
        Starts the first street's clock the moment the wheels actually turn.

        The bot is placed at an intersection exit, so its first street has no
        crossing in front of it to start the clock, and node startup is too
        early -- the perception network loads first. This is the real "it is
        driving now" signal.
        """
        if self.motion_started or abs(msg.v) <= 0.0:
            return

        now = rospy.Time.now().to_sec()
        self.motion_started = True
        self.run_started_at = now
        self.timer.start_street(self.graph_map.current_edge_key, now)
        rospy.loginfo("Run started on %s -- timing from here",
                      self.graph_map.current_edge_key)

    def _cb_mode(self, msg):
        try:
            new_mode = DriveMode(msg.data)
        except ValueError:
            rospy.logwarn("Invalid drive mode: %s", msg.data)
            return

        previous = self.active_mode

        if previous == new_mode:
            return

        self.active_mode = new_mode

        # Every timing boundary is one of these three transitions, which is what
        # keeps stop + turn + street adding up to the whole move.
        if new_mode == DriveMode.STOPPED:
            self._handle_arrived_at_line()

        elif (previous == DriveMode.STOPPED
                and new_mode == DriveMode.CROSSING_INTERSECTION):
            self.timer.start_turn(rospy.Time.now().to_sec())

        elif (previous == DriveMode.CROSSING_INTERSECTION
                and new_mode == DriveMode.LANE_FOLLOWING):
            self._handle_crossing_finished()

    def _handle_arrived_at_line(self):
        """The bot is stopped at a red line: the street it just drove is timed."""
        # The street is behind the bot now, so anything on it has been passed.
        self.street_ahead = False

        recorded = self.timer.finish_street(rospy.Time.now().to_sec())

        if recorded is not None:
            key, seconds = recorded
            rospy.loginfo("Street %s driven in %.2fs", key, seconds)
            self.costs = self._build_costs()

        # Must run before the run is reported: it is what guarantees the last
        # gate is credited even if its tag was never caught.
        self._sight_gate_on_current_street()

        # Mapping is only finished once the bot is standing at a red line with
        # every street driven -- that is the point at which every street has
        # also been timed, and the point the bot may be picked up from.
        if self.phase == MissionPhase.MAPPING and self._mapping_finished():
            self._finish_mapping()
            return

        # The gate run ends here too: the last gate is counted as passed when
        # its street is entered, but the bot still has to drive that street.
        if self.phase == MissionPhase.DONE and not self.gate_run_reported:
            self._report_gate_run()

        self._publish_state(reason="arrived_at_line")

    def _report_gate_run(self):
        """
        Reports how long the timed run actually took.

        Two numbers, because they answer different questions. The scored one is
        the last gate being passed. The one to compare against the planner's
        estimate is the bot standing at the red line beyond it, because that is
        the boundary the cost model is built from -- every leg it charges ends
        with a stop. Comparing the estimate against the other number would make
        the planner look optimistic by one street every time.
        """
        self.gate_run_reported = True
        now = rospy.Time.now().to_sec()

        if self.run_started_at is None:
            rospy.logwarn("Gate run finished, but the bot never moved -- "
                          "no time to report")
            return

        to_last_gate = (self.gate_run_last_gate_at or now) - self.run_started_at
        to_stop = now - self.run_started_at

        self.gate_run_result = {
            "gates": list(self.gate_order),
            "time_to_last_gate": round(to_last_gate, 1),
            "time_to_stop": round(to_stop, 1),
            "estimate": self.gate_run_estimate,
        }

        rospy.loginfo("=" * 62)
        rospy.loginfo("GATE RUN COMPLETE")
        rospy.loginfo("  gates driven : %s", self.gate_order)
        rospy.loginfo("  TIME         : %.1f s   (first movement -> last gate "
                      "seen)", to_last_gate)
        rospy.loginfo("  at stop line : %.1f s   (+%.1f s driving out the last "
                      "street)", to_stop, to_stop - to_last_gate)

        if self.gate_run_estimate:
            error = to_stop - self.gate_run_estimate
            rospy.loginfo("  planner said : %.1f s   (%+.1f s, %+.0f%%)",
                          self.gate_run_estimate, error,
                          100.0 * error / self.gate_run_estimate)

        rospy.loginfo("=" * 62)

    def _mapping_finished(self):
        """Every street driven, and every one of them timed."""
        if self.graph_map.unvisited_edge_keys():
            return False

        return not self.timer.unmeasured(self.graph_map.edges)

    def _finish_mapping(self):
        """
        Mapping is over: the bot stays stopped at this red line.

        It deliberately does not roll on into the gate run. The bot is picked up
        here and placed wherever the gate run should start, which is a different
        street in general -- so the run has to wait for an explicit command.
        """
        self.phase = MissionPhase.AWAITING_GATE_RUN
        self.timer.abandon_street()

        rospy.loginfo("=" * 62)
        rospy.loginfo("MAPPING COMPLETE -- holding at this red line on %s",
                      self.graph_map.current_edge_key)

        for line in self.timer.summary_lines(self.graph_map.edges):
            rospy.loginfo(line)

        gates = self.graph_map.gate_edges()
        rospy.loginfo("Gates found (%d):", len(gates))
        for gate in gates:
            rospy.loginfo("  gate %-3d on %s (%s)", gate["gate_id"],
                          gate["edge_key"], gate["gate_colour"] or "?")

        self._report_start_recommendations()

        rospy.loginfo("The bot will not move again until the gate run is "
                      "started -- see docs/workflow.md")
        rospy.loginfo("=" * 62)

        if self.auto_start_gate_run and self.gate_order:
            rospy.loginfo("auto_start_gate_run is set; starting immediately")
            self._enter_gate_run()

        self._replan(reason="mapping_complete")

    def _report_start_recommendations(self):
        """
        Where to place the bot for the fastest gate run, if the order is known.

        Only useful once the map carries measured street times, which is exactly
        the state mapping has just reached.
        """
        self.start_recommendations = []

        if not self.gate_order:
            rospy.loginfo("No gate_order given yet, so no start position can "
                          "be recommended. Pass gate_order when starting the "
                          "run and the route will be planned then.")
            return

        edge_keys = [self.graph_map.edge_key_for_gate(gate)
                     for gate in self.gate_order]

        if any(key is None for key in edge_keys):
            missing = [gate for gate, key in zip(self.gate_order, edge_keys)
                       if key is None]
            rospy.logwarn("Cannot recommend a start position: gate(s) %s were "
                          "not found", missing)
            return

        self.start_recommendations = planner.recommend_start_edges(
            self.city, edge_keys, costs=self.costs,
            strict_order=self.strict_gate_order,
        )

        if not self.start_recommendations:
            return

        rospy.loginfo("Best start positions for gate order %s:", self.gate_order)

        for rank, option in enumerate(self.start_recommendations, start=1):
            note = (" (first gate is on this street)"
                    if option["first_gate_on_start_street"] else "")
            rospy.loginfo("  %d. start_edge:=%-10s %5.1fs, %d crossing(s)%s",
                          rank, option["start_edge_arg"], option["cost"],
                          option["steps"], note)

    def _handle_crossing_finished(self):
        executed = self.last_turn_direction
        measured = self.timer.finish_turn(executed.name,
                                          rospy.Time.now().to_sec())

        if measured is not None:
            rospy.logdebug("Turn %s took %.2fs", *measured)
            self.costs = self._build_costs()

        result = self.graph_map.move(executed)

        if result["success"]:
            rospy.loginfo("Moved %s: now on %s", result["direction"],
                          result["new_edge_key"])
            # Freshly on a street, at its start: the whole of it is ahead.
            self.street_ahead = True
            # The street clock starts here: the bot has left the intersection.
            self.timer.start_street(result["new_edge_key"],
                                    rospy.Time.now().to_sec())
            # Gate progress is ticked off inside _plan_gate_run below, so there
            # is exactly one place that decides a gate has been passed.
        else:
            # Position unknown, so there is nothing to attribute a street time
            # to; drop the measurement rather than filing it under a guess.
            self.timer.abandon_street()

            # The bot turned somewhere the graph says it could not. The believed
            # position is now unreliable, so the planner stands down and
            # switch_control falls back to sign-based driving rather than the
            # bot freezing on the track.
            self.localization_ok = False
            rospy.logerr("Localization lost: %s (tried %s at %s, available %s)",
                         result.get("reason"), result.get("direction"),
                         result.get("node"), result.get("available"))

        self._replan(reason="crossing_finished")
        self._publish_state(reason="crossing_finished", move_result=result)

    def _mark_gate_sighted(self, gate_id, how):
        """
        Records that a gate of the announced order has actually been passed.

        Only counts on the street the gate is mapped to: a tag confirmed while
        the bot is elsewhere is either a misdetection or a gate further along
        the route, and crediting it would tick off a gate never driven.
        """
        if self.phase not in (MissionPhase.GATE_RUN, MissionPhase.DONE):
            return

        if gate_id not in self.gate_order or gate_id in self.gates_sighted:
            return

        if self.graph_map.edge_key_for_gate(gate_id) != self.graph_map.current_edge_key:
            return

        self.gates_sighted.append(gate_id)
        rospy.loginfo("Gate %d passed on %s (%s) -- %d of %d",
                      gate_id, self.graph_map.current_edge_key, how,
                      len(self.gates_sighted), len(self.gate_order))

        # The scored time is to the last gate actually passed, not to the
        # moment its street was entered.
        if len(self.gates_sighted) == len(self.gate_order):
            self.gate_run_last_gate_at = rospy.Time.now().to_sec()

        # Publish immediately. Without this the map would not show the gate as
        # passed until the next state message, which is the arrival at the red
        # line at the end of the street -- so it would mark it in the right
        # order but still at the wrong moment.
        self._publish_state(reason="gate_passed", tag_id=gate_id)

    def _sight_gate_on_current_street(self):
        """
        Safety net: the bot has now driven this street end to end.

        Whatever the camera did or did not catch, the gate on this street has
        physically been passed. Without this a single missed detection would
        leave a gate marked pending forever, and the run would never report a
        result -- worse than crediting it a street too late.
        """
        gate_id = self.graph_map.edges.get(
            self.graph_map.current_edge_key, {}
        ).get("gate_id")

        if gate_id is not None:
            self._mark_gate_sighted(gate_id, "street driven")

    def _update_gate_progress(self):
        """
        Ticks off gates whose street the bot is about to drive.

        Only when the street is still ahead of it. Standing at the red line at
        the end of a street, a gate on that street is *behind* the bot -- it was
        driven during mapping, not during the timed run -- so counting it there
        would award a gate that never gets driven.

        Loops because two gates could in principle sit on the same street; it
        stops at the first gate that is somewhere else.
        """
        if not self.street_ahead:
            return

        while self.remaining_gates:
            expected_key = self.graph_map.edge_key_for_gate(self.remaining_gates[0])

            if expected_key != self.graph_map.current_edge_key:
                return

            done = self.remaining_gates.pop(0)
            rospy.loginfo("Passed gate %d on %s -- %d left",
                          done, expected_key, len(self.remaining_gates))

    def _cb_sign_detections(self, msg):
        """
        Gate mapping. Runs on every frame's detections, accepted or not.

        Three filters in front of the map, from cheapest to strictest: the bot
        must be driving a street, the tag must be close enough to be on it
        (`accepted`, decided in detect_signs), and it must have been seen
        several frames running. Only then is it written -- and once written it
        is never replaced.
        """
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return

        if not gate_detection.mapping_allowed(
            self.active_mode, allow_approaching=self.map_gates_while_approaching
        ):
            # Drop any part-built streak: it belongs to what the bot could see
            # before it stopped, and the next street is a different question.
            self.gate_confirmer.reset()
            return

        candidate = gate_detection.largest_accepted(
            payload.get("detections", []),
            known_sign_ids=city_map.KNOWN_SIGN_IDS,
        )

        if candidate is None:
            return

        tag_id = int(candidate["tag_id"])
        now = rospy.Time.now().to_sec()

        if not self.gate_confirmer.confirm(
            self.graph_map.current_edge_key, tag_id, now
        ):
            return

        # Before the cooldown, which exists to stop repeated map writes and
        # must not be able to swallow the one sighting that proves a gate was
        # driven.
        self._mark_gate_sighted(tag_id, "seen")

        if (self.last_logged_gate == tag_id
                and now - self.last_logged_gate_time < self.gate_cooldown):
            return

        self.last_logged_gate = tag_id
        self.last_logged_gate_time = now

        self._record_confirmed_gate(tag_id, candidate)

    def _record_confirmed_gate(self, tag_id, detection):
        """Writes a confirmed gate to the map and reports what came of it."""
        edge = self.graph_map.current_edge_key
        result = self.graph_map.record_gate(tag_id)

        if result == city_map.GATE_ALREADY_SET:
            return

        if result == city_map.GATE_EDGE_LOCKED:
            # The street's gate was mapped earlier, from closer up. This is the
            # sighting that used to overwrite it.
            rospy.loginfo_throttle(
                5.0, "Ignoring gate %d on %s: already mapped as gate %s",
                tag_id, edge, self.graph_map.edges[edge]["gate_id"]
            )
            return

        if result == city_map.GATE_ELSEWHERE:
            rospy.loginfo_throttle(
                5.0, "Ignoring gate %d seen on %s: already mapped on %s",
                tag_id, edge, self.graph_map.edge_key_for_gate(tag_id)
            )
            return

        if result != city_map.GATE_RECORDED:
            rospy.logwarn("Could not record gate %d (%s)", tag_id, result)
            return

        colour = self.gate_config.get(tag_id, {}).get("colour", "unknown")
        rospy.loginfo("Gate %d (%s) recorded on %s (area %.0f px)",
                      tag_id, colour, edge, detection.get("area", 0.0))

        # A new gate changes the map, so the route may change too -- most
        # importantly, this is what lets the gate run start as soon as the last
        # missing gate turns up rather than one intersection later.
        self._replan(reason="gate_detected")
        self._publish_state(reason="gate_detected", tag_id=tag_id)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_turn_command(self, _event=None):
        if self.halt:
            value = TURN_COMMAND_HALT
        elif self.next_turn is None:
            value = TURN_COMMAND_NONE
        else:
            value = self.next_turn.value

        self.pub_turn_command.publish(Int32(data=value))

    def _publish_state(self, reason, tag_id=None, move_result=None):
        state = {
            "reason": reason,
            "stamp": rospy.Time.now().to_sec(),
            "tag_id": tag_id,
            "active_mode": self.active_mode.name,
            "last_turn_direction": self.last_turn_direction.name,

            "mission": {
                "phase": self.phase.name,
                "gate_order": self.gate_order,
                "remaining_gates": self.remaining_gates,
                "localization_ok": self.localization_ok,
                "halted": self.halt,
                "gate_run_estimate": self.gate_run_estimate,
                "gate_run_result": self.gate_run_result,
                "gates_sighted": list(self.gates_sighted),
                "gate_run_elapsed": (
                    None if self.run_started_at is None
                    else round(rospy.Time.now().to_sec() - self.run_started_at, 1)
                ),
            },

            "graph": self.graph_map.current_state(),
            "edges": self.graph_map.edges_state(),
            "gates": self.graph_map.gate_edges(),
            "timing": self.timer.as_dict(),
            "start_recommendations": self.start_recommendations,

            "plan": {
                "note": self.plan_note,
                "next_turn": None if self.next_turn is None else self.next_turn.name,
                "route_edge_keys": planner.route_edge_keys(self.planned_steps),
                "turns": planner.turn_sequence(self.planned_steps),
                "steps": planner.steps_as_dicts(self.planned_steps),
                "estimated_cost": planner.total_cost(self.planned_steps),
            },

            "move_result": move_result,
        }

        self.pub_state.publish(String(data=json.dumps(state)))


if __name__ == "__main__":
    try:
        MappingPathplanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
