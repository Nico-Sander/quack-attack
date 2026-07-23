#!/usr/bin/env python3

"""
Menu-driven front end for the whole mission.

    rosrun ch4_mapping_pathfinding mission_tui.py

Walks through the two phases in the order they actually happen: configure the
mapping run, watch it, and when it stops at the red line, configure and start
the timed gate run from the same menu.

The important structural point is that **one roslaunch stays alive across both
phases**. The map, the gates and the measured street times all live in the
mapping node; restarting it between phases throws them away. So this owns the
launch process for the whole session and switches phases through the parameters
and the service, exactly as docs/workflow.md describes by hand. Every command it
runs is printed, so nothing here is a shortcut you cannot take yourself.

roslaunch's own output goes to a log file rather than the terminal, because the
two cannot share a screen: a menu prompt buried in scrolling node output is
unreadable. The log path is shown, and `tail -f` on it in another tmux pane
gives the usual view.
"""

import os
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import city_map
import mission_setup
from mission_setup import SetupError


COLOUR = sys.stdout.isatty() and os.environ.get("NO_COLOR") is None


def _c(code, text):
    return f"\033[{code}m{text}\033[0m" if COLOUR else text


def bold(text):
    return _c("1", text)


def dim(text):
    return _c("2", text)


def green(text):
    return _c("32", text)


def yellow(text):
    return _c("33", text)


def red(text):
    return _c("31", text)


def blue(text):
    return _c("36", text)


WIDTH = 66


def rule(char="-"):
    print(dim(char * WIDTH))


def heading(text):
    print()
    rule("=")
    print(bold(f"  {text}"))
    rule("=")


def note(text):
    print(dim(f"  {text}"))


# ---------------------------------------------------------------------------
# Prompts
# ---------------------------------------------------------------------------

class Aborted(Exception):
    """The operator asked to stop."""


def ask(question, default=None, parse=None, help_text=None):
    """
    Asks until the answer parses. Enter accepts the default.

    `parse` returns the value to use or raises SetupError with a message that
    says what to do differently -- the point of validating here rather than at
    the robot.
    """
    suffix = f" [{default}]" if default not in (None, "") else ""

    while True:
        if help_text:
            note(help_text)

        try:
            raw = input(f"  {question}{suffix}: ").strip()
        except (EOFError, KeyboardInterrupt):
            raise Aborted()

        if not raw and default is not None:
            raw = str(default)

        if parse is None:
            return raw

        try:
            return parse(raw)
        except SetupError as error:
            print(f"  {red('!')} {error}")


def ask_bool(question, default=True):
    def parse(text):
        lowered = text.strip().lower()

        if lowered in ("y", "yes", "true", "1"):
            return True
        if lowered in ("n", "no", "false", "0"):
            return False

        raise SetupError("Answer y or n")

    return ask(question, "y" if default else "n", parse)


def menu(title, options):
    """options: list of (key, label). Returns the chosen key."""
    heading(title)

    for key, label in options:
        print(f"  {bold(key)}) {label}")

    print()
    keys = {key.lower() for key, _ in options}

    while True:
        try:
            choice = input("  choice: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise Aborted()

        if choice in keys:
            return choice

        print(f"  {red('!')} Pick one of: {', '.join(sorted(keys))}")


# ---------------------------------------------------------------------------
# Mission state, read off /mapping/state
# ---------------------------------------------------------------------------

class MissionMonitor:
    """Subscribes to /mapping/state and keeps the latest one."""

    def __init__(self, namespace):
        self.namespace = namespace
        self.state = None
        self.lock = threading.Lock()
        self._started = False

    def start(self):
        import rospy
        from std_msgs.msg import String

        rospy.init_node("mission_tui", anonymous=True, disable_signals=True)
        rospy.Subscriber(f"{self.namespace}/mapping/state", String,
                         self._cb, queue_size=1)
        self._started = True

    def _cb(self, msg):
        import json

        try:
            parsed = json.loads(msg.data)
        except ValueError:
            return

        with self.lock:
            self.state = parsed

    def get(self):
        with self.lock:
            return self.state

    def phase(self):
        state = self.get()
        return (state or {}).get("mission", {}).get("phase")


def wait_for_master(timeout=40.0):
    import rosgraph

    deadline = time.time() + timeout

    while time.time() < deadline:
        if rosgraph.is_master_online():
            return True
        time.sleep(0.5)

    return False


def running_mapping_node(namespace):
    """True if a mapping node is already up -- launching a second would fight."""
    try:
        import rosgraph
        import rosnode

        if not rosgraph.is_master_online():
            return False

        return f"{namespace}/mapping_pathplanning_node" in rosnode.get_node_names()
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Status rendering
# ---------------------------------------------------------------------------

def clear():
    if COLOUR:
        print("\033[2J\033[H", end="")


def render_status(state, log_path, elapsed):
    """One screen describing where the mission is."""
    clear()
    mission = state.get("mission", {})
    graph = state.get("graph", {})
    plan = state.get("plan", {})
    timing = state.get("timing", {})

    phase = mission.get("phase", "?")
    colour = {"MAPPING": blue, "GATE_RUN": green,
              "AWAITING_GATE_RUN": yellow, "DONE": green}.get(phase, bold)

    print(bold("  Duckiebot mission"), dim(f"   ({elapsed:.0f}s)"))
    rule()
    print(f"  Phase      {colour(phase)}")
    print(f"  Position   {graph.get('current_edge_key')} "
          f"-> {graph.get('approaching_node')} "
          f"(port {graph.get('entry_port')})")

    edges = state.get("edges", {})
    driven = sum(1 for data in edges.values() if data.get("visited"))
    times = timing.get("street_times", {})
    print(f"  Streets    {driven}/{len(edges)} driven, {len(times)} timed")

    gates = state.get("gates", [])
    if gates:
        listed = ", ".join(f"{g['gate_id']}@{g['edge_key']}" for g in gates)
        print(f"  Gates      {listed}")
    else:
        print(f"  Gates      {dim('none found yet')}")

    order = mission.get("gate_order") or []
    if order:
        remaining = mission.get("remaining_gates") or []
        done = [gate for gate in order if gate not in remaining]
        print(f"  Order      {order}  done {green(str(done))}  left {remaining}")

    if mission.get("halted"):
        print(f"  {yellow('HOLDING at the red line')}")
    else:
        print(f"  Next turn  {plan.get('next_turn')}")

    if mission.get("localization_ok") is False:
        print(f"  {red('LOCALIZATION LOST -- the planner has stood down')}")

    print(f"  Plan       {dim(str(plan.get('note')))}")
    rule()
    note(f"log: tail -f {log_path}")
    note("Ctrl-C to shut the mission down")


def print_summary(state):
    """The end-of-mapping detail, from the state rather than the log."""
    timing = state.get("timing", {})
    times = timing.get("street_times", {})

    if times:
        print(bold("  Measured street times"))
        for key, value in sorted(times.items()):
            print(f"    {key:<12} {value:5.1f} s")

    turns = timing.get("turn_times") or {}
    if turns:
        rendered = ", ".join(f"{name} {value:.1f}s"
                             for name, value in sorted(turns.items()))
        print(f"  {bold('Turn times')}    {rendered}")

    gates = state.get("gates", [])
    print(bold(f"  Gates found ({len(gates)})"))
    for gate in gates:
        print(f"    gate {gate['gate_id']:<3} on {gate['edge_key']} "
              f"({gate['gate_colour'] or '?'})")

    edges = state.get("edges", {})
    gateless = sorted(key for key, data in edges.items()
                      if data.get("gate_id") is None)
    if gateless:
        print(f"  {yellow('No gate on')} {', '.join(gateless)}")
        note("Either there is none there, or one was missed -- the map cannot")
        note("tell you which. Check the rosbag before trusting the run.")


def print_run_result(result):
    """The timed result, with the two boundaries kept apart."""
    scored = "%.1f s" % result["time_to_last_gate"]

    print("  Gates driven   %s" % result["gates"])
    print("  %s           %s   %s" % (bold("TIME"), green(scored),
                                      dim("(first movement -> last gate passed)")))
    print("  At stop line   %.1f s   %s" % (result["time_to_stop"],
                                            dim("(+ driving out the last street)")))

    estimate = result.get("estimate")
    if estimate:
        error = result["time_to_stop"] - estimate
        detail = "(%+.1f s, %+.0f%%)" % (error, 100.0 * error / estimate)
        print("  Planner said   %.1f s   %s" % (estimate, dim(detail)))


def print_recommendations(state):
    options = state.get("start_recommendations") or []

    if not options:
        return None

    print()
    print(bold("  Best placements for this order"))
    for rank, option in enumerate(options, start=1):
        marker = " (first gate is on this street)" if option.get(
            "first_gate_on_start_street") else ""
        print(f"    {rank}. {green(option['start_edge_arg']):<20} "
              f"{option['cost']:5.1f} s, {option['steps']} crossing(s){marker}")

    return options[0]["start_edge_arg"]


# ---------------------------------------------------------------------------
# Configuration screens
# ---------------------------------------------------------------------------

def show_city(city, gate_config):
    heading("City reference")
    print(bold("  Legal placements  (start_edge)"))

    for _edge, arg, key in mission_setup.legal_start_edges(city):
        print(f"    {arg:<12} drives {key}")

    print()
    print(bold("  Gate colours  (cosmetic; the order is announced as IDs)"))
    for gate_id in sorted(gate_config):
        print(f"    {gate_id:<4} {gate_config[gate_id].get('colour', '?')}")


def configure_mapping(city, gate_config, simulate=False):
    heading("Configure the mapping run")
    note("Enter accepts the value in brackets.")
    print()

    config = mission_setup.default_config()

    print(bold("  Placement"))
    note("The bot starts at an intersection EXIT, facing along the street.")
    note("Format: from,port,to,port   e.g.  A,4,D,2   (menu option 2 lists them)")
    config["start_edge"] = ask(
        "start_edge",
        config["start_edge"],
        lambda text: mission_setup.parse_start_edge(text, city),
    )
    print(f"    {green('->')} {mission_setup.describe_start_edge(config['start_edge'], city)}")

    print()
    print(bold("  Gate order"))
    note("Optional now. Giving it lets the bot recommend where to place it")
    note("for the run once mapping finishes. Blank to skip.")
    note("Format: comma-separated tag IDs, in order   e.g.  7,5,6")
    order = ask("gate_order", "",
                lambda text: mission_setup.parse_gate_order(text, gate_config))
    unknown = mission_setup.unknown_gates(order, gate_config)
    if unknown:
        print(f"    {yellow('note')} gate(s) {unknown} are not in gates.json "
              f"(fine, colours only)")
    config["gate_order"] = ",".join(str(gate) for gate in order)

    print()
    print(bold("  Run options"))
    if not simulate:
        # No motors and no camera in simulation, so these have nothing to act on
        # and simulate.launch does not declare them.
        config["driving"] = ask_bool("drive the motors", True)
        config["dashboard"] = ask_bool("show the perception dashboard", False)
    config["visualization"] = ask_bool("show the map window", True)

    if ask_bool("configure advanced options", False):
        print()
        print(bold("  Advanced"))
        config["gate_min_area"] = ask(
            "gate_min_area px (-1 = use config.json)",
            config["gate_min_area"], int)
        config["gate_confirm_frames"] = ask(
            "gate_confirm_frames", config["gate_confirm_frames"], int)
        config["map_gates_while_approaching"] = ask_bool(
            "map gates while approaching a stop line", True)
        config["strict_gate_order"] = ask_bool(
            "strict gate order (route around gates not yet due)", False)
        config["force_turn"] = ask(
            "force_turn (NONE/LEFT/STRAIGHT/RIGHT)", "NONE",
            lambda text: _one_of(text.upper(),
                                 ("NONE", "LEFT", "STRAIGHT", "RIGHT")))
        config["gate_cooldown"] = ask("gate_cooldown s",
                                      config["gate_cooldown"], float)
        config["command_timeout"] = ask("command_timeout s",
                                        config["command_timeout"], float)
        config["sign_cooldown"] = ask("sign_cooldown s",
                                      config["sign_cooldown"], float)

    return config


def _one_of(value, allowed):
    if value not in allowed:
        raise SetupError(f"Must be one of {', '.join(allowed)}")
    return value


def configure_gate_run(city, gate_config, state, previous_order):
    heading("Configure the gate run")

    recommended = print_recommendations(state)
    print()

    note("Format: comma-separated tag IDs, in order   e.g.  7,5,6")
    order = ask("gate_order (announced on site)",
                ",".join(str(gate) for gate in previous_order) or None,
                lambda text: mission_setup.parse_gate_order(text, gate_config))

    if not order:
        raise SetupError("The run needs an order")

    mapped = {gate["gate_id"] for gate in state.get("gates", [])}
    missing = [gate for gate in order if gate not in mapped]

    if missing:
        print(f"  {red('!')} gate(s) {missing} were never mapped; "
              f"the run cannot be planned")
        raise SetupError(f"gate(s) {missing} not on the map")

    print()
    note("Place the bot at an intersection exit, then give that placement.")
    note("Blank keeps it where it is standing (it will cross, not drive off).")
    here = state.get("graph", {}).get("current_edge_key")
    gate_here = next((gate["gate_id"] for gate in state.get("gates", [])
                      if gate["edge_key"] == here), None)
    if gate_here is not None and order and order[0] == gate_here:
        # The bot is at the far end of that street, so the gate is behind it and
        # the route has to drive a whole lap back onto it.
        print(f"  {yellow('note')} gate {gate_here} is on {here}, which the bot "
              f"is at the END of.")
        note(f"Leaving the placement blank means driving a lap back to it.")
    start_edge = ask("gate_run_start_edge", recommended or "",
                     lambda text: (mission_setup.parse_start_edge(text, city)
                                   if text.strip() else ""))

    strict = ask_bool("strict gate order (avoid gates not yet due)", False)

    return order, start_edge, strict


# ---------------------------------------------------------------------------
# Launching and driving the mission
# ---------------------------------------------------------------------------

class Mission:
    """Owns the roslaunch process for the whole session."""

    def __init__(self, namespace, log_path, simulate=False):
        self.namespace = namespace
        self.log_path = log_path
        self.simulate = simulate
        self.process = None
        self.log_file = None

    def launch(self, config):
        command = mission_setup.build_launch_command(config, self.simulate)

        heading("Launching")
        print(f"  {dim(mission_setup.as_shell(command))}")
        print()
        note(f"output -> {self.log_path}")

        self.log_file = open(self.log_path, "w")
        # Own process group, so Ctrl-C here can be delivered to roslaunch and
        # every node under it rather than only to this script.
        self.process = subprocess.Popen(
            command, stdout=self.log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        return self.process

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def shutdown(self):
        """SIGINT, not SIGKILL: control_wheels stops the robot on shutdown."""
        if not self.alive():
            return

        print()
        print(yellow("  Shutting the mission down (stopping the robot)..."))

        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            self.process.send_signal(signal.SIGINT)

        try:
            self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            print(red("  roslaunch did not exit; killing it"))
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

        if self.log_file:
            self.log_file.close()

        print(green("  Stopped."))


def run_commands(commands):
    """Runs the documented rosparam/rosservice calls, showing each one."""
    for command in commands:
        print(f"  {dim('$ ' + mission_setup.as_shell(command))}")
        result = subprocess.run(command, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"  {red('!')} {result.stderr.strip() or result.stdout.strip()}")
            return False, result

        if result.stdout.strip():
            for line in result.stdout.strip().splitlines():
                print(f"    {line}")

    return True, result


def watch(monitor, mission, until, started_at, refresh=1.0):
    """
    Renders status until `until(state)` is true or the launch dies.

    Returns the state that satisfied it, or None if the mission ended first.
    """
    while True:
        if not mission.alive():
            print()
            print(red("  roslaunch exited. Last lines of the log:"))
            _tail(mission.log_path)
            return None

        state = monitor.get()

        if state is not None:
            render_status(state, mission.log_path, time.time() - started_at)

            if until(state):
                return state

        time.sleep(refresh)


def _tail(path, lines=25):
    try:
        with open(path) as handle:
            content = handle.read().splitlines()
    except IOError:
        return

    for line in content[-lines:]:
        print(f"    {line}")


# ---------------------------------------------------------------------------
# Main flow
# ---------------------------------------------------------------------------

def main():
    # rosrun appends its own arguments (__name:=, __log:=), so match on
    # presence rather than parsing argv strictly.
    simulate = "--simulate" in sys.argv

    vehicle = os.environ.get("VEHICLE_NAME", "default_robot")
    namespace = f"/{vehicle}"

    city, _layout = city_map.load_city()
    gate_config = city_map.load_gate_config()

    clear()
    heading("Duckiebot Challenge 4 -- mapping and gate run")
    print(f"  Robot   {bold(vehicle)}")
    print(f"  City    {len(city)} intersections, "
          f"{len(city_map.all_edge_keys(city))} streets")

    if simulate:
        print(f"  Mode    {yellow('SIMULATION')} -- fake driver, no robot")
        note("Rehearses the menu and the phase switch. Nothing moves.")

    if vehicle == "default_robot":
        print(f"  {yellow('VEHICLE_NAME is not set')} -- topics would live under "
              f"/default_robot")
        note("Start the container with ./start.sh <vehicle> first.")

    if running_mapping_node(namespace):
        print()
        print(f"  {red('A mapping node is already running.')}")
        note("Launching a second one would have them fight over the robot.")
        note("Stop the existing launch first, then start this again.")
        return 1

    log_path = os.path.join(
        os.environ.get("HOME", "/tmp"),
        f"mission_{time.strftime('%H%M%S')}.log"
    )
    mission = Mission(namespace, log_path, simulate=simulate)
    monitor = MissionMonitor(namespace)
    gate_order = []

    try:
        while True:
            choice = menu("Main menu", [
                ("1", "Configure and run the MAPPING phase"),
                ("2", "Show the city reference (placements, gate colours)"),
                ("q", "Quit"),
            ])

            if choice == "q":
                return 0
            if choice == "2":
                show_city(city, gate_config)
                continue

            config = configure_mapping(city, gate_config, simulate)
            gate_order = mission_setup.parse_gate_order(config["gate_order"])

            heading("Review")
            print(f"  {mission_setup.describe_start_edge(config['start_edge'], city)}")
            if gate_order:
                print(f"  gate order {gate_order}")
            if simulate:
                print(f"  {yellow('simulation')} -- nothing moves")
            else:
                print(f"  motors {'ON' if config['driving'] else 'OFF'}")
            print()

            if not ask_bool("start the mapping run", True):
                continue

            mission.launch(config)

            if not wait_for_master():
                print(red("  No ROS master appeared. Check the log."))
                _tail(log_path)
                return 1

            monitor.start()
            started_at = time.time()

            state = watch(
                monitor, mission,
                lambda s: s["mission"]["phase"] in ("AWAITING_GATE_RUN", "DONE"),
                started_at,
            )

            if state is None:
                return 1

            break

        # -- mapping is done; the launch stays up ------------------------
        while True:
            state = monitor.get() or state
            heading("Mapping complete -- the bot is holding at the red line")
            print_summary(state)

            choice = menu("Now what?", [
                ("1", "Configure and start the GATE RUN"),
                ("2", "Show the measured times and gates again"),
                ("3", "Shut down"),
            ])

            if choice == "3":
                mission.shutdown()
                return 0
            if choice == "2":
                continue

            try:
                order, start_edge, strict = configure_gate_run(
                    city, gate_config, state, gate_order
                )
            except SetupError as error:
                print(f"  {red('!')} {error}")
                continue

            gate_order = order

            heading("Starting the gate run")
            note("These are the commands from docs/workflow.md:")
            ok, _result = run_commands(mission_setup.build_gate_run_commands(
                namespace, order, start_edge or None, strict
            ))

            if not ok:
                note("The run was not started; nothing has moved.")
                continue

            started_at = time.time()
            # Not just phase == DONE: the last gate counts as passed when its
            # street is entered, and the bot then drives that street and stops.
            # The result only exists once it has, so waiting for DONE would
            # report an empty time.
            state = watch(monitor, mission,
                          lambda s: s["mission"].get("gate_run_result"),
                          started_at)

            if state is None:
                return 1

            heading("Gate run finished")
            print_run_result(state["mission"]["gate_run_result"])

            choice = menu("Another run?", [
                ("1", "Yes -- configure another gate run (map is kept)"),
                ("2", "Shut down"),
            ])

            if choice == "2":
                mission.shutdown()
                return 0

    except Aborted:
        mission.shutdown()
        print()
        print("  Aborted.")
        return 130
    except KeyboardInterrupt:
        mission.shutdown()
        return 130


if __name__ == "__main__":
    sys.exit(main())
