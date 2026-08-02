#!/usr/bin/env python3
r"""
Keyboard teleoperation for the tri-tiltrotor VTOL over PX4's ROS 2 interface.

This is the node that makes the aircraft hand-flyable, and in particular makes
VECTORED YAW visible: press A or D and the two wing nacelles tilt differentially
-- one leaning aft of vertical, one forward -- producing a couple about z. Watch
it happen in a second terminal with:

    bash sim/watch_tilt.sh

Nothing about yawing this aircraft involves differential propeller speed. The
props do not yaw it. The nacelles do.

--------------------------------------------------------------------------
How PX4 offboard actually works, and the two traps in it
--------------------------------------------------------------------------
1. Setpoints must ALREADY be streaming at > 2 Hz before you ask for offboard
   mode, or PX4 rejects the mode change. This node therefore starts streaming
   immediately on launch and only offers the mode switch afterwards.

2. If the stream stops for ~0.5 s while armed in offboard, PX4 triggers a
   failsafe. So the publish loop must never block -- which is why keyboard
   reading runs on its own thread with a raw, non-blocking terminal, rather
   than inside the timer callback.

QoS is not negotiable either: PX4's uXRCE-DDS bridge publishes and subscribes
BEST_EFFORT with TRANSIENT_LOCAL durability. A default rclpy RELIABLE profile
silently fails to connect -- topics list fine and no data ever moves.

3. ⚠ TOPIC VERSIONING. PX4 v1.16+ appends _v1 to versioned messages, so the
   real topics on this firmware are:

       /fmu/out/vehicle_local_position_v1     pub=1     <- actual
       /fmu/out/vehicle_local_position        pub=0     <- does not exist

   Subscribing to the unversioned name is COMPLETELY SILENT: the topic shows up
   in `ros2 topic list` (your own subscription creates it), the node runs, and
   no callback ever fires. It presents as "waiting for PX4 position data..."
   forever, which reads like a bridge problem rather than a name problem.

   Note that src/modules/uxrce_dds_client/dds_topics.yaml lists the UNVERSIONED
   names and is therefore misleading. The authority is the live publisher
   count:  ros2 topic info /fmu/out/vehicle_status_v1

   This node subscribes to BOTH spellings so it works on either firmware.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

# --- tuning ---------------------------------------------------------------
STEP_XY = 1.0          # m per keypress, horizontal
STEP_Z = 0.5           # m per keypress, vertical
STEP_YAW = math.radians(15.0)
RATE_HZ = 20.0         # setpoint stream rate; PX4 needs > 2 Hz
TAKEOFF_ALT = 10.0     # m above the spawn point

# Glide numbers come from cad/params.py via gen_flight_params.py. They are
# DERIVED from the drag polar (best L/D is where induced drag equals parasite
# drag), not typed in -- which is what lets verify_glide.sh test the polar
# rather than merely demonstrate a descent.
_FP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "flight_params.json")
try:
    with open(_FP) as _f:
        _P = json.load(_f)
    V_BEST_GLIDE = _P["glide"]["v_best_glide_ms"]
    GLIDE_ANGLE = math.radians(_P["glide"]["glide_angle_deg"])
    LD_MAX = _P["glide"]["l_over_d_max"]
    SINK_RATE = _P["glide"]["sink_rate_ms"]
except (OSError, KeyError, ValueError):
    # Refuse to invent numbers: a hand-guessed glide speed would silently make
    # the verification meaningless.
    raise SystemExit(
        f"missing or malformed {_FP}\n"
        "Regenerate it:  .venv-cad/Scripts/python.exe cad/gen_flight_params.py")

HELP = r"""
+--------------------------------------------------------------------------+
|  TRI-TILTROTOR KEYBOARD TELEOP                                           |
+--------------------------------------------------------------------------+
|  space   arm / disarm                                                    |
|  o       engage OFFBOARD mode  (stream is already running)               |
|                                                                          |
|  w / s   climb / descend                                                 |
|  a / d   YAW left / right   <-- the vectored-yaw demo                    |
|  i / k   forward / back                                                  |
|  j / l   slide left / right                                              |
|                                                                          |
|  t       transition to FORWARD FLIGHT (nacelles 0 -> 90 deg)             |
|  b       back-transition to HOVER     (nacelles 90 -> 0 deg)             |
|                                                                          |
|  g       GLIDE -- unpowered descent at best L/D                          |
|  p       leave glide, back to powered level flight                       |
|                                                                          |
|  h       hold current position         r  reset setpoint to here         |
|  ?       reprint this help             q  quit (disarms first)           |
+--------------------------------------------------------------------------+
  Watch the nacelles split apart on a/d:  bash sim/watch_tilt.sh
  Measure the glide against the polar:    bash sim/verify_glide.sh
"""


def px4_qos() -> QoSProfile:
    """The only QoS profile PX4's uXRCE-DDS bridge will talk to."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class KeyReader(threading.Thread):
    """Raw non-blocking keyboard reader on its own thread.

    Kept off the ROS timer thread deliberately: a blocking read there would
    stall the setpoint stream and trip PX4's offboard failsafe.
    """

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.key: str | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)

    def run(self) -> None:
        try:
            tty.setcbreak(self._fd)
            while not self._stop.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    with self._lock:
                        self.key = ch
        finally:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def take(self) -> str | None:
        with self._lock:
            k, self.key = self.key, None
        return k

    def stop(self) -> None:
        self._stop.set()
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)


class Teleop(Node):
    def __init__(self, scripted: bool = False, glide_hold: float = 60.0,
                 climb_m: float = 40.0) -> None:
        super().__init__("tiltrotor_teleop")
        self.scripted = scripted
        self._glide_hold = glide_hold
        self._climb_m = climb_m
        qos = px4_qos()

        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos)

        # Subscribe to BOTH the versioned and unversioned spellings. Only one
        # will ever carry data; the other costs nothing and makes this node
        # work against PX4 before and after the v1.16 versioning change.
        for base, msg, cb in (
            ("vehicle_local_position", VehicleLocalPosition, self._on_pos),
            ("vehicle_status", VehicleStatus, self._on_status),
        ):
            for name in (f"/fmu/out/{base}_v1", f"/fmu/out/{base}"):
                self.create_subscription(msg, name, cb, qos)

        # Setpoint in NED. z is NEGATIVE up -- this is the single easiest thing
        # to get backwards, and it presents as the aircraft burying itself.
        self.sp = [0.0, 0.0, -TAKEOFF_ALT]
        self.yaw = 0.0
        self.have_pos = False
        self.armed = False
        self.nav_state = -1
        self.offboard_ticks = 0
        self.gliding = False

        # KeyReader puts the terminal in raw mode, which needs a tty. Under
        # `--script` there isn't one, and asking for it would abort the run.
        self.keys = None if scripted else KeyReader()
        if self.keys is not None:
            self.keys.start()

        self._z0 = None                 # ground-level NED z, set on first fix
        self._script = self._build_script() if scripted else None
        self._script_i = 0
        self._script_t0 = None

        self.create_timer(1.0 / RATE_HZ, self._tick)
        # Don't let "no data" look like "still starting up" indefinitely.
        self._ticks = 0
        self._warned = False
        if not scripted:
            print(HELP)
        self.get_logger().info(
            "streaming setpoints; waiting for PX4 position data...")

    # --- subscriptions ----------------------------------------------------
    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        if not self.have_pos and msg.xy_valid and msg.z_valid:
            # Seed the setpoint at the CURRENT position, not the origin, so
            # engaging offboard does not command a jump back to the spawn.
            self.sp = [msg.x, msg.y, msg.z - TAKEOFF_ALT]
            self.yaw = msg.heading
            self.have_pos = True
            self._z0 = msg.z            # ground datum, for scripted climbs
            self.get_logger().info(
                f"position acquired: x={msg.x:.1f} y={msg.y:.1f} z={msg.z:.1f}")
        self._x, self._y, self._z = msg.x, msg.y, msg.z
        self._hdg = msg.heading

    def _on_status(self, msg: VehicleStatus) -> None:
        was_armed, was_nav = self.armed, self.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        # Report what PX4 actually did, not what we asked for. An arm or mode
        # request that PX4 rejects is otherwise invisible from this side.
        if self.armed != was_armed:
            print(f"  [PX4] now {'ARMED' if self.armed else 'DISARMED'}")
        if self.nav_state != was_nav:
            print(f"  [PX4] nav_state -> {self.nav_state}"
                  f"{'  (OFFBOARD)' if self.nav_state == 14 else ''}")

    # --- helpers ----------------------------------------------------------
    def _stamp(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _command(self, command: int, **params) -> None:
        m = VehicleCommand()
        m.timestamp = self._stamp()
        m.command = command
        for i in range(1, 8):
            setattr(m, f"param{i}", float(params.get(f"p{i}", 0.0)))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def _arm(self, on: bool) -> None:
        self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                      p1=1.0 if on else 0.0)
        print(f"  -> {'ARM' if on else 'DISARM'} requested")

    def _offboard(self) -> None:
        # base_mode=1 (custom), main_mode=6 (offboard)
        self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        print("  -> OFFBOARD requested")

    def _transition(self, forward: bool) -> None:
        # MAV_VTOL_STATE: 3 = MC, 4 = FW
        self._command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
                      p1=4.0 if forward else 3.0)
        print(f"  -> transition to {'FORWARD FLIGHT' if forward else 'HOVER'}"
              f"  (watch the nacelles: bash sim/watch_tilt.sh)")

    def _glide(self, on: bool) -> None:
        """Enter or leave the unpowered glide."""
        if on:
            # Gliding in multicopter mode is meaningless -- the rotors are the
            # only thing holding it up. Ask for forward flight first if needed.
            if self.nav_state != 14 or not self.armed:
                print("  ⚠ arm and engage OFFBOARD first (space, then o)")
                return
            self._transition(True)
            self.gliding = True
            print(f"  -> GLIDE. Commanding the descent the polar predicts:")
            print(f"       {V_BEST_GLIDE:.2f} m/s at {math.degrees(GLIDE_ANGLE):.2f} deg "
                  f"= {SINK_RATE:.3f} m/s sink, L/D {LD_MAX:.2f}")
            print(f"     Measure it:  bash sim/verify_glide.sh")
        else:
            self.gliding = False
            if self.have_pos:
                self.sp = [self._x, self._y, self._z]
            print("  -> powered flight, holding here")

    # --- scripted flight --------------------------------------------------
    def _build_script(self):
        """The same key sequence a human would type, on a clock.

        The glide test is the only one that measures the *aerodynamic* model
        rather than the mechanism, and it was the only one that could not run
        without somebody sitting at the RDP desktop pressing keys. That made
        the drag polar the least-tested part of the aircraft, which is exactly
        backwards. Times are wall-clock offsets from the first position fix.
        """
        hold = self._glide_hold
        return [
            (0.0,  "arm",                    lambda: self._arm(True)),
            (3.0,  "engage offboard",        self._offboard),
            (6.0,  f"climb {self._climb_m:.0f} m",
             lambda: self._set_alt(self._climb_m)),
            (38.0, "transition to forward flight",
             lambda: self._transition(True)),
            (54.0, "enter glide",            lambda: self._glide(True)),
            (54.0 + hold, "sequence complete", self._script_done),
        ]

    def _set_alt(self, above_ground_m: float) -> None:
        if self._z0 is None:
            return
        self.sp[2] = self._z0 - above_ground_m      # NED: negative is up
        print(f"  -> climbing to {above_ground_m:.0f} m above spawn")

    def _script_done(self) -> None:
        print("  -> scripted sequence complete; disarming")
        self._arm(False)
        raise KeyboardInterrupt

    def _run_script(self) -> None:
        # The clock does not start until PX4 has a position, otherwise the
        # whole sequence burns down while the agent is still connecting.
        if not self.have_pos:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._script_t0 is None:
            self._script_t0 = now
            print("\n  [script] position acquired, starting sequence\n")
        t = now - self._script_t0

        while (self._script_i < len(self._script)
               and t >= self._script[self._script_i][0]):
            at, label, fn = self._script[self._script_i]
            self._script_i += 1
            alt = (self._z0 - self._z) if self._z0 is not None else 0.0
            print(f"  [script t={t:5.1f}s  alt={alt:5.1f}m] {label}")
            fn()

    # --- main loop --------------------------------------------------------
    def _tick(self) -> None:
        if self.scripted:
            self._run_script()
        else:
            self._handle_key()

        self._ticks += 1
        if not self.have_pos and not self._warned and self._ticks > 8 * RATE_HZ:
            self._warned = True
            print("\n  ⚠ no PX4 position data after 8 s. Arming will not work.")
            print("    Check which topic actually has a publisher:")
            print("      ros2 topic info /fmu/out/vehicle_local_position_v1")
            print("    pub=0 on BOTH spellings means the uXRCE-DDS agent is not")
            print("    connected:  bash sim/ros2/run_agent.sh\n")

        mode = OffboardControlMode()
        mode.timestamp = self._stamp()
        # In glide we command VELOCITY, not position. A position setpoint would
        # make PX4 hold an altitude, and holding altitude is precisely what an
        # unpowered aircraft cannot do -- it would add throttle to stay there
        # and the "glide" would be powered level flight.
        mode.position = not self.gliding
        mode.velocity = self.gliding
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self._stamp()
        if self.gliding:
            nan = float("nan")
            sp.position = [nan, nan, nan]
            # Fly the descent the aircraft's OWN polar predicts: forward at
            # v_best_glide, down at v_best_glide * sin(glide_angle). If the
            # derived L/D is right, PX4 holds this with the throttle on its
            # stop; if it is wrong, PX4 has to add power and verify_glide.sh
            # sees the discrepancy.
            v = V_BEST_GLIDE
            sp.velocity = [
                float(v * math.cos(GLIDE_ANGLE) * math.cos(self.yaw)),
                float(v * math.cos(GLIDE_ANGLE) * math.sin(self.yaw)),
                float(v * math.sin(GLIDE_ANGLE)),      # NED: +z is DOWN
            ]
        else:
            sp.position = [float(x) for x in self.sp]
        sp.yaw = float(self.yaw)
        self.pub_sp.publish(sp)

    def _handle_key(self) -> None:
        if self.keys is None:
            return
        k = self.keys.take()
        if k is None:
            return

        if k == " ":
            self._arm(not self.armed)
        elif k == "o":
            self._offboard()
        elif k == "w":
            self.sp[2] -= STEP_Z          # NED: negative is up
        elif k == "s":
            self.sp[2] += STEP_Z
        elif k == "a":
            self.yaw = self._wrap(self.yaw - STEP_YAW)
            print(f"  yaw LEFT  -> {math.degrees(self.yaw):+.0f} deg "
                  f"(nacelles split)")
        elif k == "d":
            self.yaw = self._wrap(self.yaw + STEP_YAW)
            print(f"  yaw RIGHT -> {math.degrees(self.yaw):+.0f} deg "
                  f"(nacelles split)")
        elif k in "ikjl":
            # Body-relative translation, rotated into NED by current yaw.
            fwd = {"i": STEP_XY, "k": -STEP_XY}.get(k, 0.0)
            rgt = {"l": STEP_XY, "j": -STEP_XY}.get(k, 0.0)
            self.sp[0] += fwd * math.cos(self.yaw) - rgt * math.sin(self.yaw)
            self.sp[1] += fwd * math.sin(self.yaw) + rgt * math.cos(self.yaw)
        elif k == "t":
            self._transition(True)
        elif k == "b":
            self._transition(False)
        elif k == "g":
            self._glide(True)
        elif k == "p":
            self._glide(False)
        elif k == "h":
            print(f"  hold at {self.sp[0]:.1f} {self.sp[1]:.1f} {self.sp[2]:.1f}")
        elif k == "r":
            if self.have_pos:
                self.sp = [self._x, self._y, self._z]
                self.yaw = self._hdg
                print("  setpoint reset to current position")
        elif k == "?":
            print(HELP)
        elif k == "q":
            if self.armed:
                self._arm(False)
            print("  quitting")
            raise KeyboardInterrupt

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--script", action="store_true",
                    help="fly arm -> offboard -> climb -> transition -> glide "
                         "on a timer, with no keyboard. Lets verify_glide.sh "
                         "run headlessly instead of needing the RDP desktop.")
    ap.add_argument("--glide-hold", type=float, default=60.0,
                    help="seconds to stay in the glide (default 60), which "
                         "must exceed verify_glide.sh's sampling window")
    ap.add_argument("--climb", type=float, default=40.0,
                    help="metres above spawn to climb before transitioning")
    args = ap.parse_args()

    rclpy.init()
    node = Teleop(scripted=args.script, glide_hold=args.glide_hold,
                  climb_m=args.climb)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.keys is not None:
            node.keys.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
