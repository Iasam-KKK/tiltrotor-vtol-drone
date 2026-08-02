#!/usr/bin/env bash
# Measure the unpowered glide and compare it against the derived drag polar.
#
# THIS IS A TEST, NOT A DEMONSTRATION. params.py derives, from the section and
# the planform alone:
#     (L/D)max = 0.5 sqrt(pi e AR / CD0)   at   CL = sqrt(CD0 pi e AR)
# and the teleop commands exactly that descent. If the polar is right, the
# aircraft holds it with the throttle on its stop and the measured glide ratio
# matches. If the polar is optimistic, PX4 has to add power to hold the
# commanded path and the measured ratio comes out different -- which this
# script will show rather than hide.
#
# Glide ratio is measured the honest way: horizontal distance travelled divided
# by altitude lost, straight from Gazebo's pose topic. No PX4 log lines, no
# self-reported numbers.
#
# Usage:
#   1. bash sim/run_gui.sh            (from the RDP session)
#   2. bash sim/ros2/run_agent.sh
#   3. bash sim/ros2/teleop.sh        -> space, o, w to climb, t, then g
#   4. bash sim/verify_glide.sh       while it is gliding
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FP="$HERE/ros2/flight_params.json"
DUR="${DUR:-40}"

command -v gz >/dev/null || { echo "gz not found" >&2; exit 1; }
pgrep -f "gz sim.* -s" >/dev/null || {
    echo "ERROR: no Gazebo server running. Start with sim/run_gui.sh" >&2; exit 1; }
[ -f "$FP" ] || {
    echo "ERROR: $FP missing. Run cad/gen_flight_params.py" >&2; exit 1; }

echo "logging ${DUR}s of flight from Gazebo's pose topic"
echo "put the aircraft in a glide first: teleop -> space, o, w, t, g"
echo

timeout "$DUR" gz topic -e -t /world/default/dynamic_pose/info 2>/dev/null \
  | python3 -u -c '
import sys, math, time, json, os

fp = json.load(open(sys.argv[1]))
g = fp["glide"]

# Pull these out BEFORE any f-string. This whole program is embedded in a
# single-quoted shell string, so a single quote anywhere in it -- including
# inside an f-string subscript like g[<quote>v_best_glide_ms<quote>] -- ends
# the shell string and Python then sees a bare name. That bug sat here
# undetected because it is past the "not actually gliding" early exit, so the
# line had never once been reached.
PRED_LD    = g["l_over_d_max"]
PRED_V     = g["v_best_glide_ms"]
PRED_SINK  = g["sink_rate_ms"]

t0 = None
name = None
vals = []
samples = []          # (t, x, y, z)

for line in sys.stdin:
    s = line.strip()
    if s.startswith("name:"):
        name = s.split(":", 1)[1].strip().strip(chr(34))
        vals = []
    elif name and name.startswith("tri_tiltrotor"):
        for k in ("x:", "y:", "z:"):
            if s.startswith(k) and len(vals) < 3:
                try:
                    vals.append(float(s.split(":", 1)[1]))
                except ValueError:
                    pass
        if len(vals) == 3:
            now = time.time()
            t0 = t0 if t0 is not None else now
            samples.append((now - t0, vals[0], vals[1], vals[2]))
            vals = []
            name = None

if len(samples) < 20:
    print(f"  only {len(samples)} samples -- is the sim running and airborne?")
    raise SystemExit(1)

# Use the steadiest stretch: drop the first and last 20%, which contain the
# entry transient and whatever the pilot did next.
n = len(samples)
mid = samples[int(0.2 * n):int(0.8 * n)]
t_a, x_a, y_a, z_a = mid[0]
t_b, x_b, y_b, z_b = mid[-1]

dt = t_b - t_a
dh = math.hypot(x_b - x_a, y_b - y_a)
dz = z_a - z_b                       # positive = lost altitude

print(f"  samples            {n}  ({dt:.1f} s of steady flight used)")
print(f"  horizontal travel  {dh:8.2f} m")
print(f"  altitude lost      {dz:8.2f} m")
if dt > 0:
    print(f"  ground speed       {dh / dt:8.2f} m/s")
    print(f"  sink rate          {dz / dt:8.3f} m/s   "
          f"(predicted {PRED_SINK:.3f})")

print()
if dz <= 0.5:
    print("  VERDICT: barely descending -- this was not an unpowered glide.")
    print("           Press g in the teleop and re-run while it is gliding.")
    raise SystemExit(1)

# Speed gate. Altitude lost alone cannot tell a glide from powered flight that
# happens to be sinking, and dividing by a near-zero sink rate turns a powered
# cruise into a spectacular glide ratio. If the aircraft is flying far faster
# than the speed the polar asked for, the throttle is not closed and no ratio
# computed from this run means anything.
v_gnd = dh / dt if dt > 0 else 0.0
if v_gnd > 1.35 * PRED_V:
    print(f"  VERDICT: NOT a glide. Ground speed {v_gnd:.1f} m/s against a")
    print(f"           commanded {PRED_V:.2f} m/s -- the aircraft is under power.")
    print( "           PX4 does not track offboard NED velocity setpoints in")
    print( "           fixed-wing mode; the setpoint is accepted and ignored,")
    print( "           and the FW controller holds altitude at its own airspeed.")
    print(f"           (A ratio computed here would read {dh / dz:.0f}, which is")
    print( "            not a number this airframe can produce.)")
    raise SystemExit(2)

ld = dh / dz
pred = PRED_LD
err = 100.0 * (ld - pred) / pred
print(f"  MEASURED glide ratio  {ld:6.2f}")
print(f"  PREDICTED (L/D)max    {pred:6.2f}   at {PRED_V:.2f} m/s")
print(f"  difference            {err:+6.1f} %")
print()
if abs(err) <= 15.0:
    print("  The measured glide agrees with the polar derived from the NACA")
    print("  section and the planform. The aerodynamic model is self-consistent.")
else:
    print("  The measured glide does NOT match the polar. Either the drag")
    print("  model is off, or PX4 is adding thrust to hold the commanded path.")
    print("  Check the throttle: a real glide should sit on its lower stop.")
' "$FP"
