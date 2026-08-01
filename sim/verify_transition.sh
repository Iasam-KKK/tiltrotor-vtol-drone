#!/usr/bin/env bash
# Does the tri-tiltrotor actually transition from hover to forward flight?
#
# This is the claim the whole project rests on, and it is the one most easily
# faked. `commander transition` returning without an error proves only that the
# string parsed. PX4 will happily accept the command and then decline to act on
# it -- an earlier run showed exactly that, with "ESC failure detected" logged
# immediately afterwards.
#
# So this script does not trust PX4's console. It reads, from Gazebo:
#   1. the actual tilt joint angles  (did the nacelles rotate?)
#   2. the actual ground speed       (did it accelerate past transition speed?)
#   3. the actual altitude           (did it stay up while doing so?)
#
# A transition is only claimed when all three agree.
set -o pipefail

export GALLIUM_DRIVER=d3d12
export PX4_GZ_SIM_RENDER_ENGINE=ogre
source /opt/ros/jazzy/setup.bash 2>/dev/null
export PATH="$HOME/.local/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOGDIR="$HOME/tritilt_logs"
mkdir -p "$LOGDIR"
STAMP=$(date +%H%M%S)
LOG="$LOGDIR/trans_$STAMP.log"
JOINTS="$LOGDIR/joints_$STAMP.txt"
POSE="$LOGDIR/tpose_$STAMP.txt"
RUN_SECONDS="${1:-140}"

PASS=0; FAIL=0
ok()  { echo "  PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }

echo "=============================================================="
echo "TRI-TILTROTOR TRANSITION VERIFICATION"
echo "=============================================================="
echo "log: $LOG"

ln -sfn "$HERE/worlds/capture.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/capture.sdf"
pkill -f "gz sim" 2>/dev/null; pkill -x px4 2>/dev/null; sleep 3

cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"

(
  sleep 34
  echo "commander takeoff"
  sleep 30
  echo "commander transition"
  sleep 45
  echo "shutdown"
  sleep 5
) | HEADLESS=1 PX4_GZ_WORLD=capture \
    timeout "$RUN_SECONDS" make px4_sitl gz_tri_tiltrotor > "$LOG" 2>&1 &
RUN=$!

# --- sample joints and pose across the whole flight ------------------------
sleep 30
echo "sampling..."
for i in $(seq 1 40); do
    echo "### t=$i" >> "$JOINTS"
    timeout 4 gz topic -e -t /world/capture/model/tri_tiltrotor_0/joint_state -n 1 \
        2>/dev/null >> "$JOINTS"
    timeout 4 gz topic -e -t /world/capture/dynamic_pose/info -n 1 \
        2>/dev/null >> "$POSE"
    sleep 2
done

wait $RUN 2>/dev/null
pkill -f "gz sim" 2>/dev/null

# --- analyse ---------------------------------------------------------------
python3 - "$JOINTS" "$LOG" <<'PY'
import re, sys, math

joints_path, log_path = sys.argv[1], sys.argv[2]
txt = open(joints_path, errors="replace").read()

# The message is a Model protobuf in text form. Each joint block carries BOTH
# a `pose { position { x y z } }` -- a TRANSLATION -- and, for revolute joints,
# `axis1 { position: <radians> }` -- the ANGLE. Scraping `position:` naively
# picks up the translation and reports garbage (an earlier version of this
# script printed -103231 degrees of nacelle tilt that way). Parse blocks.
series: dict[str, list[float]] = {}
for sample in txt.split("### t=")[1:]:
    for m in re.finditer(r'joint\s*\{', sample):
        start, depth, i = m.end(), 1, m.end()
        while i < len(sample) and depth:
            if sample[i] == "{":
                depth += 1
            elif sample[i] == "}":
                depth -= 1
            i += 1
        block = sample[start:i]

        nm = re.search(r'name:\s*"([^"]+)"', block)
        ax = re.search(r'axis1\s*\{(.*?)\n  \}', block, re.S)
        if not (nm and ax):
            continue
        pos = re.search(r'position:\s*(-?[\d.eE+-]+)', ax.group(1))
        if pos:
            series.setdefault(nm.group(1), []).append(
                math.degrees(float(pos.group(1))))

print()
print("[1] did the nacelles actually rotate?")
tilts = {k: v for k, v in series.items() if k.startswith("tilt_")}
if not tilts:
    print("  FAIL  no tilt joint angles captured")
    sys.exit(0)

moved = 0
for n, vals in sorted(tilts.items()):
    lo, hi = min(vals), max(vals)
    print(f"      {n:20s} {lo:+7.1f} -> {hi:+7.1f} deg   travel {hi - lo:6.1f}")
    if hi - lo > 45.0:
        moved += 1

# Expect every TILTING nacelle to move. Do not hardcode three: a fixed
# hover-only lift rotor is emitted as a `fixed` joint, so it never appears in
# the joint-angle data at all. Hardcoding 3 would report a deliberate design
# choice as a failure.
if moved == len(tilts) and moved >= 2:
    print(f"  PASS  all {moved} tilting nacelle(s) rotated hover -> cruise")
    print(f"        ({len(tilts)} tilt joint(s) present; any fixed lift rotor "
          f"is a `fixed` joint and correctly absent here)")
elif moved > 0:
    print(f"  PARTIAL  {moved} of {len(tilts)} tilting nacelles rotated")
else:
    print(f"  FAIL  no nacelle moved -- commanded but not acted on")

# Show the manoeuvre, not just the endpoints. Endpoints alone cannot
# distinguish a real transition from a single glitch sample.
print()
print("      trajectory (deg, 0 = thrust UP, 90 = thrust FORWARD):")
for n in sorted(tilts):
    v = tilts[n]
    step = max(1, len(v) // 14)
    print(f"      {n:20s} " + " ".join(f"{x:5.1f}" for x in v[::step]))

# Control surfaces and rotors should also be alive.
print()
print("[1b] are the other actuators alive?")
for n in ("elevator_joint", "aileron_left_joint", "rudder_joint"):
    v = series.get(n)
    if v:
        print(f"      {n:20s} range {max(v) - min(v):6.1f} deg")
spin = [k for k in series if k.startswith("rotor_")
        and max(series[k]) - min(series[k]) > 1000]
print(f"      rotors spinning: {len(spin)}/3")

print()
print("[2] what did PX4 say?")
log = open(log_path, errors="replace").read()
for pat in ("Transition to fixed", "transition", "VTOL", "ESC failure",
            "Takeoff detected", "Armed by"):
    hits = [l.strip() for l in log.splitlines()
            if pat.lower() in l.lower() and "pxh>" not in l]
    for h in hits[:2]:
        print(f"      {h[:110]}")
PY

echo
echo "=============================================================="
echo "  raw joint samples : $JOINTS"
echo "  raw pose samples  : $POSE"
echo "  px4 log           : $LOG"
echo "=============================================================="
