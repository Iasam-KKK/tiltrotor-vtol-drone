#!/usr/bin/env bash
# Measure the pitch excursion during takeoff, from Gazebo's own pose data.
#
# WHY: the aircraft visibly pitches nose-up on takeoff. There are at least three
# candidate causes and they are NOT distinguishable by eye:
#
#   A) CG mismatch. base_link's inertial sits at the model origin, but the three
#      nacelle links hang mass at +0.1728 (x2) and -0.700, putting the true CG
#      11.2 mm aft of the origin. Every CA_ROTOR*_PX is quoted from the CG, so
#      PX4 allocates with the wrong arms. Worth ~0.53 N.m nose-up.
#
#   B) Rotor authority mismatch. The airframe never sets CA_ROTOR*_CT, so PX4
#      models all three rotors as equally powerful. The SDF gives the wing
#      rotors 38 N and the tail 25 N, with different motorConstants. PX4's
#      allocator is linear in normalised command; the Gazebo motor model is
#      quadratic in rotor velocity. Worth ~2.5 N.m nose-up.
#
#   C) Spawn tilt. The nacelles sit at 37.08 deg when disarmed (servo neutral of
#      the -15..+90 range). PX4 drives them to 0 at VT_TILT_MC, but the slew
#      takes ~0.8 s at TILT_RATE 45 deg/s, during which the wing rotors have a
#      large forward thrust component and reduced vertical component.
#
# A and B predict a pitch offset that PERSISTS in steady hover.
# C predicts a transient that decays once the nacelles reach 0 deg.
# So: log pitch AND tilt together, and the shape of the trace separates them.
#
# Usage:  bash sim/verify_takeoff_pitch.sh        # assumes sim already running
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12

command -v gz >/dev/null || { echo "gz not found" >&2; exit 1; }
pgrep -f "gz sim.* -s" >/dev/null || {
    echo "ERROR: no Gazebo server running. Start with sim/run_gui.sh" >&2; exit 1; }

DUR="${DUR:-30}"
OUT="${OUT:-/tmp/takeoff_pitch.csv}"

POSE_TOPIC="/world/default/dynamic_pose/info"
JOINT_TOPIC=$(timeout 8 gz topic -l 2>/dev/null | grep -m1 'joint_state')

echo "pose  topic: $POSE_TOPIC"
echo "joint topic: ${JOINT_TOPIC:-<none>}"
echo "logging ${DUR}s -> $OUT"
echo
echo "  >>> issue 'commander takeoff' in the pxh> window NOW <<<"
echo

# --- pitch, from the model pose quaternion --------------------------------
timeout "$DUR" gz topic -e -t "$POSE_TOPIC" 2>/dev/null | python3 -u -c '
import sys, math, time

t0 = time.time()
name = None; q = {}; p = {}
print("t_s,pitch_deg,z_m")
for line in sys.stdin:
    s = line.strip()
    if s.startswith("name:"):
        name = s.split(":",1)[1].strip().strip(chr(34))
        q = {}; p = {}
    elif name and name.startswith("tri_tiltrotor"):
        for k in ("x","y","z","w"):
            if s.startswith(k+":"):
                try: v = float(s.split(":",1)[1])
                except ValueError: continue
                if k == "w" or len(q) >= 3 and k in q: q[k] = v
                elif len(p) < 3 and k in "xyz" and k not in p: p[k] = v
                else: q[k] = v
        if len(q) == 4:
            # ZYX euler pitch from quaternion
            sinp = 2.0*(q["w"]*q["y"] - q["z"]*q["x"])
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.degrees(math.asin(sinp))
            print(f"{time.time()-t0:.2f},{pitch:+.2f},{p.get(chr(122),0.0):.2f}")
            q = {}; p = {}; name = None
' | tee "$OUT"

echo
echo "=== summary ==="
python3 - "$OUT" <<'PY'
import sys, csv
rows = []
with open(sys.argv[1]) as f:
    for r in csv.DictReader(f):
        try: rows.append((float(r["t_s"]), float(r["pitch_deg"])))
        except (ValueError, KeyError): pass
if not rows:
    print("  no samples -- was the sim running and did you command takeoff?")
    raise SystemExit
peak = max(rows, key=lambda r: abs(r[1]))
tail = [p for t, p in rows if t > rows[-1][0] - 5.0]
print(f"  samples          {len(rows)}")
print(f"  peak pitch       {peak[1]:+.2f} deg at t={peak[0]:.1f}s")
if tail:
    mean = sum(tail)/len(tail)
    print(f"  settled pitch    {mean:+.2f} deg (last 5 s)")
    print()
    if abs(mean) > 2.0:
        print("  VERDICT: pitch does NOT return to zero -> a STANDING moment.")
        print("           Consistent with cause A (CG) and/or B (rotor CT),")
        print("           NOT with C (spawn tilt), which would decay away.")
    else:
        print("  VERDICT: pitch settles near zero -> the excursion is a")
        print("           TRANSIENT, consistent with cause C (spawn tilt slew).")
PY
