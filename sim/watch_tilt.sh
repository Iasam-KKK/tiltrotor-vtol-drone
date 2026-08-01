#!/usr/bin/env bash
# Live readout of the two wing tilt joints, in degrees.
#
# This is how you SEE yaw on this aircraft. Yaw is not produced by the props --
# it is produced by the two wing nacelles tilting differentially, one leaning
# aft of vertical while the other leans forward, giving a couple about z.
# So a yaw command shows up here as the two numbers SPLITTING APART:
#
#   hover, no yaw   :  tilt_left   0.0    tilt_right   0.0     (both vertical)
#   yaw command     :  tilt_left  -8.0    tilt_right  +8.0     (split)
#   cruise          :  tilt_left  90.0    tilt_right  90.0     (both forward)
#
# Sign convention is PX4's: 0 deg = thrust UP, +90 = thrust FORWARD, and the
# negative travel (to -15) exists ONLY for vectored yaw.
#
# ⚠ Expect a standing split of roughly 4 deg even with no yaw commanded. That
# is the fixed lift rotor's reaction torque being trimmed out by the wing pair
# -- see the momentConstant/CA_ROTOR2_KM note in the review.
#
# Usage:  bash sim/watch_tilt.sh
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"

command -v gz >/dev/null || { echo "gz not found" >&2; exit 1; }

# Find the joint_state topic without hard-coding the model instance suffix
# (PX4 spawns as tri_tiltrotor_0, _1, ... depending on instance).
TOPIC=$(timeout 8 gz topic -l 2>/dev/null | grep -m1 'joint_state')
if [ -z "$TOPIC" ]; then
    echo "ERROR: no joint_state topic. Is the sim running?" >&2
    echo "Topics visible:" >&2
    timeout 8 gz topic -l 2>/dev/null | head -20 >&2
    exit 1
fi
echo "topic: $TOPIC"
echo "watching tilt_left_joint / tilt_right_joint  (Ctrl-C to stop)"
echo

gz topic -e -t "$TOPIC" 2>/dev/null | python3 -u -c '
import sys, math

name = None
left = right = None
def deg(r):
    return math.degrees(r)

for line in sys.stdin:
    s = line.strip()
    if s.startswith("name:"):
        name = s.split('"'"'"'"'"')[1] if '"'"'"'"'"' in s else s.split(":",1)[1].strip()
    elif s.startswith("position:") and name:
        try:
            v = float(s.split(":",1)[1])
        except ValueError:
            continue
        if name == "tilt_left_joint":
            left = v
        elif name == "tilt_right_joint":
            right = v
        if left is not None and right is not None and name == "tilt_right_joint":
            split = deg(left) - deg(right)
            bar = "yaw>>" if split > 1.0 else ("<<yaw" if split < -1.0 else "  -  "
            )
            print(f"left {deg(left):7.2f}   right {deg(right):7.2f}   "
                  f"split {split:+7.2f}   {bar}")
'
