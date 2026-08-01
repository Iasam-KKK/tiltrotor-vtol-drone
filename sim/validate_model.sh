#!/usr/bin/env bash
# Validate the generated tri-tiltrotor model against SDFormat and load it in
# Gazebo Harmonic headless.
#
# XML well-formedness is NOT validity: a file can parse as XML and still be
# rejected by SDFormat, or load with every plugin silently failing. This script
# distinguishes those cases.
#
# Uses the ROS-vendored Gazebo Harmonic (gz-sim 8.11.0). A separate standalone
# gz-harmonic install is NOT required and is deliberately avoided so the
# verified ROS 2 Jazzy stack is left alone.
#
# NOTE: run via `wsl -- bash validate_model.sh`, which does NOT source
# ~/.bashrc, so GALLIUM_DRIVER must be exported here or Mesa silently falls
# back to llvmpipe. See CLAUDE.md.
set -o pipefail

export GALLIUM_DRIVER=d3d12
source /opt/ros/jazzy/setup.bash 2>/dev/null
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$HERE/models:$GZ_SIM_RESOURCE_PATH"
MODEL="$HERE/models/tri_tiltrotor/model.sdf"

PASS=0
FAIL=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
bad()  { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }

echo "=============================================================="
echo "TRI-TILTROTOR MODEL VALIDATION"
echo "=============================================================="

# --- 1. SDFormat spec conformance ------------------------------------------
echo
echo "[1] SDFormat parse"
SDF_OUT="$(gz sdf -k "$MODEL" 2>&1)"
# Judge on SDFormat's own verdict, not on the presence of warnings.
# `gz_frame_id` is a Gazebo extension that SDFormat does not know about and
# warns on; every stock PX4 model uses it, and dropping it to silence the
# warning would break the frame reported in sensor messages. Errors still fail.
if echo "$SDF_OUT" | grep -q "^Valid\." && ! echo "$SDF_OUT" | grep -qi "error"; then
    ok "gz sdf -k reports the file valid"
    NWARN=$(echo "$SDF_OUT" | grep -ci "warning")
    [ "$NWARN" -gt 0 ] && echo "        ($NWARN benign warning(s), gz_frame_id extension)"
else
    bad "gz sdf -k reported an error:"
    echo "$SDF_OUT" | grep -i "error" | head -10
fi

# --- 2. Resolved model ------------------------------------------------------
echo
echo "[2] Structure after SDFormat resolution"
RES="$(gz sdf -p "$MODEL" 2>/dev/null)"
for item in \
    "link name='base_link'" \
    "joint name='tilt_left_joint'" \
    "joint name='tilt_right_joint'" \
    "joint name='tilt_tail_joint'" \
    "link name='airspeed_link'" \
    "sensor name='air_speed'"
do
    if echo "$RES" | grep -q "$item"; then ok "$item present"; else bad "$item MISSING"; fi
done

N_LD=$(echo "$RES" | grep -c "LiftDrag")
N_MC=$(echo "$RES" | grep -c "MulticopterMotorModel")
N_JP=$(echo "$RES" | grep -c "JointPositionController")
[ "$N_MC" -eq 3 ] && ok "3 MulticopterMotorModel plugins" || bad "expected 3 motors, got $N_MC"

# Do NOT hardcode the servo count: it depends on TAIL_TILTS. Derive it -- every
# revolute joint that is not a rotor needs exactly one position controller.
# Hardcoding 7 made this check fail the moment the tail rotor became fixed,
# reporting a configuration change as a defect.
N_REV=$(echo "$RES" | grep -c "joint name='\(tilt\|aileron\|vtail\|elevator\|rudder\)[^']*' type='revolute'")
if [ "$N_JP" -eq "$N_REV" ]; then
    ok "$N_JP JointPositionController plugins, one per revolute control joint"
else
    bad "$N_JP servos for $N_REV revolute control joints"
fi
[ "$N_LD" -eq 4 ] && ok "4 LiftDrag surfaces" || bad "expected 4 lift surfaces, got $N_LD"

# --- 3. Headless load -------------------------------------------------------
echo
echo "[3] Headless load in Gazebo Harmonic"
LOG="$(timeout 120 gz sim -s -r --iterations 300 -v 2 "$HERE/worlds/test.sdf" 2>&1)"
RC=$?
if [ $RC -eq 0 ]; then ok "simulator ran 300 iterations and exited cleanly"
else bad "simulator exited rc=$RC"; fi

if echo "$LOG" | grep -qi "Unable to load\|Failed to load\|could not be found"; then
    bad "plugin or resource load failures:"
    echo "$LOG" | grep -i "Unable to load\|Failed to load\|could not be found" | head -10
else
    ok "no plugin or resource load failures"
fi

if echo "$LOG" | grep -qiE "\[Err\]"; then
    bad "Gazebo reported errors:"
    echo "$LOG" | grep -iE "\[Err\]" | head -10
else
    ok "no [Err] lines"
fi

echo
echo "=============================================================="
echo "  $PASS passed, $FAIL failed"
echo "=============================================================="
[ "$FAIL" -eq 0 ]
