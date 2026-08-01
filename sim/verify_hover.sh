#!/usr/bin/env bash
# Does the tri-tiltrotor actually leave the ground?
#
# This is the check everything else defers to. params.py proves the arithmetic
# closes; validate_model.sh proves the model loads. Neither proves the physics.
# If the tilt sign convention is inverted the aircraft looks perfect in the
# GUI, reports healthy actuators, and drives itself into the ground.
#
# Altitude comes from Gazebo's own pose topic, not from scraping the pxh shell:
# the simulator's opinion of where the aircraft is cannot be faked by a
# misparsed log line.
#
# Launch goes through `make px4_sitl gz_tri_tiltrotor`, which is the supported
# path. Invoking ./bin/px4 by hand skips the orchestration that starts the
# Gazebo server, and PX4 then sits in "Waiting for Gazebo world..." until it
# times out with return value 15.
#
# Logs go to ~/tritilt_logs, NOT /tmp: WSL's /tmp is tmpfs and a WSL restart
# silently deletes the evidence you are trying to read.
set -o pipefail

export GALLIUM_DRIVER=d3d12
source /opt/ros/jazzy/setup.bash 2>/dev/null
export PATH="$HOME/.local/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOGDIR="$HOME/tritilt_logs"
mkdir -p "$LOGDIR"
STAMP=$(date +%H%M%S)
LOG="$LOGDIR/hover_$STAMP.log"
POSE="$LOGDIR/pose_$STAMP.txt"
RUN_SECONDS="${RUN_SECONDS:-90}"

PASS=0; FAIL=0
ok()  { echo "  PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }

echo "=============================================================="
echo "TRI-TILTROTOR HOVER VERIFICATION"
echo "=============================================================="
echo "log  : $LOG"
echo "pose : $POSE"

# --- clear strays ----------------------------------------------------------
# A leftover gz server keeps advertising /world/default and the next run
# attaches to a world that has no aircraft in it.
pkill -f "gz sim" 2>/dev/null
pkill -f "gz-sim-server" 2>/dev/null
pkill -x px4 2>/dev/null
sleep 3

cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"

# --- launch ----------------------------------------------------------------
(
  sleep 35
  # `commander mode takeoff` is rejected ("argument takeoff unsupported").
  # The takeoff verb is its own command, and it arms as part of the sequence;
  # arming separately first gets auto-disarmed by preflight before anything
  # can happen.
  echo "commander takeoff"
  sleep 40
  echo "shutdown"
  sleep 5
) | HEADLESS=1 timeout "$RUN_SECONDS" make px4_sitl gz_tri_tiltrotor \
    > "$LOG" 2>&1 &
RUN_PID=$!

# --- sample the model pose while it runs -----------------------------------
sleep 30
echo "sampling pose..."
for i in $(seq 1 12); do
    timeout 5 gz topic -e -t /world/default/dynamic_pose/info -n 1 2>/dev/null \
        >> "$POSE"
    sleep 4
done

wait $RUN_PID 2>/dev/null

echo
echo "--- boot ---"
grep -iE "startup script|SYS_AUTOSTART|Gazebo simulator|Ready for takeoff|Armed" "$LOG" | head -10

echo
echo "[1] airframe and simulator"
grep -q "4030" "$LOG" && ok "airframe 4030 selected" || bad "airframe 4030 not selected"
# "Waiting for Gazebo world..." during startup is NORMAL -- PX4 polls until
# the server is up. Only never getting past it is a failure, and the tell for
# that is the startup script returning non-zero (return value: 15).
if grep -qi "Startup script returned successfully" "$LOG"; then
    ok "connected to Gazebo world"
elif grep -qi "Startup script returned with return value" "$LOG"; then
    bad "PX4 never connected to a Gazebo world (startup timed out)"
else
    bad "PX4 startup outcome unclear"
fi

echo
echo "[2] no fatal errors"
if grep -qiE "\bFATAL\b|Segmentation fault|core dumped" "$LOG"; then
    bad "fatal error:"; grep -iE "\bFATAL\b|Segmentation" "$LOG" | head -5
else
    ok "no fatal errors"
fi

echo
echo "[3] arming"
grep -iE "Armed by|Arming denied|arm.*[Rr]eject|preflight" "$LOG" | head -4
grep -qi "Armed by" "$LOG" && ok "vehicle armed" || bad "vehicle never armed"

echo
echo "[4] did it climb?"
ZMAX=$(grep -oP '^\s*z:\s*\K-?[0-9.]+' "$POSE" 2>/dev/null | sort -g | tail -1)
NSAMP=$(grep -c 'z:' "$POSE" 2>/dev/null)
echo "      pose samples containing z: $NSAMP"
if [ -n "$ZMAX" ]; then
    echo "      max Gazebo z (UP-positive) = $ZMAX m"
    CLIMBED=$(python3 -c "print(1 if float('$ZMAX') > 1.0 else 0)" 2>/dev/null)
    if [ "$CLIMBED" = "1" ]; then
        ok "climbed to $ZMAX m"
    else
        bad "did not climb (max z = $ZMAX m)"
        echo "      >>> PRIME SUSPECT: tilt sign convention."
        echo "      >>> PX4 defines tilt 0 deg = UP (CA_SV_TL*_MINA/MAXA)."
        echo "      >>> Cross-check against SDF joint limits in gen_sdf.py."
    fi
else
    bad "no pose samples captured"
fi

echo
echo "=============================================================="
echo "  $PASS passed, $FAIL failed"
echo "  log: $LOG"
echo "=============================================================="
[ "$FAIL" -eq 0 ]
