#!/usr/bin/env bash
# Run the whole glide test unattended: PX4 + agent + scripted flight + measure.
#
# WHY THIS EXISTS. verify_glide.sh is the only script here that tests the
# AERODYNAMIC model rather than a mechanism -- it measures horizontal travel
# over altitude lost and compares it against the (L/D)max that params.py
# derives from the NACA section and the planform. But it needed a human in the
# RDP session pressing space, o, w, t, g at the right moments, so in practice
# it was the least-run check in the project. The drag polar was the single
# least-tested thing about the aircraft, which is exactly backwards.
#
# teleop_tiltrotor.py --script flies that key sequence on a timer. This
# orchestrates the three processes around it and then measures.
#
# The glide entry is detected by watching the teleop's own output rather than
# by sleeping a fixed time: PX4 boot and the first position fix vary by tens of
# seconds between runs, and a fixed sleep measures whatever happens to be
# underway -- usually the climb, which reads as a superb glide ratio.
#
# Usage:  bash sim/verify_glide_auto.sh
set -o pipefail

export GALLIUM_DRIVER=d3d12
unset WAYLAND_DISPLAY
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HOME/px4_ros2_ws/install/setup.bash" 2>/dev/null || true
export PATH="$HOME/.local/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOGDIR="$HOME/tritilt_logs"
mkdir -p "$LOGDIR"
STAMP=$(date +%H%M%S)
PX4_LOG="$LOGDIR/glide_px4_$STAMP.log"
TELEOP_LOG="$LOGDIR/glide_teleop_$STAMP.log"
AGENT_LOG="$LOGDIR/glide_agent_$STAMP.log"
GLIDE_HOLD="${GLIDE_HOLD:-75}"
RUN_S="${RUN_S:-340}"

echo "=============================================================="
echo "TRI-TILTROTOR GLIDE VERIFICATION  (unattended)"
echo "=============================================================="
echo "px4    : $PX4_LOG"
echo "teleop : $TELEOP_LOG"

cleanup() {
    kill "$TELEOP_PID" 2>/dev/null
    pkill -x MicroXRCEAgent 2>/dev/null
    pkill -x px4 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim-server" 2>/dev/null
}
trap cleanup EXIT

# --- clear strays ----------------------------------------------------------
# A leftover gz server keeps advertising /world/default and the next run
# attaches to a world with no aircraft in it.
pkill -f "gz sim" 2>/dev/null
pkill -f "gz-sim-server" 2>/dev/null
pkill -x px4 2>/dev/null
pkill -x MicroXRCEAgent 2>/dev/null
sleep 3

# --- PX4 + Gazebo, headless ------------------------------------------------
cd "$PX4_DIR" || exit 1
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"
# The `( sleep ... )` pipe is not decoration. With nothing on stdin PX4's pxh
# shell sees EOF, redraws its prompt in a tight loop, and writes ANSI
# clear-line escapes as fast as the disk accepts them -- a 2.3 GB log in two
# minutes, measured, which starves the simulation badly enough that the
# aircraft climbs away at 45 m/s during what is supposed to be a glide. Holding
# the pipe open costs nothing and keeps the shell quiet.
( sleep "$((RUN_S - 10))" ) \
  | HEADLESS=1 timeout "$RUN_S" make px4_sitl gz_tri_tiltrotor > "$PX4_LOG" 2>&1 &

echo
echo "[1] waiting for PX4 to be ready for takeoff..."
for i in $(seq 1 90); do
    grep -q "Ready for takeoff" "$PX4_LOG" 2>/dev/null && break
    sleep 2
done
if ! grep -q "Ready for takeoff" "$PX4_LOG" 2>/dev/null; then
    echo "  FAIL  PX4 never reported 'Ready for takeoff'"
    tail -20 "$PX4_LOG"
    exit 1
fi
echo "  PASS  PX4 up"

# --- uXRCE-DDS agent -------------------------------------------------------
# Must start AFTER PX4: the client retries, but starting first reliably lands
# in a state where the topics exist with no publisher behind them.
PREFIX="$HOME/.local"
export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"
"$PREFIX/bin/MicroXRCEAgent" udp4 -p 8888 > "$AGENT_LOG" 2>&1 &
sleep 6
echo "  PASS  uXRCE-DDS agent started"

# --- scripted flight -------------------------------------------------------
echo
echo "[2] flying arm -> offboard -> climb -> transition -> glide"
python3 -u "$HERE/ros2/teleop_tiltrotor.py" --script \
        --glide-hold "$GLIDE_HOLD" > "$TELEOP_LOG" 2>&1 &
TELEOP_PID=$!

for i in $(seq 1 120); do
    grep -q -- "-> GLIDE" "$TELEOP_LOG" 2>/dev/null && break
    kill -0 "$TELEOP_PID" 2>/dev/null || break
    sleep 2
done
if ! grep -q -- "-> GLIDE" "$TELEOP_LOG" 2>/dev/null; then
    echo "  FAIL  never entered the glide"
    tail -25 "$TELEOP_LOG"
    exit 1
fi
sed -n '/\[script/p' "$TELEOP_LOG"
echo "  PASS  glide entered"

# Let the entry transient wash out before measuring. The aircraft is still
# converging on the commanded velocity vector for the first few seconds.
sleep 8

# --- measure ---------------------------------------------------------------
echo
echo "[3] measuring against the derived polar"
DUR="${DUR:-40}" bash "$HERE/verify_glide.sh"
RC=$?

echo
echo "=============================================================="
echo "  px4 log    : $PX4_LOG"
echo "  teleop log : $TELEOP_LOG"
echo "=============================================================="
exit $RC
