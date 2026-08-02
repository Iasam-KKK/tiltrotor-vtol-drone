#!/usr/bin/env bash
# Keyboard teleoperation for the tri-tiltrotor.
#
# Requires, in this order:
#   1. PX4 SITL + Gazebo running      -> sim/run_gui.sh
#   2. the uXRCE-DDS agent running    -> sim/ros2/run_agent.sh
#   3. this
#
# Usage:  bash sim/ros2/teleop.sh
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/px4_ros2_ws"

source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "ERROR: ROS 2 Jazzy not found" >&2; exit 1; }
source "$WS/install/setup.bash" 2>/dev/null || {
    echo "ERROR: px4_msgs workspace not built. Run sim/ros2/setup_px4_ros2.sh" >&2
    exit 1; }

pgrep -x MicroXRCEAgent >/dev/null || {
    echo "⚠ the uXRCE-DDS agent is NOT running -- there will be no /fmu topics."
    echo "  Start it first:  bash sim/ros2/run_agent.sh"
    echo
}

# Fail loudly rather than sitting at a prompt that never responds.
if ! timeout 8 ros2 topic list 2>/dev/null | grep -q '/fmu/out/'; then
    echo "⚠ no /fmu/out/* topics visible yet."
    echo "  Check, in order: PX4 running, agent running, then retry."
    echo
fi

# "$@" matters: sim/verify_glide.sh drives this with --script, and without it
# the flags are swallowed and the node sits waiting for keys that never come.
exec python3 "$HERE/teleop_tiltrotor.py" "$@"
