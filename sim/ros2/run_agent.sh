#!/usr/bin/env bash
# Start the uXRCE-DDS agent that bridges PX4 <-> ROS 2.
#
# PX4's uxrce_dds_client is already running inside SITL and logs at startup:
#     INFO [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888
# It waits there for this agent. Until this runs there are NO /fmu/* topics --
# and nothing reports an error, which is the trap: `ros2 topic list` simply
# comes back short and it reads as a broken ROS install.
#
# Keep this running in its own terminal for as long as you want ROS control.
#
# Usage:  bash sim/ros2/run_agent.sh
set -o pipefail

PREFIX="$HOME/.local"
export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"

[ -x "$PREFIX/bin/MicroXRCEAgent" ] || {
    echo "ERROR: agent not built. Run sim/ros2/setup_px4_ros2.sh first." >&2
    exit 1; }

if ! pgrep -x px4 >/dev/null; then
    echo "⚠ PX4 is not running. Start it first (sim/run_gui.sh), otherwise the"
    echo "  agent will sit here with no client and no topics will appear."
    echo
fi

# A stale agent holds port 8888 and the new one dies with a bind error that is
# easy to misread as "PX4 refused the connection".
pkill -x MicroXRCEAgent 2>/dev/null
sleep 1

echo "agent on udp4:8888  (Ctrl-C to stop)"
echo "verify from another terminal:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    source ~/px4_ros2_ws/install/setup.bash"
echo "    ros2 topic list | grep fmu"
echo
exec "$PREFIX/bin/MicroXRCEAgent" udp4 -p 8888
