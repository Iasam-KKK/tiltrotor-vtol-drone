#!/usr/bin/env bash
# What is available on this machine for flying the tri-tiltrotor by hand?
#
# Two candidate paths:
#   A) PX4 ROS 2 interface -- needs the uXRCE-DDS AGENT plus px4_msgs.
#      PX4's uxrce_dds_client is already running (it logs "init UDP agent
#      IP:127.0.0.1, port:8888" at startup) and is sitting there waiting for an
#      agent to connect. No agent = no ROS 2 topics, silently.
#   B) MAVLink -- pymavlink/MAVSDK over udp:14540. No agent, no message
#      packages, no build. Less "ROS", far less setup.
#
# Diagnostic only; reports, changes nothing.
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

echo "=== A. uXRCE-DDS agent (required for PX4 <-> ROS 2) ==="
found=""
for c in MicroXRCEAgent micro_ros_agent micro-xrce-dds-agent; do
    p=$(command -v "$c" 2>/dev/null) && { echo "  FOUND   $c -> $p"; found=1; }
done
[ -z "$found" ] && echo "  ABSENT  no agent binary on PATH"
echo "  apt candidates:"
apt-cache policy micro-xrce-dds-agent ros-jazzy-micro-ros-agent 2>/dev/null \
    | grep -E "^[a-z]|Candidate" | sed 's/^/    /' || echo "    (none)"

echo
echo "=== px4_msgs (ROS 2 message definitions) ==="
if command -v ros2 >/dev/null; then
    pkgs=$(ros2 pkg list 2>/dev/null | grep -E "^px4_msgs$|^px4_ros_com$")
    [ -n "$pkgs" ] && echo "  FOUND: $pkgs" || echo "  ABSENT  px4_msgs not on the ROS 2 path"
    echo "  ros_gz bridge: $(ros2 pkg list 2>/dev/null | grep -c ros_gz) packages"
else
    echo "  ros2 CLI not found"
fi

echo
echo "=== B. MAVLink route ==="
python3 - <<'PY'
for m in ("pymavlink", "mavsdk"):
    try:
        mod = __import__(m)
        print(f"  FOUND   {m}")
    except ImportError:
        print(f"  ABSENT  {m}")
PY
echo "  PX4 offboard MAVLink port: udp:14540 (mode Onboard, per startup log)"

echo
echo "=== Is PX4 up right now? ==="
pgrep -a px4 | head -2 || echo "  px4 not running"
