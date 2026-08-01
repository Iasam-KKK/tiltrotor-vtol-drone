#!/usr/bin/env bash
# One-shot status of the whole simulation stack.
#
# Reports what is ACTUALLY true, not what should be true. Two traps this exists
# to avoid, both of which produced wrong conclusions during bring-up:
#
#   1. `pgrep -f "gz sim -g"` does NOT match PX4's client, whose command line is
#      `gz sim --render-engine ogre2 -g`. The literal substring isn't there, so
#      a running GUI reports as NOT RUNNING.
#
#   2. A topic appearing in `ros2 topic list` proves NOTHING about the bridge.
#      A local subscriber (the teleop node) makes /fmu/out/* appear with no
#      agent connected at all. Only a non-zero PUBLISHER count proves PX4 is
#      actually talking.
#
# Usage:  bash sim/status.sh
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HOME/px4_ros2_ws/install/setup.bash" 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
export GALLIUM_DRIVER=d3d12

line() { printf '  %-22s %s\n' "$1" "$2"; }
alive() { pgrep -f "$1" >/dev/null && echo "RUNNING" || echo "-- not running --"; }

echo "=== processes ==="
line "PX4 SITL"        "$(alive '^/.*bin/px4$|bin/px4')"
line "Gazebo server"   "$(alive 'gz sim.* -s')"
line "Gazebo GUI"      "$(alive 'gz sim.* -g')"
line "uXRCE-DDS agent" "$(alive 'MicroXRCEAgent')"
line "teleop node"     "$(alive 'teleop_tiltrotor.py')"

echo
echo "=== GUI window (geometry is the only proof of visibility) ==="
if command -v xwininfo >/dev/null; then
    out=$(xwininfo -root -tree 2>/dev/null | grep -iE 'gazebo|gz-sim-gui')
    if [ -n "$out" ]; then
        echo "$out" | sed 's/^/  /'
        echo "$out" | grep -q '1x1' && \
            echo "  ⚠ a 1x1 window is present -- that one is invisible by definition"
    else
        echo "  no gazebo window mapped"
    fi
else
    echo "  xwininfo absent (sudo apt install -y x11-utils) -- cannot verify"
fi

echo
echo "=== PX4 <-> ROS 2 bridge ==="
if command -v ros2 >/dev/null; then
    pubs=$(timeout 12 ros2 topic info /fmu/out/vehicle_status 2>/dev/null \
           | grep -i 'Publisher count' | tr -dc '0-9')
    line "vehicle_status pubs" "${pubs:-0}"
    if [ "${pubs:-0}" -gt 0 ]; then
        echo "  bridge is LIVE -- PX4 telemetry is reaching ROS 2"
    else
        echo "  bridge is DEAD -- topics may still be listed by local nodes."
        echo "  Start the agent:  bash sim/ros2/run_agent.sh"
    fi
else
    echo "  ros2 CLI not found"
fi

echo
echo "=== nacelle tilt (0 = hover, 90 = cruise, split = yaw) ==="
T=$(timeout 8 gz topic -l 2>/dev/null | grep -m1 joint_state)
if [ -n "$T" ]; then
    timeout 6 gz topic -e -t "$T" 2>/dev/null | grep -A2 -m2 'tilt_.*_joint' \
        | sed 's/^/  /' | head -8
else
    echo "  no joint_state topic (Gazebo not running?)"
fi
