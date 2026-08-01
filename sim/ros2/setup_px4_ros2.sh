#!/usr/bin/env bash
# Stand up the PX4 <-> ROS 2 bridge for the tri-tiltrotor.
#
# PX4's uxrce_dds_client runs inside SITL already and logs at startup:
#     INFO [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888
# It then sits there waiting for an AGENT to connect. Without one there are no
# PX4 topics at all -- and nothing errors. `ros2 topic list` just comes back
# without /fmu/* and it looks like ROS is broken rather than absent.
#
# Installs to $HOME/.local, NOT /usr/local, so no sudo is required.
#
# Usage:  bash sim/ros2/setup_px4_ros2.sh
set -o pipefail

PREFIX="$HOME/.local"
WS="$HOME/px4_ros2_ws"
AGENT_SRC="$HOME/src/Micro-XRCE-DDS-Agent"
AGENT_TAG="${AGENT_TAG:-v3.0.0}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy" >&2; exit 1; }

step() { echo; echo "=== $* ==="; }

step "0. preconditions"
for c in git cmake colcon; do
    command -v "$c" >/dev/null && echo "  ok   $c" || { echo "  MISSING $c" >&2; exit 1; }
done
# PX4 message definitions must match the FIRMWARE, or topics decode to garbage
# or vanish. Read the version out of the tree rather than assuming.
PX4_VER=$(cd "$HOME/PX4-Autopilot" && git describe --tags 2>/dev/null | head -1)
echo "  PX4 firmware: ${PX4_VER:-unknown}"

step "1. Micro-XRCE-DDS-Agent ($AGENT_TAG) -> $PREFIX"
if [ -x "$PREFIX/bin/MicroXRCEAgent" ]; then
    echo "  already installed, skipping"
else
    mkdir -p "$(dirname "$AGENT_SRC")"
    [ -d "$AGENT_SRC" ] || git clone -b "$AGENT_TAG" --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_SRC" || exit 1
    mkdir -p "$AGENT_SRC/build"
    cd "$AGENT_SRC/build" || exit 1
    # UAGENT_*_PROFILE off trims the superbuild; we only need UDP.
    cmake .. -DCMAKE_INSTALL_PREFIX="$PREFIX" \
             -DCMAKE_BUILD_TYPE=Release \
             -DUAGENT_BUILD_EXECUTABLE=ON \
             -DUAGENT_ISOLATED_INSTALL=OFF || exit 1
    cmake --build . -j "$(nproc)" || exit 1
    cmake --install . || exit 1
fi

step "2. px4_msgs matching the firmware -> $WS"
mkdir -p "$WS/src"
cd "$WS/src" || exit 1
if [ ! -d px4_msgs ]; then
    git clone https://github.com/PX4/px4_msgs.git || exit 1
fi
cd px4_msgs || exit 1
# Prefer a release branch matching the firmware; fall back to main and SAY SO,
# because a silent mismatch here is the classic "topics exist but fields are
# wrong" failure.
BRANCH=""
for b in "release/1.17" "release/1.16"; do
    if git ls-remote --exit-code --heads origin "$b" >/dev/null 2>&1; then
        BRANCH="$b"; break
    fi
done
if [ -n "$BRANCH" ]; then
    git fetch origin "$BRANCH" --depth 1 && git checkout "$BRANCH" || exit 1
    echo "  px4_msgs on $BRANCH"
else
    echo "  ⚠ no release branch found; staying on default branch"
    echo "  ⚠ message definitions may not match $PX4_VER -- verify field names"
fi

step "3. build the workspace"
cd "$WS" || exit 1
colcon build --packages-select px4_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -6

step "4. verify"
source "$WS/install/setup.bash" 2>/dev/null
echo -n "  MicroXRCEAgent : "; "$PREFIX/bin/MicroXRCEAgent" --help >/dev/null 2>&1 \
    && echo "ok" || echo "FAILED"
echo -n "  px4_msgs       : "; ros2 pkg list 2>/dev/null | grep -qx px4_msgs \
    && echo "ok" || echo "FAILED"
echo -n "  TrajectorySetpoint fields: "
ros2 interface show px4_msgs/msg/TrajectorySetpoint 2>/dev/null \
    | grep -cE "^(float32|uint64)" || echo "?"

echo
echo "Next:"
echo "  bash sim/ros2/run_agent.sh        # in its own terminal, keep it running"
echo "  bash sim/ros2/teleop.sh           # keyboard control"
