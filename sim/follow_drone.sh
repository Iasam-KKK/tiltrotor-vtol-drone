#!/usr/bin/env bash
# Make the Gazebo camera follow the aircraft indefinitely.
#
# Without this the camera is fixed at the origin, the aircraft climbs to 10 m
# and flies away, and you spend the whole test dragging the view around. Follow
# mode locks the camera to the model and keeps it there through takeoff,
# transition and cruise.
#
# Two ways to do this; the service call is the repeatable one:
#   GUI    right-click "tri_tiltrotor_0" in the Entity Tree -> Follow
#   here   gz service calls, so it survives being scripted and re-run
#
# The offset is expressed in the TARGET's frame, so the camera rotates with the
# aircraft -- which is what you want for watching the nacelles tilt during
# transition.
#
# ⚠ This gz-sim build advertises only TWO follow services, verified with
# `gz service -l | grep follow`:
#     /gui/follow          set the target
#     /gui/follow/offset   set the camera offset
# There is no /gui/follow/pgain and no /gui/follow/worldframe here, so smoothing
# and world-frame locking are not settable this way. Adjust them in the GUI's
# camera-tracking panel if you need them.
#
# Usage:
#   bash sim/follow_drone.sh                 # chase from behind and above
#   OFFSET="-3 0 1"  bash sim/follow_drone.sh   # closer
#   OFFSET="0 -6 2"  bash sim/follow_drone.sh   # side-on, good for tilt
#   bash sim/follow_drone.sh --stop          # release the camera
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12

command -v gz >/dev/null || { echo "gz not found" >&2; exit 1; }

# Chase camera: behind (-x), level (y=0), above (+z). Gazebo model frame is
# FLU, so -x is aft of the nose.
OFFSET="${OFFSET:--6 0 2}"

if [ "${1:-}" = "--stop" ]; then
    gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""' >/dev/null
    echo "follow released"
    exit 0
fi

# Find the model rather than hard-coding the instance suffix: PX4 spawns
# tri_tiltrotor_0, _1, ... depending on the instance number.
TARGET=$(timeout 8 gz model --list 2>/dev/null | grep -m1 -o 'tri_tiltrotor[_0-9]*')
if [ -z "$TARGET" ]; then
    TARGET=$(timeout 8 gz topic -l 2>/dev/null \
        | grep -m1 -o 'tri_tiltrotor[_0-9]*')
fi
[ -n "$TARGET" ] || { echo "ERROR: no tri_tiltrotor model found. Sim running?" >&2; exit 1; }
echo "target: $TARGET"

# These services are provided by gz-sim's CameraTracking GUI system. If they
# are absent the GUI is not running (a headless server has no camera).
if ! timeout 8 gz service -l 2>/dev/null | grep -q '/gui/follow'; then
    echo "ERROR: /gui/follow service not advertised." >&2
    echo "  The Gazebo GUI must be running -- a headless server has no camera." >&2
    exit 1
fi

call() {
    gz service -s "$1" --reqtype "$2" --reptype gz.msgs.Boolean \
        --timeout 3000 --req "$3" >/dev/null 2>&1 \
        && echo "  ok   $1" || echo "  FAIL $1"
}

echo "attaching camera:"
call /gui/follow gz.msgs.StringMsg "data: \"$TARGET\""

read -r ox oy oz <<< "$OFFSET"
call /gui/follow/offset gz.msgs.Vector3d "x: $ox, y: $oy, z: $oz"

echo
echo "camera now follows $TARGET at offset ($OFFSET)"
echo "release with:  bash sim/follow_drone.sh --stop"
