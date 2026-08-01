#!/usr/bin/env bash
# Fly the tri-tiltrotor headless and record it to MP4.
#
# No GUI, no window, no compositor. Gazebo renders the cameras offscreen and
# publishes image topics; ros_gz_image bridges them into ROS 2; record_video.py
# encodes with OpenCV. Every step of that chain is verified working on this
# machine, unlike the Gazebo GUI (window is created and stays 1x1 or renders
# black, process healthy, no errors -- a WSLg defect) and unlike Gazebo's own
# <save> element (writes nothing, dumps core).
#
# Bonus: the ROS 2 hop is not a workaround, it is the deliverable. It proves
# the aircraft is drivable from ROS 2, which is the positioning.
#
# Produces:
#     media/master_16x9.mp4        1920x1080  -> YouTube / LinkedIn
#     media/master_vertical.mp4    1080x1920  -> Shorts / Reels
#
# Usage:  bash capture_video.sh [seconds]
set -o pipefail

export GALLIUM_DRIVER=d3d12
export PX4_GZ_SIM_RENDER_ENGINE=ogre
source /opt/ros/jazzy/setup.bash 2>/dev/null
export PATH="$HOME/.local/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname "$HERE")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
MEDIA="$PROJECT/media"
LOGDIR="$HOME/tritilt_logs"
RUN_SECONDS="${1:-120}"

mkdir -p "$MEDIA" "$LOGDIR"
STAMP=$(date +%H%M%S)

# Cameras live ON the aircraft, not in the world. A world-fixed camera loses
# the aircraft within ~15 s of takeoff -- the first capture run produced 30 s
# of empty field. Model-mounted cameras track it for free.
GZ_WIDE="/world/capture/model/tri_tiltrotor_0/link/chase_cam_link/sensor/chase_cam/image"
GZ_VERT="/world/capture/model/tri_tiltrotor_0/link/nacelle_cam_link/sensor/nacelle_cam/image"

echo "=============================================================="
echo "TRI-TILTROTOR HEADLESS VIDEO CAPTURE"
echo "=============================================================="

ln -sfn "$HERE/worlds/capture.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/capture.sdf"

pkill -f "gz sim" 2>/dev/null
pkill -x px4 2>/dev/null
pkill -f image_bridge 2>/dev/null
pkill -f record_video 2>/dev/null
sleep 3

# --- fly it ----------------------------------------------------------------
cd "$PX4_DIR" || exit 1
(
  sleep 34
  echo "commander takeoff"
  sleep 26
  echo "commander transition"
  sleep 40
  echo "shutdown"
  sleep 5
) | HEADLESS=1 PX4_GZ_WORLD=capture \
    timeout "$RUN_SECONDS" make px4_sitl gz_tri_tiltrotor \
    > "$LOGDIR/capture_$STAMP.log" 2>&1 &
PX4_PID=$!

# --- wait for the camera topics to exist -----------------------------------
echo "waiting for camera topics..."
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "cam_16x9/image"; then
        echo "  cameras are publishing"
        break
    fi
    sleep 2
done

# --- bridge gz -> ROS 2 ----------------------------------------------------
ros2 run ros_gz_image image_bridge "$GZ_WIDE" "$GZ_VERT" \
    > "$LOGDIR/bridge_$STAMP.log" 2>&1 &
BRIDGE_PID=$!
sleep 6
echo "bridged ROS topics:"
ros2 topic list 2>/dev/null | grep -iE "cam_16x9|cam_vertical" | sed 's/^/    /'

# --- record ----------------------------------------------------------------
python3 "$HERE/record_video.py" --topic "$GZ_WIDE" \
    --out "$MEDIA/master_16x9.mp4" --fps 30 \
    > "$LOGDIR/rec_wide_$STAMP.log" 2>&1 &
REC1=$!
python3 "$HERE/record_video.py" --topic "$GZ_VERT" \
    --out "$MEDIA/master_vertical.mp4" --fps 30 \
    > "$LOGDIR/rec_vert_$STAMP.log" 2>&1 &
REC2=$!

echo "recording..."
wait $PX4_PID 2>/dev/null
sleep 8

kill $REC1 $REC2 2>/dev/null
sleep 3
kill $BRIDGE_PID 2>/dev/null
pkill -f "gz sim" 2>/dev/null

# --- report ----------------------------------------------------------------
echo
echo "=== recorder logs ==="
tail -3 "$LOGDIR/rec_wide_$STAMP.log" 2>/dev/null | sed 's/^/  wide: /'
tail -3 "$LOGDIR/rec_vert_$STAMP.log" 2>/dev/null | sed 's/^/  vert: /'

echo
echo "=== flight ==="
grep -iE "Takeoff detected|Transition|VTOL|Armed by" "$LOGDIR/capture_$STAMP.log" \
    2>/dev/null | head -8

echo
echo "=== output ==="
ls -la "$MEDIA"/*.mp4 2>/dev/null || echo "  no mp4 produced"

OK=0
for f in "$MEDIA/master_16x9.mp4" "$MEDIA/master_vertical.mp4"; do
    [ -s "$f" ] && OK=$((OK+1))
done
echo
echo "  $OK/2 videos produced"
[ "$OK" -gt 0 ]
