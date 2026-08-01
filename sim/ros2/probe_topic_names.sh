#!/usr/bin/env bash
# What topic names and message types does THIS PX4 actually use?
#
# PX4 v1.16+ introduced message VERSIONING: some topics gained a _v1 suffix.
# The agent log for this build showed, among others:
#     rt/fmu/out/vehicle_local_position_v1
#     rt/fmu/out/vehicle_status_v1
# while teleop_tiltrotor.py subscribes to the UNVERSIONED names. A subscriber
# on a non-existent topic is completely silent -- it just never fires, which
# presents as "streaming setpoints; waiting for PX4 position data..." forever.
#
# dds_topics.yaml in the firmware tree is the authority. Read it, don't guess.
set -o pipefail
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HOME/px4_ros2_ws/install/setup.bash" 2>/dev/null || true

YAML="$HOME/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml"

echo "=== firmware dds_topics.yaml: what we SUBSCRIBE to (fmu/out) ==="
grep -A2 'fmu/out' "$YAML" 2>/dev/null \
    | grep -E 'topic:|type:' \
    | grep -iE 'local_position|vehicle_status|vehicle_attitude' \
    | sed 's/^/  /'

echo
echo "=== firmware dds_topics.yaml: what we PUBLISH to (fmu/in) ==="
sed -n '/^subscriptions:/,/^publications/p' "$YAML" 2>/dev/null \
    | grep -E 'topic:|type:' \
    | grep -iE 'offboard_control_mode|trajectory_setpoint|vehicle_command' \
    | sed 's/^/  /'

echo
echo "=== live topics carrying real PX4 data (publisher count > 0) ==="
for t in $(timeout 15 ros2 topic list 2>/dev/null | grep '^/fmu/'); do
    n=$(timeout 6 ros2 topic info "$t" 2>/dev/null \
        | grep -i 'Publisher count' | tr -dc '0-9')
    s=$(timeout 6 ros2 topic info "$t" 2>/dev/null \
        | grep -i 'Subscription count' | tr -dc '0-9')
    printf '  %-52s pub=%s sub=%s\n' "$t" "${n:-0}" "${s:-0}"
done

echo
echo "=== px4_msgs types available for these ==="
ros2 interface list 2>/dev/null \
    | grep -iE 'px4_msgs.*(VehicleLocalPosition|VehicleStatus|TrajectorySetpoint|OffboardControlMode|VehicleCommand)' \
    | sed 's/^/  /'
