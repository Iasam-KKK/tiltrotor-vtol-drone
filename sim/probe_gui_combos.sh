#!/usr/bin/env bash
# Probe which (render engine x Qt Quick backend) combination gives a Gazebo GUI
# client that survives under WSLg. Attaches to the RUNNING server, so it does
# not disturb the flight.
#
# ⚠ THIS PROBE ASKS THE WRONG QUESTION. It measures whether the `gz sim -g`
# PROCESS survives 14 s, which is NOT the same as a window appearing. Under
# WSLg a client can run happily forever having mapped nothing, or having mapped
# a 1x1 window -- and this script scores that as "SURVIVES". It correctly
# identified the QT_QUICK_BACKEND=software SEGFAULT, and nothing more.
# Use sim/probe_gui_window.sh, which measures window geometry, to decide
# anything about visibility.
#
# Throwaway diagnostic -- the winning combination gets baked into
# run_gui_client.sh and this file can be deleted.
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12

command -v gz >/dev/null || { echo "gz not found" >&2; exit 1; }

try() {
    local label="$1" engine="$2" backend="$3"
    pkill -f "gz sim.* -g" 2>/dev/null; sleep 1
    local log="/tmp/gzprobe_${label}.log"

    if [ -n "$backend" ]; then
        QT_QUICK_BACKEND="$backend" nohup gz sim -g --render-engine "$engine" \
            > "$log" 2>&1 &
    else
        env -u QT_QUICK_BACKEND nohup gz sim -g --render-engine "$engine" \
            > "$log" 2>&1 &
    fi

    sleep 14
    if pgrep -f "gz sim.* -g" >/dev/null; then
        echo "  SURVIVES   engine=$engine backend=${backend:-<unset>}"
        echo "$label" >> /tmp/gzprobe_winners
    else
        local why
        why=$(grep -m1 -iE "segmentation fault|abort|Err\]|failed" "$log" | head -1)
        echo "  DIED       engine=$engine backend=${backend:-<unset>}  ${why:-(no message)}"
    fi
}

rm -f /tmp/gzprobe_winners
echo "renderer: $(glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer' | head -1)"
echo

try ogre2_native ogre2 ""
try ogre_native  ogre  ""
try ogre2_soft   ogre2 software

pkill -f "gz sim.* -g" 2>/dev/null
echo
echo "winners: $(cat /tmp/gzprobe_winners 2>/dev/null | tr '\n' ' ')"
