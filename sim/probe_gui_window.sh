#!/usr/bin/env bash
# Does the Gazebo GUI actually put a VISIBLE WINDOW on screen?
#
# ⚠ This exists because probe_gui_combos.sh asked the wrong question. It tested
# whether the `gz sim -g` PROCESS survived 14 seconds, and both native-backend
# combinations passed -- while showing no window at all. A live process with no
# window is precisely the failure being chased, so "survives" was never
# evidence of anything.
#
# The right measurement is the window's existence and GEOMETRY. The classic
# WSLg failure is a window that is created at 1x1 and never resized, which is
# invisible but perfectly healthy from the process's point of view.
#
# Usage:  bash sim/probe_gui_window.sh              # report on the current client
#         ENGINE=ogre bash sim/probe_gui_window.sh  # restart client, then report
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12
unset QT_QUICK_BACKEND

have_tool() { command -v "$1" >/dev/null; }

report_windows() {
    echo "  --- X client windows ---"
    if have_tool xwininfo; then
        # -root -tree lists every top-level window with its geometry.
        xwininfo -root -tree 2>/dev/null \
            | grep -iE "gazebo|gz sim|Gazebo Sim" \
            | sed 's/^/    /' || echo "    (no window matching gazebo)"
    elif have_tool wmctrl; then
        wmctrl -lG 2>/dev/null | sed 's/^/    /' || echo "    (wmctrl found nothing)"
    else
        echo "    NEITHER xwininfo NOR wmctrl installed -- cannot measure the"
        echo "    window. Install with:  sudo apt install -y x11-utils wmctrl"
        return 1
    fi
}

if [ -n "${ENGINE:-}" ]; then
    echo "restarting GUI client with engine=$ENGINE"
    pkill -f "gz sim.* -g" 2>/dev/null
    sleep 2
    nohup gz sim -g --render-engine "$ENGINE" > "/tmp/gzgui_$ENGINE.log" 2>&1 &
    sleep 15
fi

echo "process:"
pgrep -af "gz sim.* -g" | grep -v pgrep | sed 's/^/    /' || echo "    NOT RUNNING"
echo
report_windows
echo
echo "Interpretation:"
echo "  window listed at a real size (e.g. 1200x800)  -> GUI is genuinely up"
echo "  window listed as 1x1                          -> created but never sized"
echo "  no window at all                              -> client never mapped one"
