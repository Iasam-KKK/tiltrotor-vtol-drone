#!/usr/bin/env bash
# Attach a Gazebo GUI client to an ALREADY-RUNNING PX4/Gazebo server.
#
# Why this exists separately from run_gui.sh:
# PX4 starts its own `gz gui` from px4-rc.gzsim. Under WSLg that client
# sometimes appears for a second and then vanishes while the SERVER keeps
# running perfectly -- the aircraft is still flying, there is just no window.
# Killing the whole session to get a picture back also kills the flight, so
# this reattaches a fresh client instead.
#
# The server is `gz sim -s`, the client is `gz sim -g`. They are separate
# processes on one transport; you can restart the client as often as you like.
#
# Environment:
#   GALLIUM_DRIVER=d3d12    load-bearing. A script run as `bash foo.sh` does NOT
#                           source ~/.bashrc, so Mesa silently drops to llvmpipe.
#                           With it set, glxinfo reports D3D12 (RTX 5070).
#
# ⚠ QT_QUICK_BACKEND=software must NOT be set. It was inherited from an older
# workaround for a black GUI window, and it is now the cause of an immediate
# segfault: with the software Qt Quick backend there is no QOpenGLContext, but
# gz-gui's MinimalScene plugin calls doneCurrent() on it regardless ->
#
#   MinimalScene::updatePaintNode -> QOpenGLContext::doneCurrent()
#   Segmentation fault (Address not mapped to object [0x8])
#
# Probed 2026-08-01 against the live server (sim/probe_gui_combos.sh):
#   ogre2 + native backend    SURVIVES
#   ogre  + native backend    SURVIVES
#   ogre2 + software backend  SEGFAULT
# ogre2 aborting under WSLg was true only while Mesa was on llvmpipe; with
# GALLIUM_DRIVER=d3d12 applied it runs. Default to ogre2, override with $ENGINE.
#
# ⚠ "it runs" MEANS "it no longer aborts". IT STILL COMPOSITES SOLID BLACK
# UNDER WSLg -- a separate bug that d3d12 does not touch. Re-verified
# 2026-08-01 by pixel dump: ogre2 default loop, QSG_RENDER_LOOP=basic,
# xcb_egl and xcb_glx all mean=0.00 / nonzero=0.00%, and the whole Qt surface
# is black (toolbars too), not just the 3D viewport. Native Wayland fails
# earlier -- no DRM node in WSL, so EGL finds no device ("ZINK: failed to
# choose pdev"). ATTACH THIS CLIENT FROM THE xrdp SESSION, NOT FROM WSLg.
# Full reasoning and a faster transport: sim/setup_sunshine.sh
#
# Usage:  bash sim/run_gui_client.sh          # attach, stay in foreground
#         bash sim/run_gui_client.sh -b       # attach in background, log to /tmp
#         ENGINE=ogre bash sim/run_gui_client.sh -b
# NOTE: deliberately NOT `set -u`. ROS 2's setup.bash reads unbound variables,
# and under `set -u` sourcing it aborts the whole shell -- silently, if the
# source line is redirected to /dev/null. Cost 10 minutes once; don't re-add it.
set -o pipefail

export GALLIUM_DRIVER=d3d12
ENGINE="${ENGINE:-ogre2}"
# Inherited from the shell or a parent script, this segfaults gz-gui. See above.
unset QT_QUICK_BACKEND

# gz is not on PATH by default; it ships inside the ROS 2 Jazzy vendor tree.
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"

command -v gz >/dev/null || {
    echo "ERROR: gz not found. Expected /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz" >&2
    exit 1
}

# Refuse to attach to nothing -- otherwise the client opens an empty world and
# it looks like the aircraft failed to spawn.
if ! pgrep -f "gz sim.* -s" >/dev/null; then
    echo "ERROR: no Gazebo server running. Start the sim first:" >&2
    echo "    bash sim/run_gui.sh" >&2
    exit 1
fi

# One client at a time; two fight over the same world.
pkill -f "gz sim.* -g" 2>/dev/null
sleep 1

echo "renderer : $(glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer' | head -1 || echo '(glxinfo not installed)')"
echo "attaching GUI client to the running world..."

if [ "${1:-}" = "-b" ]; then
    nohup gz sim -g --render-engine "$ENGINE" > /tmp/gzgui.log 2>&1 &
    sleep 12
    if pgrep -f "gz sim.* -g" >/dev/null; then
        echo "GUI client attached (log: /tmp/gzgui.log)"
    else
        echo "GUI client exited. Log:" >&2
        tail -30 /tmp/gzgui.log >&2
        exit 1
    fi
else
    exec gz sim -g --render-engine "$ENGINE"
fi
