#!/usr/bin/env bash
# Launch the tri-tiltrotor in PX4 SITL with a Gazebo GUI.
#
# ⚠⚠ RUN THIS FROM THE xrdp SESSION, NOT FROM WSLg. CORRECTED 2026-08-01 (2nd).
# The file below says the GUI "actually renders under WSLg". It does not, and
# the correction lower down that says d3d12 makes ogre2 work under WSLg is
# addressing the wrong bug. There are TWO separate failures:
#
#   1. ABORT (SIGABRT, "failed to create OpenGL context") -- WAS llvmpipe.
#      GALLIUM_DRIVER=d3d12 genuinely fixes this one.
#   2. BLACK WINDOW -- NOT llvmpipe, NOT fixed by d3d12, still broken today.
#      The entire Qt surface composites black, toolbars and side panels
#      included, so it is not an Ogre or render-engine problem.
#
# Failure 2 re-measured 2026-08-01 by pixel-dumping the window (xwd + byte
# stats), every case mean=0.00, nonzero=0.00%:
#      ogre2, default Qt render loop          BLACK
#      ogre2, QSG_RENDER_LOOP=basic           BLACK
#      ogre2, QT_XCB_GL_INTEGRATION=xcb_egl   BLACK
#      ogre2, QT_XCB_GL_INTEGRATION=xcb_glx   BLACK
# and native Wayland (qtwayland5 IS installed) fails earlier still, because WSL
# exposes /dev/dxg rather than a DRM node so EGL enumerates no device:
#      libEGL warning: failed to get driver name for fd -1
#      MESA: error: ZINK: failed to choose pdev
#
# So the xrdp session in setup_xfce_xrdp.sh is still required, and its stated
# reason for existing is still correct. See sim/setup_sunshine.sh for a faster
# transport than xrdp (NVENC via Sunshine/Moonlight) -- it also keeps a real
# Xorg and only replaces the RDP encode path.
#
# HISTORICAL, and only true of failure 1:
# Gazebo's GUI defaults to the ogre2 render engine. Under WSLg ogre2 cannot
# create an OpenGL context and aborts:
#
#     glx: failed to create drisw screen
#     [GUI] [Err] [Application.cc:912] [QT] Failed to create OpenGL context
#     exit 134 (SIGABRT)
#
# ogre v1 runs cleanly on the same machine. This is not a GPU capability
# problem -- glxinfo reports OpenGL 4.6 core on D3D12 (RTX 5070), accelerated.
# It is the GLX path ogre2 uses, which WSLg does not provide.
#
# PX4 launches the GUI itself from ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim
# and passes PX4_GZ_SIM_RENDER_ENGINE through to BOTH the server and the GUI,
# so setting that one variable is the whole fix. Do not start a second GUI
# here -- PX4 already started one, and two clients on one world gives two
# windows fighting over it.
#
# GALLIUM_DRIVER=d3d12 is set explicitly because a script run as
# `bash run_gui.sh` does NOT source ~/.bashrc and Mesa silently drops to
# llvmpipe software rendering. See CLAUDE.md.

export GALLIUM_DRIVER=d3d12

# ⚠ CORRECTED 2026-08-01. This block used to set:
#     export PX4_GZ_SIM_RENDER_ENGINE=ogre
#     export QT_QUICK_BACKEND=software
# Both were workarounds for a Mesa stack that was silently on llvmpipe. With
# GALLIUM_DRIVER=d3d12 actually applied (renderer: D3D12, RTX 5070) ogre2 runs
# fine, and QT_QUICK_BACKEND=software became the BUG: with the software Qt
# Quick backend there is no QOpenGLContext, but gz-gui's MinimalScene plugin
# calls doneCurrent() on it anyway, and the client dies instantly --
#
#   MinimalScene::updatePaintNode -> QOpenGLContext::doneCurrent()
#   Segmentation fault (Address not mapped to object [0x8])
#
# which presents as "the GUI opened for a second then vanished" while the
# server kept running and the aircraft kept flying.
#
# Probed against the live server (sim/probe_gui_combos.sh):
#   ogre2 + native backend    SURVIVES   <- chosen
#   ogre  + native backend    SURVIVES   (fallback: PX4_GZ_SIM_RENDER_ENGINE=ogre)
#   ogre2 + software backend  SEGFAULT
export PX4_GZ_SIM_RENDER_ENGINE=ogre2
unset QT_QUICK_BACKEND

# If the GUI still dies while the sim keeps running, do NOT kill the session --
# reattach a fresh client to the same world:  bash sim/run_gui_client.sh -b

source /opt/ros/jazzy/setup.bash 2>/dev/null
export PATH="$HOME/.local/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH"

# A leftover server keeps advertising /world/default and the next run attaches
# to a world with no aircraft in it. PX4 refuses to start twice on instance 0,
# so a stale px4 is fatal to the next launch too.
pkill -f "gz sim" 2>/dev/null
pkill -x px4 2>/dev/null
sleep 2

echo "render engine : $PX4_GZ_SIM_RENDER_ENGINE   (ogre2 aborts under WSLg)"
echo "renderer      : $(glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer' | head -1 || echo '(glxinfo not installed)')"

cat <<'EOF'

Once you reach the pxh> prompt:
    commander takeoff        # hover to MIS_TAKEOFF_ALT (10 m)   [verified]
    commander transition     # hover -> forward flight           [untested]
    commander land
    shutdown

Harmless noise, expected:
  [Err] Failed to load system plugin [libGstCameraSystem.so]
        gstreamer dev packages absent; only affects camera streaming.
  Warning ... gz_frame_id ... not defined in SDF
        Gazebo extension element; every stock PX4 model uses it.
  Preflight Fail: ekf2 missing data
        clears once EKF converges, before "Ready for takeoff!".

EOF

cd "$PX4_DIR" || exit 1
make px4_sitl gz_tri_tiltrotor

pkill -f "gz sim" 2>/dev/null
