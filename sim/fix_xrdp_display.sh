#!/usr/bin/env bash
# Fix the black XFCE-over-xrdp desktop.
#
# DIAGNOSIS (measured, not guessed -- see sim/diag_xrdp.sh output):
#   xrdp's Xorg IS running on :10, and startxfce4 found it:
#       /usr/bin/startxfce4: X server already running on display :10.0
#   but every GTK component then died with:
#       (xfwm4): cannot open display: wayland-0
#       xfce4-panel: Unable to open display from environment DISPLAY='wayland-0'
#
#   Nothing in /etc/profile, ~/.profile, ~/.bashrc or /etc/profile.d sets
#   DISPLAY. The cause is that WSLg injects WAYLAND_DISPLAY=wayland-0 into
#   EVERY process in the distro, and GDK prefers the Wayland backend whenever
#   that variable is present. The XFCE components therefore ignore the working
#   X server, reach for a Wayland compositor that the RDP session cannot see,
#   and exit -- leaving a session that is alive and drawing nothing.
#
# THE FIX: pin the session to X11 explicitly. This does not disable WSLg for
# normal (non-RDP) use; it only affects processes started inside the xrdp
# session, because ~/.xsessionrc is read by /etc/X11/Xsession.d/40x11-common_*.
#
# Usage:  bash sim/fix_xrdp_display.sh      then log out of RDP and back in
set -euo pipefail

RC="$HOME/.xsessionrc"
[ -f "$RC" ] && cp "$RC" "$RC.bak-$(date +%s)" 2>/dev/null || true

cat > "$RC" <<'EOF'
# ---------------------------------------------------------------------------
# Read by /etc/X11/Xsession.d/40x11-common_xsessionrc at RDP session start.
# NOT the same as ~/.bashrc, which an xrdp session never sources.
# ---------------------------------------------------------------------------

# --- force X11, or the desktop is black ------------------------------------
# WSLg injects WAYLAND_DISPLAY=wayland-0 into every process in the distro.
# GTK/GDK prefers Wayland whenever that is set, so xfwm4 / xfce4-panel /
# xfdesktop ignore xrdp's perfectly good X server on :10, try to reach a
# Wayland compositor that does not exist in this session, and exit at once.
# The session stays alive and draws nothing.
unset WAYLAND_DISPLAY
export GDK_BACKEND=x11         # GTK apps: XFCE itself
export QT_QPA_PLATFORM=xcb     # Qt apps: the Gazebo GUI
export XDG_SESSION_TYPE=x11
export CLUTTER_BACKEND=x11

# --- hardware GL ------------------------------------------------------------
# Mesa's d3d12 Gallium driver reaches the RTX 5070 via /dev/dxg, which is
# available to any process in WSL -- not just WSLg ones. Without this Mesa
# silently falls back to llvmpipe: a working window at a few frames per second
# with no error explaining why.
export GALLIUM_DRIVER=d3d12

# ogre2 is fine here. It failed under WSLg because of the window-sharing
# hand-off, which an RDP session does not use.
export PX4_GZ_SIM_RENDER_ENGINE=ogre2

# NEVER set QT_QUICK_BACKEND=software: it segfaults gz-gui outright
# (MinimalScene calls doneCurrent() on a null QOpenGLContext).
unset QT_QUICK_BACKEND
EOF

chmod 644 "$RC"
echo "wrote $RC"
echo

# startwm.sh sources /etc/profile and ~/.profile before the session, so belt
# and braces: strip WAYLAND_DISPLAY there too, guarded so it only fires for
# xrdp sessions and never disturbs normal WSLg use.
SWM=/etc/xrdp/startwm.sh
if ! grep -q 'WAYLAND_DISPLAY' "$SWM" 2>/dev/null; then
    sudo cp "$SWM" "$SWM.bak-tritilt"
    sudo sed -i '2i # Added for WSL2: WSLg exports WAYLAND_DISPLAY into every process, which makes\n# GTK prefer a Wayland compositor this session cannot reach -> black desktop.\nunset WAYLAND_DISPLAY\nexport GDK_BACKEND=x11\nexport QT_QPA_PLATFORM=xcb\n' "$SWM"
    echo "patched $SWM (backup: $SWM.bak-tritilt)"
else
    echo "$SWM already patched"
fi

echo
echo "Now: log OUT of the RDP session and reconnect to localhost:3390."
echo "Then, inside the desktop's terminal:"
echo "    glxinfo -B | grep -i 'OpenGL renderer'    # want D3D12, not llvmpipe"
echo "    bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/run_gui.sh"
