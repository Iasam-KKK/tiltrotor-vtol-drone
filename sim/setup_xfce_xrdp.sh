#!/usr/bin/env bash
# Install XFCE + xrdp inside WSL2 so the Gazebo GUI renders in a real X session.
#
# WHY THIS EXISTS
# Under WSLg the Gazebo GUI window is created at full size, the process is
# healthy, and it composites SOLID BLACK. Measured, not guessed:
#   ogre2 + native Qt backend    window mapped 1000x845, black
#   ogre  + native Qt backend    (tested after the above; see notes)
#   any   + QT_QUICK_BACKEND=software   SEGFAULT in MinimalScene
# WSLg shares individual windows with the Windows compositor; that hand-off is
# what fails. An xrdp session composites INSIDE Linux and RDP ships a bitmap,
# so the failing path is not used at all.
#
# GPU: 3D stays on the RTX 5070. Mesa's d3d12 Gallium driver talks to /dev/dxg,
# which is available to ANY process in WSL, not just WSLg ones. RDP encoding is
# CPU-side, so the cap is transport, not the GPU.
#
# ⚠ THE TRAP THIS SCRIPT EXISTS TO AVOID
# An xrdp session does NOT source ~/.bashrc. Without GALLIUM_DRIVER set
# somewhere the session actually reads, Mesa silently falls back to llvmpipe
# and you get a working-but-crawling window with no error. Same class of bug as
# the one already documented in CLAUDE.md for `wsl -- bash foo.sh`.
#
# RUN THIS YOURSELF -- it needs sudo:
#     wsl -d Ubuntu-24.04
#     bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/setup_xfce_xrdp.sh
set -euo pipefail

# 3389 is taken by Windows' own RDP listener. Using it makes mstsc connect to
# Windows itself, which looks like "xrdp installed but shows my own desktop".
RDP_PORT=3390

echo "=== 1. packages ==="
sudo apt update
# xfce4-session needs dbus-x11 or the session dies immediately after login.
# x11-utils gives glxinfo/xwininfo, which is how we verify the GPU claim.
sudo DEBIAN_FRONTEND=noninteractive apt install -y \
    xfce4 xfce4-terminal xrdp dbus-x11 x11-utils mesa-utils

echo
echo "=== 2. xrdp on port $RDP_PORT ==="
sudo sed -i "s/^port=3389/port=$RDP_PORT/" /etc/xrdp/xrdp.ini
grep -m1 '^port=' /etc/xrdp/xrdp.ini | sed 's/^/  /'

# xrdp runs as the xrdp user and must be able to read its own key.
sudo adduser xrdp ssl-cert 2>/dev/null || true

echo
echo "=== 3. session = XFCE ==="
cat > "$HOME/.xsession" <<'EOF'
exec startxfce4
EOF
chmod +x "$HOME/.xsession"

# THE IMPORTANT BIT. ~/.xsessionrc is read by the xrdp session startup, unlike
# ~/.bashrc. Without this every GL app in the session lands on llvmpipe.
cat > "$HOME/.xsessionrc" <<'EOF'
# --- force X11, or the desktop comes up BLACK -------------------------------
# WSLg injects WAYLAND_DISPLAY=wayland-0 into every process in the distro, and
# GTK/GDK prefers Wayland whenever that is set. xfwm4 / xfce4-panel / xfdesktop
# then ignore xrdp's working X server on :10, try to reach a Wayland compositor
# that does not exist in an RDP session, and exit immediately -- a live session
# drawing nothing. Measured, not guessed:
#     (xfwm4): cannot open display: wayland-0
export GDK_BACKEND=x11         # GTK apps: XFCE itself
export QT_QPA_PLATFORM=xcb     # Qt apps: the Gazebo GUI
export XDG_SESSION_TYPE=x11
export CLUTTER_BACKEND=x11
unset WAYLAND_DISPLAY

# Hardware GL inside the xrdp session. Mesa's d3d12 driver reaches the RTX 5070
# through /dev/dxg. Without this line Mesa silently uses llvmpipe (software)
# and Gazebo runs at a few frames per second with no error message.
export GALLIUM_DRIVER=d3d12
# ogre2 is fine here -- the WSLg compositing path that broke it is not in use.
export PX4_GZ_SIM_RENDER_ENGINE=ogre2
# Never set this: it segfaults gz-gui (doneCurrent on a null QOpenGLContext).
unset QT_QUICK_BACKEND
EOF

# Belt and braces: also make it a system-wide default for non-login contexts.
if ! grep -q GALLIUM_DRIVER /etc/environment 2>/dev/null; then
    echo 'GALLIUM_DRIVER=d3d12' | sudo tee -a /etc/environment >/dev/null
fi

echo
echo "=== 4. start xrdp ==="
# WSL may or may not have systemd enabled; try both, in that order.
if command -v systemctl >/dev/null && systemctl list-units >/dev/null 2>&1; then
    sudo systemctl enable xrdp || true
    sudo systemctl restart xrdp
    systemctl is-active xrdp | sed 's/^/  xrdp: /'
else
    sudo service xrdp restart
    sudo service xrdp status | head -3 | sed 's/^/  /'
fi

echo
echo "=== 5. connect ==="
echo "  From Windows, open Remote Desktop (mstsc) and connect to:"
echo "      localhost:$RDP_PORT"
echo "  Username: $USER    Password: your WSL password"
echo
echo "  Session type at the login screen must be 'Xorg'."
echo
echo "=== 6. VERIFY THE GPU (do this INSIDE the xrdp desktop) ==="
echo "  Open a terminal in XFCE and run:"
echo "      glxinfo -B | grep -i 'OpenGL renderer'"
echo
echo "  D3D12 (NVIDIA GeForce RTX 5070)  -> hardware, correct"
echo "  llvmpipe                         -> SOFTWARE. The env var did not"
echo "                                      reach the session; re-check"
echo "                                      ~/.xsessionrc before flying."
echo
echo "  Then launch the sim from inside that desktop's terminal:"
echo "      bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/run_gui.sh"
