#!/usr/bin/env bash
# Where does DISPLAY=wayland-0 come from?
#
# xrdp creates an X server on :10.0, but every XFCE component reports
#     Unable to open display from environment variable DISPLAY='wayland-0'
# and exits, leaving a black desktop. `wayland-0` is a WAYLAND_DISPLAY value,
# not an X display -- something is assigning it to DISPLAY.
#
# /etc/xrdp/startwm.sh sources /etc/profile and ~/.profile BEFORE starting the
# session, so anything exported there lands in the session environment and
# overrides what xrdp set.
#
# Read-only.
echo "=== DISPLAY / WAYLAND_DISPLAY in shell startup files ==="
for f in /etc/profile ~/.profile ~/.bashrc ~/.xsessionrc /etc/environment; do
    [ -f "$f" ] || continue
    hits=$(grep -nE 'DISPLAY' "$f" 2>/dev/null)
    if [ -n "$hits" ]; then
        echo "  --- $f ---"
        echo "$hits" | sed 's/^/      /'
    fi
done

echo
echo "=== /etc/profile.d ==="
grep -rnE 'DISPLAY' /etc/profile.d/ 2>/dev/null | sed 's/^/  /' || echo "  (none)"

echo
echo "=== what WSL injects into every process ==="
echo "  DISPLAY=${DISPLAY:-<unset>}"
echo "  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>}"
echo "  XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-<unset>}"

echo
echo "=== /etc/wsl.conf (may disable WSLg entirely) ==="
cat /etc/wsl.conf 2>/dev/null | sed 's/^/  /' || echo "  (no /etc/wsl.conf)"

echo
echo "=== X servers actually running ==="
pgrep -af 'Xorg|Xvnc|Xrdp' | grep -v pgrep | sed 's/^/  /' || echo "  none"
ls /tmp/.X11-unix/ 2>/dev/null | sed 's/^/  socket: /' || echo "  no X11 sockets"
