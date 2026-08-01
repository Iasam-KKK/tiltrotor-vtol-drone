#!/usr/bin/env bash
# Why is the XFCE session over xrdp a black screen?
#
# A black RDP desktop means the connection and login SUCCEEDED and the session
# then failed to draw. The three usual causes, all visible in the logs below:
#
#   1. xfce4-session exited immediately -- almost always a missing/!running
#      D-Bus. Shows up in ~/.xsession-errors as "Failed to connect to session
#      bus" or "dbus-launch: command not found".
#   2. /etc/xrdp/startwm.sh never reached ~/.xsession, so nothing was launched
#      at all. startwm.sh normally runs /etc/X11/Xsession, whose behaviour
#      depends on the distro; on Ubuntu it can silently pick a session that
#      isn't installed.
#   3. The X server started but no window manager did, leaving a root window
#      with nothing on it.
#
# Read-only. Changes nothing.
echo "=== xrdp / sesman services ==="
(systemctl is-active xrdp xrdp-sesman 2>/dev/null || service xrdp status 2>&1 | head -3) | sed 's/^/  /'

echo
echo "=== listening on 3390? ==="
(ss -ltnp 2>/dev/null | grep -E '3389|3390' || echo "  nothing listening") | sed 's/^/  /'

echo
echo "=== ~/.xsession-errors (the decisive file) ==="
if [ -f "$HOME/.xsession-errors" ]; then
    tail -40 "$HOME/.xsession-errors" | sed 's/^/  /'
else
    echo "  ABSENT -- the session script may never have run at all"
fi

echo
echo "=== /var/log/xrdp-sesman.log ==="
sudo -n tail -25 /var/log/xrdp-sesman.log 2>/dev/null | sed 's/^/  /' \
    || tail -25 /var/log/xrdp-sesman.log 2>/dev/null | sed 's/^/  /' \
    || echo "  unreadable without sudo"

echo
echo "=== what startwm.sh actually does ==="
grep -vE '^\s*#|^\s*$' /etc/xrdp/startwm.sh 2>/dev/null | sed 's/^/  /'

echo
echo "=== session files present? ==="
for f in "$HOME/.xsession" "$HOME/.xsessionrc"; do
    if [ -f "$f" ]; then echo "  ok      $f"; sed 's/^/            /' "$f"
    else echo "  MISSING $f"; fi
done

echo
echo "=== required binaries ==="
for b in startxfce4 xfce4-session dbus-launch dbus-daemon xfwm4 xfdesktop; do
    p=$(command -v "$b" 2>/dev/null)
    printf '  %-16s %s\n' "$b" "${p:-MISSING}"
done

echo
echo "=== dbus running? ==="
(pgrep -a dbus-daemon | head -3 || echo "  no dbus-daemon") | sed 's/^/  /'
ls -ld /run/dbus 2>/dev/null | sed 's/^/  /' || echo "  /run/dbus absent"
