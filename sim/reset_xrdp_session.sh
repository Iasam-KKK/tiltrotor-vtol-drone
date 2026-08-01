#!/usr/bin/env bash
# Destroy the existing xrdp session so the next connection builds a fresh one.
#
# WHY THIS IS NEEDED, and why "log out and back in" was not enough:
# xrdp's sesman keeps a session per user and REATTACHES to it on reconnect.
# Closing the RDP window only disconnects; the Xorg server, xfce4-session and
# their whole environment survive. So environment changes in ~/.xsessionrc or
# /etc/xrdp/startwm.sh are NOT picked up by reconnecting -- the session was
# created before they existed and keeps its original environment forever.
#
# Proof this was happening here: after applying the display fix and
# reconnecting, ~/.xsession-errors was byte-identical, same 17:32:30
# timestamps. A new session would have rewritten it.
#
# Usage:  bash sim/reset_xrdp_session.sh     then reconnect to localhost:3390
set -o pipefail

echo "=== before ==="
pgrep -af 'Xorg :10|xfce4-session' | grep -v pgrep | sed 's/^/  /' || echo "  (no session)"

echo
echo "=== killing the session ==="
pkill -u "$USER" -f xfce4-session 2>/dev/null && echo "  killed xfce4-session"
pkill -u "$USER" -f 'xfwm4|xfdesktop|xfce4-panel|xfsettingsd' 2>/dev/null \
    && echo "  killed xfce components"
sleep 1
sudo pkill -f 'Xorg :10' 2>/dev/null && echo "  killed Xorg :10"
sleep 1

# sesman caches session state; restarting it guarantees a clean slate.
if command -v systemctl >/dev/null && systemctl list-units >/dev/null 2>&1; then
    sudo systemctl restart xrdp-sesman xrdp
    echo "  restarted xrdp-sesman + xrdp"
else
    sudo service xrdp-sesman restart; sudo service xrdp restart
    echo "  restarted xrdp (sysv)"
fi

# Move the old log aside so the next session's errors are unambiguous -- this
# is what made the stale-session diagnosis take an extra round.
[ -f "$HOME/.xsession-errors" ] && \
    mv "$HOME/.xsession-errors" "$HOME/.xsession-errors.old" && \
    echo "  archived old ~/.xsession-errors"

echo
echo "=== after ==="
pgrep -af 'Xorg :10|xfce4-session' | grep -v pgrep | sed 's/^/  /' \
    || echo "  clean -- no session running"

echo
echo "Now reconnect Remote Desktop to localhost:3390."
echo "A NEW session will be created and will read the fixed environment."
echo
echo "If it is still black, check the FRESH log:"
echo "    grep -iE 'wayland|display' ~/.xsession-errors | head"
