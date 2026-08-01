# STATUS 2026-08-01: ✅ WORKING, with SOFTWARE encoding. Confirmed usable by
# the user at 1920x1080 / H.264. Read the NVENC warning below before "fixing"
# the encoder setting back to nvenc -- that is not an optimisation, it is the
# bug that cost most of an evening.
#
# THE WORKING CONFIGURATION -- every line is load-bearing:
#     export DISPLAY=:10                      # a REAL Xorg, i.e. the xrdp one
#     export XAUTHORITY=$HOME/.Xauthority
#     unset WAYLAND_DISPLAY                   # ⚠ see trap 1 below
#     export GALLIUM_DRIVER=d3d12
# ~/.config/sunshine/sunshine.conf:
#     capture = x11
#     encoder = software                      # ⚠ NOT nvenc -- see trap 3
# Moonlight client: Video codec = H.264 (NOT Automatic), resolution 1920x1080
# to match the source exactly.
#
# ⛔ NVENC INITIALISES AND THEN ENCODES BLACK FRAMES. This is the nastiest trap
# here because every log line looks perfect:
#     Info: Creating encoder [h264_nvenc]
#     Info: Creating encoder [hevc_nvenc]
#     Info: Creating encoder [av1_nvenc]
#     Info: Streaming display: rdp0 with res 1920x1080
# ...and the client shows a black screen with a live mouse cursor. Observed
# three times, and the ONE run that produced a picture was the run whose config
# said `encoder = software` (log: "Creating encoder [libx264]"). NVENC is
# genuinely present (/usr/lib/wsl/lib/libnvidia-encode.so.1) and genuinely
# initialises; it just does not produce frames under WSL2. Do not re-test it
# hoping for a different answer.
#
# MEASURED COST, same glxgears 1600x900 benchmark as the xrdp baseline:
#     xrdp      90-130 FPS   xrdp 31% + Xorg 31%  ~0.6 core
#     sunshine  69-113 FPS   sunshine 82% + Xorg 41%  ~1.2 core
# So this costs ~2x the CPU for no throughput gain. It was still worth doing:
# the complaint was LATENCY, not framerate, and Moonlight's frame pacing plus
# client-side hardware decode fixes that where RDP cannot. 1.2 cores is ~6% of
# this 20-thread CPU, so it does not compete with PX4 or Gazebo.
#
# ⚠ xrdp is STILL REQUIRED -- not as the transport, but because an xrdp session
# is what creates the Xorg on :10 that Sunshine captures. No mstsc connection
# has ever happened -> no :10 -> "Unable to initialize capture method". Once the
# session exists you can disconnect mstsc; xrdp-sesman keeps the X server alive
# and xrdp stops encoding, which is the entire point.
#
# ⚠ TWO MORE THINGS, BOTH OF WHICH LOOK LIKE "SUNSHINE IS BROKEN" AND ARE NOT:
#
# 1. BLACK SCREEN IN MOONLIGHT -> it negotiated HEVC. Set Moonlight's
#    Settings -> Video codec -> H.264 explicitly. "Automatic" picks hevc_nvenc
#    here and the client decodes it as black, with no error on either side.
#
# 2. PICTURE BUT NO MOUSE OR KEYBOARD -> `Option "AutoAddDevices" "off"` in
#    /etc/X11/xrdp/xorg.conf. Sunshine injects input by creating kernel devices
#    through /dev/uinput -- verified working, they appear as event0/1/2:
#        N: Name="Mouse passthrough"
#        N: Name="Mouse passthrough (absolute)"
#        N: Name="Keyboard passthrough"
#    but that xorg.conf tells X to ignore every hotplugged device, so `xinput
#    list` on :10 shows ONLY xrdpMouse/xrdpKeyboard and the injected events go
#    nowhere. Video has no equivalent gate, which is why you get picture and no
#    control. Fix with the `inputfix` step. systemd + udevd are running, and
#    xserver-xorg-input-libinput is installed, so hotplug works once enabled.
#
# ⛔ PROVEN DEAD -- Wayland / XDG-portal / PipeWire capture:
#     Info:  [wayland] Found display [wayland-0]
#     Info:  Screencasting with XDG portal
#     Error: Missing extension: [EGL_EXT_image_dma_buf_import]
#     ... every encoder fails, INCLUDING software ...
#     Fatal: Unable to find display or encoder during startup
# The software encoder failing too proves this is not about NVENC -- NVENC is
# genuinely present (/usr/lib/wsl/lib/libnvidia-encode.so.1). Sunshine imports
# portal-captured frames as dma-bufs via EGL, and EGL_EXT_image_dma_buf_import
# needs a DRM device. WSL exposes /dev/dxg, NOT /dev/dri. Same root cause that
# makes native-Wayland gz-gui fail with "ZINK: failed to choose pdev". Also
# absent for the same reason: NvFBC (libnvidia-fbc.so.1) and kmsgrab.
#
# ⚠ THE TRAP THAT SENT IT DOWN THAT PATH: WSLg injects WAYLAND_DISPLAY=wayland-0
# into EVERY process in the distro, and Sunshine prefers Wayland whenever it is
# set -- even with DISPLAY=:10 exported and `capture = x11` in the config. You
# MUST `unset WAYLAND_DISPLAY`. This is the exact trap already documented in
# ~/.xsessionrc for xfwm4/xfce4-panel; it bites Sunshine identically.
#
# ❓ UNTESTED -- X11 capture (XGetImage/XShm, no EGL, no dma-buf, so none of the
# above applies). Two attempts so far were both INVALID, not negative:
#   1st: WAYLAND_DISPLAY still set  -> took the Wayland path, see above.
#   2nd: WAYLAND_DISPLAY unset, but `xdpyinfo` proved there was NO X server on
#        :10 at the time ("Unable to initialize capture method" / "Could not
#        open X display"). `service xrdp restart` during the retune step had
#        killed the session -- an xrdp session only exists while a client has
#        connected. RECONNECT mstsc FIRST, then test.
# `strings /usr/bin/sunshine` shows 6 x11 hits, so the backend is not obviously
# compiled out. Verdict genuinely unknown -- run `quicktest` properly.
#
# If X11 capture does turn out to be unavailable, the fallback is a transport
# that reads the X framebuffer directly and encodes on the CPU -- x11vnc,
# TurboVNC, KasmVNC. See the `vnc` step below.
#
# Retained below: the xrdp `retune` step, which is independently useful and has
# already been applied on this machine, and the measurements that motivated all
# of this.
#
# ---------------------------------------------------------------------------
# Original goal: replace xrdp's CPU-encoded RDP transport with Sunshine (NVENC)
# + Moonlight.
#
# WHY THIS EXISTS
# Measured 2026-08-01 on this machine, glxgears, both displays on
# D3D12 (NVIDIA GeForce RTX 5070), "Accelerated: yes":
#
#   display        320x320      1600x900     transport CPU @1600x900
#   xrdp :10       171-174 FPS  90-130 FPS   xrdp 31% + Xorg 31%
#   WSLg :0        165 FPS      120-131 FPS  0%
#
# Small window is parity, so the GPU is not the problem. xorgxrdp costs ~25-30%
# throughput at 1600x900 plus ~0.6 of a core spent purely on capture+encode --
# and the session actually runs at 2560x1440, 2.56x those pixels. On top of the
# throughput loss, RDP adds an encode -> TCP -> decode -> present round trip,
# which is what actually reads as "laggy" under the mouse.
#
# WSLg is not the escape hatch. gz-gui composites SOLID BLACK there -- the whole
# Qt surface, toolbars included, not just the 3D viewport. Verified black by
# pixel dump (mean=0.00, nonzero=0.00%) across ogre2 default loop,
# QSG_RENDER_LOOP=basic, QT_XCB_GL_INTEGRATION=xcb_egl and =xcb_glx; and native
# Wayland (qtwayland5 IS installed) dies differently -- EGL cannot find a device
# because WSL exposes /dev/dxg, not a DRM node:
#     libEGL warning: failed to get driver name for fd -1
#     MESA: error: ZINK: failed to choose pdev
# ⚠ This is NOT the llvmpipe bug. GALLIUM_DRIVER=d3d12 does not fix it. The
# comments in run_gui.sh / run_gui_client.sh claiming ogre2 works under WSLg
# with d3d12 are wrong -- the ABORT was llvmpipe, the BLACK WINDOW is separate.
#
# WHAT SUNSHINE DOES AND DOES NOT BUY YOU
# Verified present: /usr/lib/wsl/lib/libnvidia-encode.so.1  -> NVENC works in
# WSL2, so encode moves off the CPU and onto the 5070.
# Verified ABSENT: /dev/dri -> no KMS capture. Sunshine falls back to X11
# XGetImage, which is still CPU-side. You move ENCODE to the GPU; CAPTURE stays
# on the CPU. Expect a real gain, not a miracle. Step 3 measures it before you
# commit to the full rebuild in step 4.
#
# Run the steps yourself -- several need sudo.
set -euo pipefail

STEP="${1:-help}"
DUMMY_DISPLAY=":20"
RES="${RES:-1920x1080}"

case "$STEP" in

# ---------------------------------------------------------------------------
help)
cat <<'EOF'
Usage: bash sim/setup_sunshine.sh <step>

  retune     1. Cut xrdp's encode load ~55%. 2 min, no install. Do this first --
                it is a real win on its own and xrdp stays as your fallback.
  install    2. Install Sunshine + fix /dev/uinput permissions.
  quicktest  3. Point Sunshine at the EXISTING xrdp Xorg :10 and measure.
                Proves the gain before you build a new X server.
  standalone 4. Only if step 3 wins: dummy-driver Xorg, xrdp out of the loop.
  verify     Print renderer, NVENC, uinput, ports, and the IP for Moonlight.

Recommended order: retune -> install -> quicktest -> (measure) -> standalone
EOF
;;

# ---------------------------------------------------------------------------
retune)
echo "=== xrdp retune: 32bpp+high crypt at 2560x1440 is the expensive part ==="
sudo cp /etc/xrdp/xrdp.ini /etc/xrdp/xrdp.ini.bak.$(date +%s)
# 24bpp is 25% less data than 32bpp and visually identical for a sim window.
sudo sed -i 's/^max_bpp=.*/max_bpp=24/' /etc/xrdp/xrdp.ini
# This link never leaves the machine; TLS on localhost is pure CPU cost.
sudo sed -i 's/^crypt_level=.*/crypt_level=none/' /etc/xrdp/xrdp.ini
# tcp_nodelay=true is already set on this box; assert it rather than assume.
grep -q '^tcp_nodelay=true' /etc/xrdp/xrdp.ini || \
    sudo sed -i 's/^tcp_nodelay=.*/tcp_nodelay=true/' /etc/xrdp/xrdp.ini
grep -E '^(max_bpp|crypt_level|tcp_nodelay)=' /etc/xrdp/xrdp.ini | sed 's/^/  /'
sudo service xrdp restart || sudo systemctl restart xrdp
echo
echo "  Now RECONNECT mstsc at 1920x1080, not 2560x1440 -- resolution is chosen"
echo "  by the CLIENT at connect time, so the .ini cannot do it for you."
echo "  In mstsc: Options -> Display -> drag the slider off 'Full Screen'."
;;

# ---------------------------------------------------------------------------
install)
echo "=== 1. Sunshine ==="
# ⚠ VERIFY THE ASSET NAME against https://github.com/LizardByte/Sunshine/releases
# It has changed between releases; this is the Ubuntu 24.04 build as of writing.
DEB="sunshine-ubuntu-24.04-amd64.deb"
cd /tmp
if [ ! -f "$DEB" ]; then
    wget -O "$DEB" "https://github.com/LizardByte/Sunshine/releases/latest/download/$DEB"
fi
sudo apt install -y "./$DEB"

echo
echo "=== 2. /dev/uinput -- Moonlight's mouse and keyboard come back through this ==="
# Verified present on this kernel (6.18.33.2-microsoft-standard-WSL2,
# CONFIG_INPUT_UINPUT=m) but root-only: crw------- root root 10, 223.
# Without write access you get a perfect video stream you cannot click on.
sudo modprobe uinput 2>/dev/null || true
sudo groupadd -f input
sudo usermod -aG input "$USER"
sudo tee /etc/udev/rules.d/60-sunshine.rules >/dev/null <<'EOF'
KERNEL=="uinput", SUBSYSTEM=="misc", GROUP="input", MODE="0660", OPTIONS+="static_node=uinput"
EOF

# ⚠ WSL2 often runs without a udev daemon, so the rule above may never fire and
# /dev is rebuilt on every `wsl --shutdown`. Belt and braces: re-apply at boot.
sudo tee /usr/local/bin/wsl-uinput.sh >/dev/null <<'EOF'
#!/bin/sh
modprobe uinput 2>/dev/null
[ -e /dev/uinput ] && chgrp input /dev/uinput && chmod 660 /dev/uinput
EOF
sudo chmod +x /usr/local/bin/wsl-uinput.sh
sudo /usr/local/bin/wsl-uinput.sh

# ⚠ DO NOT just append "[boot]\ncommand=..." here. /etc/wsl.conf usually ALREADY
# has a [boot] section (systemd=true), and WSL's INI parser does not merge
# duplicate section headers -- the first [boot] wins and the second is silently
# ignored. That failure is invisible until the next `wsl --shutdown`, when
# /dev/uinput reverts to root:root 0600 and Sunshine streams video with a frozen
# pointer. Add the key INTO the existing section instead.
if ! grep -q 'wsl-uinput' /etc/wsl.conf 2>/dev/null; then
    sudo cp /etc/wsl.conf "/etc/wsl.conf.bak.$(date +%s)" 2>/dev/null || true
    if grep -q '^\[boot\]' /etc/wsl.conf 2>/dev/null; then
        # insert the key directly under the existing [boot] header
        sudo sed -i '0,/^\[boot\]/s|^\[boot\]|[boot]\ncommand = /usr/local/bin/wsl-uinput.sh|' /etc/wsl.conf
    else
        printf '[boot]\ncommand = /usr/local/bin/wsl-uinput.sh\n\n' | sudo tee -a /etc/wsl.conf >/dev/null
    fi
fi
echo "  --- /etc/wsl.conf (must contain exactly ONE [boot]) ---"
sed 's/^/    /' /etc/wsl.conf
grep -c '^\[boot\]' /etc/wsl.conf | sed 's/^/    [boot] sections: /'
ls -l /dev/uinput | sed 's/^/  /'
echo
echo "  ⚠ Log out and back in (or 'wsl --shutdown' from Windows) for the new"
echo "    'input' group membership to take effect. 'id' must list input."
;;

# ---------------------------------------------------------------------------
quicktest)
# The cheap experiment. Do NOT build a new X server yet -- the open question is
# whether Sunshine's CPU-side X11 capture beats xorgxrdp's CPU-side capture
# once encode is on NVENC. Reuse the xrdp Xorg that is already running and find
# out. If it does not win, you have installed one package and lost nothing.
echo "=== reusing the existing xrdp Xorg on :10 ==="
pgrep -f 'Xorg :10' >/dev/null || {
    echo "  No Xorg on :10. Connect mstsc to localhost:3390 once to spawn the"
    echo "  session, then disconnect -- xrdp-sesman keeps the X server alive."
    exit 1
}
mkdir -p "$HOME/.config/sunshine"
cat > "$HOME/.config/sunshine/sunshine.conf" <<EOF
capture = x11
encoder = nvenc
output_name = 0
min_log_level = info
EOF
echo "  config written to ~/.config/sunshine/sunshine.conf"
echo
echo "  Start it (foreground, so you can read the encoder line):"
echo "      export DISPLAY=:10 XAUTHORITY=\$HOME/.Xauthority"
echo "      unset WAYLAND_DISPLAY      # ⚠ LOAD-BEARING -- see header. Without"
echo "                                 # this, Sunshine ignores DISPLAY and"
echo "                                 # capture=x11, takes the Wayland/portal"
echo "                                 # path, and dies on dma-buf."
echo "      export GALLIUM_DRIVER=d3d12"
echo "      sunshine"
echo
echo "  Read the log in this order:"
echo "    1. '[wayland] Found display' or 'Screencasting with XDG portal'"
echo "         -> WAYLAND_DISPLAY leaked in. Fix that first; nothing else matters."
echo "    2. 'Unable to initialize capture method' / 'Could not open X display'"
echo "         -> no X server on :10. Check with:  DISPLAY=:10 xdpyinfo | head -3"
echo "         Connect mstsc to localhost:3390 once to spawn it, then disconnect."
echo "    3. 'Encoder [nvenc]' WITHOUT a following 'failed'  -> it works."
;;

# ---------------------------------------------------------------------------
standalone)
echo "=== dummy-driver Xorg on $DUMMY_DISPLAY at $RES -- xrdp fully out of the loop ==="
sudo apt install -y xserver-xorg-video-dummy

sudo tee /etc/X11/xorg-dummy.conf >/dev/null <<EOF
Section "Device"
    Identifier  "dummy"
    Driver      "dummy"
    VideoRam    256000
EndSection
Section "Monitor"
    Identifier  "mon"
    HorizSync   5.0 - 1000.0
    VertRefresh 5.0 - 200.0
    Modeline "1920x1080" 148.50 1920 2008 2052 2200 1080 1084 1089 1125 +hsync +vsync
EndSection
Section "Screen"
    Identifier  "scr"
    Device      "dummy"
    Monitor     "mon"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
    EndSubSection
EndSection
EOF

# WSL has no seat/console, so Xorg's setuid wrapper refuses to start for a
# normal user. This is the knob that lets it.
sudo tee /etc/X11/Xwrapper.config >/dev/null <<'EOF'
allowed_users=anybody
needs_root_rights=yes
EOF

sudo tee /usr/local/bin/start-sim-desktop.sh >/dev/null <<EOF
#!/usr/bin/env bash
# Standalone X + XFCE for Sunshine to capture. No xrdp involved.
export GALLIUM_DRIVER=d3d12       # or Mesa silently falls back to llvmpipe
export GDK_BACKEND=x11
export QT_QPA_PLATFORM=xcb
export XDG_SESSION_TYPE=x11
export CLUTTER_BACKEND=x11
unset WAYLAND_DISPLAY             # WSLg injects this; GTK would chase a
                                  # compositor that is not in this session
export DISPLAY=$DUMMY_DISPLAY

pgrep -f "Xorg $DUMMY_DISPLAY" >/dev/null || \
    (Xorg $DUMMY_DISPLAY -config /etc/X11/xorg-dummy.conf -nolisten tcp &)
sleep 3
pgrep -f xfwm4 >/dev/null || (startxfce4 &)
sleep 3
exec sunshine
EOF
sudo chmod +x /usr/local/bin/start-sim-desktop.sh

echo
echo "  Start:  GALLIUM_DRIVER=d3d12 /usr/local/bin/start-sim-desktop.sh"
echo "  Then:   sudo service xrdp stop      # keep it INSTALLED as a fallback"
;;

# ---------------------------------------------------------------------------
verify)
export GALLIUM_DRIVER=d3d12
echo "=== renderer on each display (must NOT say llvmpipe) ==="
for d in :0 :10 "$DUMMY_DISPLAY"; do
    r=$(DISPLAY=$d XAUTHORITY="$HOME/.Xauthority" glxinfo -B 2>/dev/null \
        | grep -i 'OpenGL renderer' | head -1)
    printf "  %-5s %s\n" "$d" "${r:-<no display>}"
done
echo
echo "=== NVENC ==="
ls -1 /usr/lib/wsl/lib/libnvidia-encode.so.1 2>/dev/null | sed 's/^/  /' || echo "  MISSING"
echo "=== uinput (needs group input, mode 660) ==="
ls -l /dev/uinput 2>/dev/null | sed 's/^/  /' || echo "  ABSENT"
id | tr ' ' '\n' | grep -o 'input' | head -1 | sed 's/^/  user in group: /' || echo "  ⚠ user NOT in group input"
echo
echo "=== Sunshine listening ==="
ss -tulnp 2>/dev/null | grep -E '4798[0-9]|4799[0-9]|480[01][0-9]' | sed 's/^/  /' || echo "  not running"
echo
echo "=== ⚠ POINT MOONLIGHT AT THIS, NOT localhost ==="
echo "  This is Windows 10 (19045): WSL2 has no 'mirrored' networking mode,"
echo "  that needs Win11 22H2+. NAT forwards localhost TCP but not the UDP the"
echo "  video stream uses (47998-48010). Use the VM IP directly:"
hostname -I | awk '{print "      "$1}'
echo "  It changes on every 'wsl --shutdown', so re-check it after a restart."
;;

# ---------------------------------------------------------------------------
autostart)
# Run ONCE, with sudo. After this you never run `start` again.
#
# The dependency is NOT "WSL booted" -- it is "Xorg :10 exists", and that only
# happens once mstsc has logged in. So the unit runs a wrapper that BLOCKS until
# the display appears. At boot it will sit waiting, which is correct behaviour,
# not a hang. The moment you connect mstsc, Sunshine comes up on its own.
#
# Restart=always then solves the other recurring annoyance: Sunshine dies when
# its X server dies (every XFCE logout). systemd restarts it, the wrapper waits
# for the new Xorg, and it reattaches without you touching anything.
echo "=== 1. wrapper that waits for the display ==="
sudo tee /usr/local/bin/sunshine-wait-and-run.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
# ⚠ unset, do not just leave it: WSLg injects WAYLAND_DISPLAY into every process
# and Sunshine prefers Wayland whenever it is set -- which takes the dma-buf
# path that cannot work without /dev/dri. This one line is load-bearing.
unset WAYLAND_DISPLAY
export DISPLAY=:10
export XAUTHORITY="$HOME/.Xauthority"
export GALLIUM_DRIVER=d3d12

# Block until the xrdp Xorg is up. No timeout on purpose -- there is nothing
# useful to do without it, and systemd would only restart us into the same wait.
until xdpyinfo >/dev/null 2>&1; do sleep 5; done
exec sunshine
EOF
sudo chmod +x /usr/local/bin/sunshine-wait-and-run.sh

echo "=== 2. systemd unit ==="
sudo tee /etc/systemd/system/sunshine.service >/dev/null <<EOF
[Unit]
Description=Sunshine stream host (waits for the xrdp Xorg on :10)
After=network-online.target

[Service]
Type=simple
User=$USER
# /dev/uinput is root:input 0660 -- without this the stream has no mouse or
# keyboard, which presents as "picture but frozen pointer".
SupplementaryGroups=input
ExecStart=/usr/local/bin/sunshine-wait-and-run.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "=== 3. enable ==="
sudo systemctl daemon-reload
sudo systemctl enable --now sunshine.service
sleep 3
systemctl --no-pager --lines=0 status sunshine.service | head -5 | sed 's/^/  /'
echo
echo "  Check it:      systemctl status sunshine"
echo "  Watch it:      journalctl -u sunshine -f"
echo "  Stop for good: sudo systemctl disable --now sunshine"
echo
echo "  ⚠ Do NOT also run the 'start' step -- two sunshines fight over the"
echo "    ports and the second one exits."
;;

# ---------------------------------------------------------------------------
start)
# Daily driver. Run this after every mstsc reconnect -- Sunshine is bound to a
# specific Xorg and DIES SILENTLY when that X server does, which happens on
# every XFCE logout and every `wsl --shutdown`. Symptom in Moonlight:
#   "Failed to initialize video capture/encoding. Is a display connected?" (503)
# That error means "no Xorg on :10 yet" or "sunshine is not running", NOT that
# something is broken.
export DISPLAY=:10
export XAUTHORITY="$HOME/.Xauthority"
unset WAYLAND_DISPLAY
export GALLIUM_DRIVER=d3d12

if ! DISPLAY=:10 xdpyinfo >/dev/null 2>&1; then
    echo "  No Xorg on :10. Connect mstsc to localhost:3390 first (1920x1080),"
    echo "  then re-run this. The X server only exists once a client has logged in."
    exit 1
fi

pkill -f sunshine 2>/dev/null; sleep 2
cd "$HOME"
setsid nohup sunshine > /tmp/sunshine_run.log 2>&1 < /dev/null &
disown
sleep 12

pgrep -x sunshine >/dev/null || { echo "  sunshine failed to start:"; tail -10 /tmp/sunshine_run.log; exit 1; }
echo "  sunshine pid $(pgrep -x sunshine)"
grep -iE 'Screencasting|Found H.264' /tmp/sunshine_run.log | tail -2 | sed 's/^/  /'
echo "  input devices attached to X:"
xinput list 2>/dev/null | grep -i passthrough | sed 's/^/    /' || echo "    ⚠ NONE -- see inputfix"
echo
echo "  Moonlight -> $(hostname -I | awk '{print $1}')  (H.264, 1920x1080)"
;;

# ---------------------------------------------------------------------------
inputfix)
# Moonlight shows a picture but mouse/keyboard do nothing. See header note 2.
echo "=== letting Xorg :10 see Sunshine's injected input devices ==="
sudo cp /etc/X11/xrdp/xorg.conf /etc/X11/xrdp/xorg.conf.bak.$(date +%s)
sudo sed -i 's/Option "AutoAddDevices" "off"/Option "AutoAddDevices" "on"/' \
    /etc/X11/xrdp/xorg.conf
grep -n 'AutoAddDevices' /etc/X11/xrdp/xorg.conf | sed 's/^/  /'
echo
echo "  Xorg only reads this file at startup, so the session must be recreated:"
echo "    1. Log out of XFCE (or disconnect mstsc and kill the session)"
echo "    2. Reconnect mstsc to localhost:3390   -> new Xorg, new config"
echo "    3. Start sunshine with the 4 env vars from the header"
echo "    4. Connect Moonlight with Video codec = H.264"
echo
echo "  Verify it took -- 'Mouse passthrough' and 'Keyboard passthrough' must"
echo "  appear alongside xrdpMouse/xrdpKeyboard:"
echo "      DISPLAY=:10 XAUTHORITY=\$HOME/.Xauthority xinput list"
echo
echo "  If they do NOT appear, X started before the devices existed and udev"
echo "  did not notify it. Start sunshine FIRST, then recreate the session."
;;

# ---------------------------------------------------------------------------
cleanup)
# FULL REVERT to stock xrdp. Removes Sunshine AND x11vnc and restores both
# config files from the backups taken before they were edited.
# Decision 2026-08-01: plain xrdp/Xorg is what gets used. Both alternatives
# cost more time than they returned. See the header for what was learned.
echo "=== 1. sunshine ==="
sudo systemctl disable --now sunshine.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/sunshine.service /usr/local/bin/sunshine-wait-and-run.sh
sudo systemctl daemon-reload 2>/dev/null || true
sudo apt purge -y sunshine 2>/dev/null || true
sudo rm -f /etc/udev/rules.d/60-sunshine.rules
rm -rf "$HOME/.config/sunshine" /tmp/sunshine_run.log

echo "=== 2. x11vnc ==="
pkill -x x11vnc 2>/dev/null || true
sudo apt purge -y x11vnc 2>/dev/null || true
rm -rf "$HOME/.vnc" /tmp/x11vnc.log

echo "=== 3. restore xrdp.ini from the pre-edit backup ==="
# Restoring the backup is safer than sed-ing values back: it returns the file
# byte-for-byte, including max_bpp=32 and crypt_level=high.
B=$(ls -1t /etc/xrdp/xrdp.ini.bak.* 2>/dev/null | tail -1)
if [ -n "$B" ]; then
    sudo cp "$B" /etc/xrdp/xrdp.ini && echo "  restored from $B"
else
    echo "  no backup found -- setting known-stock values instead"
    sudo sed -i 's/^max_bpp=.*/max_bpp=32/;s/^crypt_level=.*/crypt_level=high/' /etc/xrdp/xrdp.ini
fi

echo "=== 4. restore xrdp's xorg.conf (AutoAddDevices back to off) ==="
X=$(ls -1t /etc/X11/xrdp/xorg.conf.bak.* 2>/dev/null | tail -1)
if [ -n "$X" ]; then
    sudo cp "$X" /etc/X11/xrdp/xorg.conf && echo "  restored from $X"
else
    sudo sed -i 's/Option "AutoAddDevices" "on"/Option "AutoAddDevices" "off"/' \
        /etc/X11/xrdp/xorg.conf
fi

echo "=== 5. restart xrdp ==="
sudo service xrdp restart 2>/dev/null || sudo systemctl restart xrdp

echo
echo "=== verify ==="
grep -E '^(max_bpp|crypt_level|tcp_nodelay)=' /etc/xrdp/xrdp.ini | sed 's/^/  /'
grep -n 'AutoAddDevices' /etc/X11/xrdp/xorg.conf | sed 's/^/  /'
for p in sunshine x11vnc; do
    dpkg -l 2>/dev/null | grep -q "^ii  $p " && echo "  ⚠ $p STILL INSTALLED" || echo "  $p removed"
done
echo
echo "  Left in place on purpose, both harmless and unrelated to transport:"
echo "    /dev/uinput permissions + the wsl.conf [boot] command"
echo "    ~/.xsessionrc  -- STILL REQUIRED. GALLIUM_DRIVER=d3d12 and the"
echo "                      WAYLAND_DISPLAY unset are what make GL work at all"
echo "                      in the xrdp session. Do not delete it."
echo
echo "  Reconnect mstsc to localhost:3390. Nothing else to run."
;;

# ---------------------------------------------------------------------------
vnc)
# The replacement plan after Sunshine failed. x11vnc reads the X framebuffer
# with XGetImage/XShm and encodes with libjpeg-turbo -- no EGL, no dma-buf, no
# DRM node, so none of what killed Sunshine applies.
#
# It attaches to the Xorg xrdp ALREADY has running on :10, which makes this a
# clean A/B: same X server, same GL, only the transport swapped. Compare
# against the xrdp baseline of 90-130 FPS at 31% xrdp + 31% Xorg (1600x900).
echo "=== x11vnc on the existing Xorg :10 ==="
command -v x11vnc >/dev/null || sudo apt install -y x11vnc

if ! pgrep -f 'Xorg :10' >/dev/null; then
    echo "  No Xorg on :10 -- connect mstsc to localhost:3390 once to spawn it."
    echo "  You may disconnect afterwards; xrdp-sesman keeps the X server alive."
    exit 1
fi

# Optional but recommended. x11vnc listens on 0.0.0.0; under WSL2 NAT that is
# reachable from this Windows host but not the LAN. Still, a password costs
# nothing. Skip by setting NOPW=1.
PWFILE="$HOME/.vnc/passwd"
if [ "${NOPW:-0}" != "1" ] && [ ! -f "$PWFILE" ]; then
    mkdir -p "$HOME/.vnc"
    echo "  Set a VNC password (used only by the viewer):"
    x11vnc -storepasswd "$PWFILE"
fi
if [ -f "$PWFILE" ]; then AUTHARG="-rfbauth $PWFILE"; else AUTHARG="-nopw"; fi

pkill -x x11vnc 2>/dev/null; sleep 1
# ⚠⚠ THE SAME TRAP FOR THE THIRD TIME. WSLg injects WAYLAND_DISPLAY=wayland-0
# into EVERY process in the distro. x11vnc sees it, decides this is a Wayland
# session, and EXITS -- ignoring DISPLAY=:10 completely:
#     Wayland display server detected.
#     Wayland sessions are as of now only supported via -rawfb ... Exiting.
# It leaves no pid and (before the redirect below) no log, so it presents as
# "x11vnc silently did nothing" and the viewer says "actively refused".
# Previous victims of the identical variable: xfwm4/xfce4-panel (guarded in
# ~/.xsessionrc) and Sunshine (took the dma-buf path and failed every encoder).
# `env -u` on the exec line as well as the unset, because a parent shell or a
# desktop launcher can reintroduce it.
unset WAYLAND_DISPLAY
export DISPLAY=:10 XAUTHORITY="$HOME/.Xauthority" GALLIUM_DRIVER=d3d12

# Flag notes, all deliberate:
#  -noxdamage  XDAMAGE misses OpenGL window updates, so the Gazebo viewport goes
#              stale while the rest of the desktop repaints. Costs CPU (full
#              polling) but it is the difference between a live sim and a
#              frozen one. This is the single most important flag here.
#  -forever    keep serving after the viewer disconnects, instead of exiting
#  -shared     allow more than one viewer
#  -threads    separate thread per client; noticeably smoother under load
#  -ncache 0   client-side caching off; it causes artefacts with 3D content
setsid nohup env -u WAYLAND_DISPLAY \
    x11vnc -display :10 -auth "$HOME/.Xauthority" \
    -rfbport 5900 $AUTHARG -forever -shared -threads -noxdamage -ncache 0 \
    > /tmp/x11vnc.log 2>&1 < /dev/null &
disown
sleep 4

if pgrep -x x11vnc >/dev/null; then
    echo "  x11vnc pid $(pgrep -x x11vnc)"
    ss -tlnp 2>/dev/null | grep 5900 | sed 's/^/  /'
else
    echo "  FAILED to start:"; tail -12 /tmp/x11vnc.log; exit 1
fi
echo
echo "  On Windows:  winget install TigerVNC.TigerVNC"
echo "  Connect to:  $(hostname -I | awk '{print $1}'):5900"
echo "               NOT localhost -- Win10 has no WSL mirrored networking."
echo
echo "  ✅ Unlike Moonlight, VNC syncs the clipboard natively, so copy/paste"
echo "     between Windows and the session works without any bridge."
echo "     ⚠ In xfce4-terminal, paste is Ctrl+Shift+V, never Ctrl+V."
echo
echo "  Log: /tmp/x11vnc.log     Stop: pkill -x x11vnc"
;;

*) echo "unknown step: $STEP"; exit 1 ;;
esac
