#!/usr/bin/env bash
# Does the Gazebo GUI window contain a PICTURE, or is it solid black?
#
# Third iteration of this probe, because the first two measured proxies:
#   probe_gui_combos.sh  measured whether the process survived  -> useless
#   probe_gui_window.sh  measured whether a window was mapped   -> still useless
# Under WSLg the window is mapped at full size, the process is healthy, and it
# composites SOLID BLACK. Only the pixels settle it, so this grabs them.
#
# Candidate fixes tested here, in increasing order of desperation:
#   ogre2  + threaded Qt Quick render loop  (current default -- known black)
#   ogre2  + QSG_RENDER_LOOP=basic          (threaded loop is a classic
#                                            black-window cause on composited
#                                            or remoted displays)
#   ogre   + basic loop                     (ogre v1, simpler GL path)
#   ogre   + LIBGL_ALWAYS_SOFTWARE=1        (llvmpipe: slow but almost always
#                                            draws something)
#
# NOTE: QT_QUICK_BACKEND=software is deliberately never tested -- it segfaults
# gz-gui outright (MinimalScene calls doneCurrent() on a null QOpenGLContext).
#
# Usage:  bash sim/probe_gui_pixels.sh
set -o pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$HOME/.local/bin:$PATH"
export GALLIUM_DRIVER=d3d12
unset QT_QUICK_BACKEND

CAP=""
for t in import xwd ffmpeg; do command -v "$t" >/dev/null && { CAP="$t"; break; }; done
if [ -z "$CAP" ]; then
    echo "NO CAPTURE TOOL. Install one:"
    echo "    sudo apt install -y imagemagick     # gives 'import' (best)"
    echo "    sudo apt install -y x11-apps        # gives 'xwd'"
    exit 2
fi
echo "capture tool: $CAP"

win_id() {
    xwininfo -root -tree 2>/dev/null \
        | grep -i 'gz-sim-gui' | grep -v '1x1' | grep -v 'Selection Owner' \
        | awk '{print $1}' | head -1
}

# Fraction of pixels that are not near-black, via ImageMagick's histogram.
# A live 3D scene has sky, ground and an aircraft; a dead one is 100% black.
nonblack_frac() {
    local id="$1" png="/tmp/gzshot.png"
    rm -f "$png"
    case "$CAP" in
      import) import -window "$id" "$png" 2>/dev/null ;;
      xwd)    xwd -id "$id" 2>/dev/null | convert xwd:- "$png" 2>/dev/null ;;
      ffmpeg) return 2 ;;
    esac
    [ -s "$png" ] || { echo "capture-failed"; return 1; }
    python3 - "$png" <<'PY'
import sys, struct, zlib
# Minimal PNG reader: enough to average luminance without numpy/PIL.
d = open(sys.argv[1], 'rb').read()
pos, w, h, idat = 8, 0, 0, b''
while pos < len(d):
    ln = struct.unpack('>I', d[pos:pos+4])[0]; typ = d[pos+4:pos+8]
    if typ == b'IHDR':
        w, h, bd, ct = struct.unpack('>IIBB', d[pos+8:pos+18])
    elif typ == b'IDAT':
        idat += d[pos+8:pos+8+ln]
    pos += 12 + ln
raw = zlib.decompress(idat)
ch = {0:1, 2:3, 4:2, 6:4}.get(ct, 3)
stride = w*ch
prev = bytearray(stride); lit = 0; tot = 0; i = 0
for _ in range(h):
    f = raw[i]; i += 1
    line = bytearray(raw[i:i+stride]); i += stride
    for x in range(stride):                       # undo PNG filters
        a = line[x-ch] if x >= ch else 0
        b = prev[x]
        c = prev[x-ch] if x >= ch else 0
        if   f == 1: line[x] = (line[x]+a) & 255
        elif f == 2: line[x] = (line[x]+b) & 255
        elif f == 3: line[x] = (line[x]+(a+b)//2) & 255
        elif f == 4:
            p = a+b-c; pa, pb, pc = abs(p-a), abs(p-b), abs(p-c)
            pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
            line[x] = (line[x]+pr) & 255
    for x in range(0, stride, ch):
        tot += 1
        if line[x] > 24 or line[x+1 if ch > 1 else x] > 24:
            lit += 1
    prev = line
print(f"{lit/max(tot,1):.4f}")
PY
}

try() {
    local label="$1" engine="$2"; shift 2
    pkill -f "gz sim.* -g" 2>/dev/null; sleep 2
    env "$@" nohup gz sim -g --render-engine "$engine" \
        > "/tmp/gzpix_$label.log" 2>&1 &
    sleep 18
    local id; id=$(win_id)
    if [ -z "$id" ]; then echo "  $label: NO WINDOW"; return; fi
    local f; f=$(nonblack_frac "$id")
    if [ "$f" = "capture-failed" ]; then echo "  $label: capture failed"; return; fi
    awk -v l="$label" -v f="$f" 'BEGIN{
        printf "  %-26s non-black pixels %5.1f%%   %s\n", l, f*100,
               (f > 0.02 ? "<== HAS A PICTURE" : "solid black")}'
}

echo
try "ogre2_threaded"  ogre2 QSG_RENDER_LOOP=threaded
try "ogre2_basic"     ogre2 QSG_RENDER_LOOP=basic
try "ogre_basic"      ogre  QSG_RENDER_LOOP=basic
try "ogre_softwaregl" ogre  QSG_RENDER_LOOP=basic LIBGL_ALWAYS_SOFTWARE=1
echo
echo "Re-attach the winner with:  ENGINE=<engine> bash sim/run_gui_client.sh -b"
