#!/usr/bin/env bash
# Tail sizing study: what does shrinking the V-tail actually buy and cost?
#
# AVL reports the neutral point, so shrinking the tail and re-solving gives the
# static margin directly instead of by rule of thumb. It also gives the three
# things you PAY for a smaller tail with, which a volume-coefficient
# calculation cannot show you:
#   Cmq  pitch damping     -- scales with area x arm^2, so it falls fastest
#   Cnb  yaw stiffness     -- weathercock stability
#   Cnr  yaw damping       -- and with Cnr, the spiral mode
#
# Usage:  bash aero/sweep_tail.sh
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAD="$HERE/../cad"
AVL="${AVL:-$HOME/.local/bin/avl}"
VENV="$HERE/../../../.venv-cad/Scripts/python.exe"
CL="${CL:-0.403}"

printf "%6s %9s %9s %9s %9s %9s %9s %9s\n" \
       scale Xnp SM% elev_deg Cma Cmq Cnb spiral
printf -- "----------------------------------------------------------------------------\n"

for S in 1.00 0.90 0.80 0.70 0.60 0.50; do
    ( cd "$CAD" && "$VENV" gen_avl.py --tail-scale "$S" > /dev/null 2>&1 ) || continue
    F="tri_tiltrotor.avl"
    [ "$S" = "1.00" ] || F="tri_tiltrotor_t${S}.avl"
    [ -f "$HERE/$F" ] || { echo "  (missing $F)"; continue; }

    cd "$HERE" || exit 1
    {
        echo "load $F"
        echo "mass tri_tiltrotor.mass"
        echo "oper"
        echo "a c $CL"
        echo "d2 pm 0"
        echo "x"
        echo "st"
        # No FT here: FT after ST makes AVL emit no neutral point at all.
        printf "\n\n\nquit\nquit\n"
    } | "$AVL" > "/tmp/tail_${S}.txt" 2>&1

    python3 - "/tmp/tail_${S}.txt" "$S" <<'PY'
import re, sys
txt = open(sys.argv[1]).read()
scale = sys.argv[2]
def g(pat, default=float("nan")):
    m = re.search(pat, txt)
    return float(m.group(1)) if m else default
xnp  = g(r"Neutral point\s+Xnp\s*=\s*([-\d.]+)")
cma  = g(r"Cma =\s*([-\d.]+)")
cmq  = g(r"Cmq =\s*([-\d.]+)")
cnb  = g(r"Cnb =\s*([-\d.]+)")
spir = g(r"Clb Cnr / Clr Cnb\s*=\s*([-\d.]+)")
elev = g(r"elevator\s+=\s*([-\d.]+)")
XREF, CREF = 0.085425, 0.26
sm = (xnp - XREF) / CREF * 100 if xnp == xnp else float("nan")
print(f"{scale:>6} {xnp:9.5f} {sm:9.1f} {elev:9.2f} "
      f"{cma:9.3f} {cmq:9.2f} {cnb:9.4f} {spir:9.3f}")
PY
done

echo
echo "  SM healthy band is 8-15%.  Spiral > 1 is stable."
echo "  Cmq is pitch damping: it sets how quickly the short-period mode dies"
echo "  out, and it falls with area x arm^2, so it drops faster than the"
echo "  static margin does. Tail volume coefficient cannot see this."
