#!/usr/bin/env bash
# Run AVL on the generated model and report the numbers params.py cannot.
#
# AVL is a vortex-lattice code. It gives the NEUTRAL POINT -- and therefore the
# real static margin -- plus Cm_alpha, Cn_beta, Cl_beta and the elevator angle
# needed to trim. params.py asserts a CG at 28% MAC and has never checked that
# the resulting aircraft is stable, or how much tail it burns holding trim.
#
# AVL is not packaged for Ubuntu. Build it once:
#   cd ~ && curl -fsSLO https://web.mit.edu/drela/Public/web/avl/avl3.36.tgz
#   tar xzf avl3.36.tgz && cd Avl
#   (cd plotlib  && make gfortran)
#   (cd eispack  && make -f Makefile.gfortran)
#   (cd bin      && make -f Makefile.gfortran)   # the final `install` step
#                                                # fails on a hardcoded path;
#                                                # the binary is already built
#   mkdir -p ~/.local/bin && cp bin/avl ~/.local/bin/
# Needs gfortran and libx11-dev.
#
# Usage:  bash aero/run_avl.sh  [CL]
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVL="${AVL:-$HOME/.local/bin/avl}"
CL="${1:-0.403}"          # cruise CL at 19 m/s and MTOW

command -v "$AVL" >/dev/null 2>&1 || [ -x "$AVL" ] || {
    echo "avl not found at $AVL -- see the build notes at the top of this file" >&2
    exit 1; }
[ -f "$HERE/tri_tiltrotor.avl" ] || {
    echo "run cad/gen_avl.py and cad/gen_airfoil_dat.py first" >&2; exit 1; }

cd "$HERE" || exit 1
OUT="$HERE/avl_out.txt"

# Feeding AVL on stdin always ends in "Fortran runtime error: End of file"
# because it wants another command after the last one. That is cosmetic -- the
# results are already printed. Do not go hunting for it.
{
    echo "load tri_tiltrotor.avl"
    echo "mass tri_tiltrotor.mass"
    echo "oper"
    echo "a c $CL"        # solve alpha for this lift coefficient
    echo "d2 pm 0"        # deflect the ruddervators until Cm = 0, i.e. trim it
    echo "x"                  # X already prints the total forces
    echo "st"                 # ...so ST is all that is needed after it
    # Do NOT add FT here. FT after ST makes AVL print no neutral point at all,
    # with no error message -- the ST output simply vanishes. Cost an hour.
    printf "\n\n\nquit\nquit\n"
} | "$AVL" > "$OUT" 2>&1

echo "=============================================================="
echo "AVL  --  trimmed at CL = $CL"
echo "=============================================================="
grep -E "Alpha =|CLtot|CDind|Cmtot|elevator  *=|^ *e =|    e =" "$OUT" | head -8
echo
grep -E "CLa =|Cma =|Cnb =|Clb =|Clp =|Cmq =|Cnr =" "$OUT" | head -8
echo
grep -E "Neutral point|spirally stable" "$OUT" | head -4

python3 - "$OUT" "$HERE/tri_tiltrotor.avl" <<'PY'
import re, sys
txt = open(sys.argv[1]).read()
m = re.search(r"Neutral point\s+Xnp\s*=\s*([-\d.]+)", txt)
if not m:
    raise SystemExit("no neutral point in the output")
xnp = float(m.group(1))

# Read Xref and Cref out of the .avl itself rather than hardcoding them.
# They were hardcoded once; moving the wing 24.4 mm to fix the MAC-position
# bug silently invalidated them and the static margin came out wrong by 9%
# MAC. A constant copied out of a generated file is a copy that will drift.
avl = open(sys.argv[2]).read().splitlines()
vals = [l.split() for l in avl if l.strip() and not l.lstrip().startswith("#")]
CREF = float(vals[3][1])        # Sref Cref Bref
XREF = float(vals[4][0])        # Xref Yref Zref
sm = (xnp - XREF) / CREF * 100
print(f"  Xref {XREF:.5f}  Cref {CREF:.5f}   (read from tri_tiltrotor.avl)")
print()
print(f"  STATIC MARGIN = (Xnp - Xref)/Cref = ({xnp:.5f} - {XREF:.5f})/{CREF}")
print(f"                = {sm:.1f}% MAC")
if sm < 0:
    print("  NEGATIVE -- statically unstable in pitch.")
elif sm < 8:
    print("  Low. Twitchy in pitch; fine only with active stabilisation.")
elif sm <= 15:
    print("  Healthy for this class.")
else:
    print("  HIGH. Very stable, but it costs trim drag, pitch authority and")
    print("  response. Either the tail is oversized or the CG wants to move aft.")
PY

echo
echo "full output: $OUT"
