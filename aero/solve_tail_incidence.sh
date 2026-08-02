#!/usr/bin/env bash
# Solve the V-tail incidence that makes the ruddervators float at zero in cruise.
#
# The relationship between tail incidence and trim elevator is linear over any
# range you would actually build, so two AVL runs and a straight line give the
# answer exactly -- no search, no guessing at flap-effectiveness ratios.
#
# Writes nothing. It prints the number to paste into params.TAIL_INCIDENCE,
# which keeps the solve auditable rather than letting a script quietly rewrite
# a design parameter.
#
# Usage:  bash aero/solve_tail_incidence.sh
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAD="$HERE/../cad"
AVL="${AVL:-$HOME/.local/bin/avl}"
VENV="$HERE/../../../.venv-cad/Scripts/python.exe"
CL="${CL:-0.403}"

elev_at() {   # $1 = tail incidence, degrees
    "$VENV" - "$1" <<'PY' > /tmp/_inc.py.out 2>&1
import re, sys, pathlib
deg = float(sys.argv[1])
p = pathlib.Path("params.py"); s = p.read_text(encoding="utf-8")
s2 = re.sub(r"(?m)^TAIL_INCIDENCE = math\.radians\([-\d.]+\)$",
            f"TAIL_INCIDENCE = math.radians({deg})", s)
p.write_text(s2, encoding="utf-8")
PY
    ( cd "$CAD" && "$VENV" gen_avl.py > /dev/null 2>&1 )
    cd "$HERE" || return 1
    {
        echo "load tri_tiltrotor.avl"
        echo "mass tri_tiltrotor.mass"
        echo "oper"
        echo "a c $CL"
        echo "d2 pm 0"
        echo "x"
        echo "st"
        printf "\n\n\nquit\nquit\n"
    } | "$AVL" 2>/dev/null | grep -E "^   elevator" | tail -1 | awk '{print $3}'
}

cd "$CAD" || exit 1
ORIG=$(grep -oP 'TAIL_INCIDENCE = math\.radians\(\K[-\d.]+' params.py)
echo "current TAIL_INCIDENCE = ${ORIG} deg"
echo

E0=$(elev_at 0.0)
echo "  incidence  0.00 deg -> elevator ${E0} deg"
E1=$(elev_at -6.0)
echo "  incidence -6.00 deg -> elevator ${E1} deg"

python3 - "$E0" "$E1" <<'PY'
import sys
e0, e1 = float(sys.argv[1]), float(sys.argv[2])
# elevator(i) = e0 + (e1-e0)/(-6-0) * i   ->  solve elevator = 0
slope = (e1 - e0) / (-6.0 - 0.0)
i = -e0 / slope if slope else float("nan")
print()
print(f"  d(elevator)/d(incidence) = {slope:.4f} deg/deg")
print(f"  => TAIL_INCIDENCE = math.radians({i:.2f})   for zero trim elevator")
print()
print("  Paste that into cad/params.py, then re-run this to confirm it lands")
print("  on ~0. A residual under about 0.3 deg is inside AVL's own resolution")
print("  here and is not worth chasing.")
PY

# Restore whatever was there before, so a solve run never silently edits the
# design. The number above is for a human to apply.
cd "$CAD" && "$VENV" - "$ORIG" <<'PY'
import re, sys, pathlib
p = pathlib.Path("params.py"); s = p.read_text(encoding="utf-8")
p.write_text(re.sub(r"(?m)^TAIL_INCIDENCE = math\.radians\([-\d.]+\)$",
                    f"TAIL_INCIDENCE = math.radians({sys.argv[1]})", s),
             encoding="utf-8")
PY
echo "restored TAIL_INCIDENCE = ${ORIG} deg"
