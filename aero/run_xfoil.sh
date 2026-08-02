#!/usr/bin/env bash
# Sweep the wing section at THIS aircraft's actual Reynolds numbers.
#
# WHY NOT JUST READ airfoiltools. Its polar grid is 50k / 100k / 200k / 500k /
# 1M. This aircraft lives at 186k (stall), 223k (loiter), 256k (best L/D) and
# 329k (cruise) -- so every operating point falls between two published curves,
# and the two that bracket cruise differ by nearly 40% in CD_min. Reading a
# number off the wrong curve is how CD_min = 0.0069 got into params.py in the
# first place: that is roughly the Re 1,000,000 value.
#
# NCRIT. Default is 5, not XFOIL's own default of 9. Ncrit 9 models a smooth,
# clean, low-turbulence surface. This wing is 3D printed. Ncrit 5 is the honest
# setting for a printed or field-handled surface and it is the difference
# between a plausible number and an optimistic one.
#
# Build XFOIL once (needs gfortran + libx11-dev, no sudo):
#   cd ~ && curl -fsSLO https://web.mit.edu/drela/Public/web/xfoil/xfoil6.99.tgz
#   tar xzf xfoil6.99.tgz && cd Xfoil
#   (cd plotlib && cp config.make.gfortranDP config.make && make)
#   (cd bin && make xfoil)      # the final `install` fails on a hardcoded
#                               # path; the binary is already built
#   mkdir -p ~/.local/bin && cp bin/xfoil ~/.local/bin/
#
# Usage:  bash aero/run_xfoil.sh [ncrit]
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XFOIL="${XFOIL:-$HOME/.local/bin/xfoil}"
NCRIT="${1:-5}"
FOIL="$HERE/naca2410.dat"

[ -x "$XFOIL" ] || { echo "xfoil not at $XFOIL -- see notes above" >&2; exit 1; }
[ -f "$FOIL" ]  || { echo "run cad/gen_airfoil_dat.py first" >&2; exit 1; }

cd "$HERE" || exit 1
rm -f polar_re*.txt

# The aircraft's real operating Reynolds numbers, from cad/gen_airfoil_dat.py.
for RE in 186165 223398 255779 329333; do
    OUT="polar_re${RE}_n${NCRIT}.txt"
    rm -f "$OUT"
    # PANE re-panels to XFOIL's own preferred distribution. Without it XFOIL
    # uses the file's points as-is and convergence at low Re gets flaky.
    {
        # Graphics OFF. XFOIL otherwise opens an X11 plot window, which under
        # WSLg composites solid black -- and then batch work looks broken for
        # a reason that has nothing to do with the aerodynamics.
        echo "plop"
        echo "G"
        echo ""
        echo "load $FOIL"
        echo "pane"
        echo "oper"
        echo "visc $RE"
        echo "vpar"
        echo "n $NCRIT"
        echo ""
        echo "iter 200"
        echo "pacc"
        echo "$OUT"
        echo ""
        echo "aseq -4 16 0.5"
        echo ""
        echo "quit"
    } | "$XFOIL" > /dev/null 2>&1
    echo "  Re $RE -> $OUT  ($( [ -f "$OUT" ] && grep -c '^ ' "$OUT" || echo 0 ) lines)"
done

python3 - "$NCRIT" <<'PY'
import glob, re, sys
ncrit = sys.argv[1]
print()
print(f"NACA 2410, Ncrit {ncrit}  --  what params.py assumes, measured")
print()
print(f"{'Re':>9} {'CD_min':>8} {'CL@CDmin':>9} {'CL_max':>7} {'a_stall':>8} "
      f"{'a_L0':>6} {'CL_a/deg':>9} {'CD@CL=0.875':>12}")
print("-" * 78)
for f in sorted(glob.glob("polar_re*.txt"),
                key=lambda p: int(re.search(r"re(\d+)", p).group(1))):
    rows = []
    for line in open(f):
        p = line.split()
        if len(p) >= 5:
            try:
                rows.append([float(x) for x in p[:5]])   # a, CL, CD, CDp, CM
            except ValueError:
                continue
    if not rows:
        continue
    re_n = int(re.search(r"re(\d+)", f).group(1))
    cdmin_row = min(rows, key=lambda r: r[2])
    clmax_row = max(rows, key=lambda r: r[1])
    # zero-lift alpha, linear interpolation across the CL sign change
    a_l0 = float("nan")
    for i in range(1, len(rows)):
        if rows[i-1][1] <= 0 <= rows[i][1]:
            x0, y0 = rows[i-1][0], rows[i-1][1]
            x1, y1 = rows[i][0], rows[i][1]
            a_l0 = x0 - y0 * (x1 - x0) / (y1 - y0)
            break
    # lift slope over the linear range, 0 to 5 deg
    lin = [r for r in rows if 0.0 <= r[0] <= 5.0]
    slope = ((lin[-1][1] - lin[0][1]) / (lin[-1][0] - lin[0][0])
             if len(lin) > 1 else float("nan"))
    # profile drag at the loiter operating CL
    near = min(rows, key=lambda r: abs(r[1] - 0.875))
    print(f"{re_n:>9,} {cdmin_row[2]:>8.5f} {cdmin_row[1]:>9.3f} "
          f"{clmax_row[1]:>7.3f} {clmax_row[0]:>8.2f} {a_l0:>6.2f} "
          f"{slope:>9.4f} {near[2]:>12.5f}")
print()
print("params.py asserts:  CD_min 0.00690   CL_max 1.400   "
      "a_stall 15.50   a_L0 -2.00   CL_a 0.1097/deg")
PY
