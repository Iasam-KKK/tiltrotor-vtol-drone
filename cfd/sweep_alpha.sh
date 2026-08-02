#!/usr/bin/env bash
# Sweep angle of attack and build a MEASURED drag polar to set against the one
# params.py derives.
#
# One point is not a polar. CD0 and the induced-drag factor cannot be separated
# from a single alpha: you need CD plotted against CL^2, whose intercept is CD0
# and whose slope is 1/(pi e AR). That is the actual test of the aerodynamic
# model, and it is what this produces.
#
# Budget: each alpha is a full mesh + solve. Expect 30-90 min per point, so a
# 6-point sweep is most of a day. Run it overnight.
#
# Usage:  bash cfd/sweep_alpha.sh 0 2 4 6 8 10
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/.." && pwd)"
VENV="$PROJECT/../../.venv-cad/Scripts/python.exe"
ALPHAS=("$@")
[ ${#ALPHAS[@]} -gt 0 ] || ALPHAS=(0 2 4 6 8)

OUT="$HERE/polar.csv"
echo "alpha_deg,CL,CD,LoverD" > "$OUT"

for A in "${ALPHAS[@]}"; do
    echo
    echo "################ alpha = $A deg ################"
    # Regenerate the case: alpha changes liftDir, dragDir and the inlet vector,
    # so it is NOT enough to edit U and re-solve.
    ( cd "$PROJECT/cad" && "$VENV" gen_cfd_case.py --alpha "$A" ) || exit 1
    bash "$HERE/run_case.sh" | tee "$HERE/log_alpha_${A}.txt"

    python3 - "$HERE/log_alpha_${A}.txt" "$A" "$OUT" <<'PY'
import sys, re
log, a, out = sys.argv[1], sys.argv[2], sys.argv[3]
txt = open(log).read()
def grab(tag):
    m = re.search(rf"^\s*{tag}\s+([+-]?[0-9.eE+-]+)", txt, re.M)
    return float(m.group(1)) if m else None
cl, cd = grab("CL"), grab("CD")
if cl is None or cd is None:
    print(f"  could not parse alpha={a} out of {log}")
else:
    with open(out, "a") as f:
        f.write(f"{a},{cl:.6f},{cd:.6f},{cl/cd if cd else 0:.4f}\n")
PY
done

echo
echo "=============================================================="
cat "$OUT"
echo
python3 - "$OUT" "$HERE/../sim/ros2/flight_params.json" <<'PY'
import sys, csv, json
rows = list(csv.DictReader(open(sys.argv[1])))
if len(rows) < 3:
    raise SystemExit("need at least 3 alphas to fit a polar")
cl = [float(r["CL"]) for r in rows]
cd = [float(r["CD"]) for r in rows]
# Least squares CD = CD0 + K CL^2.
x = [c * c for c in cl]
n = len(x)
sx, sy = sum(x), sum(cd)
sxx = sum(v * v for v in x)
sxy = sum(a * b for a, b in zip(x, cd))
den = n * sxx - sx * sx
K = (n * sxy - sx * sy) / den
CD0 = (sy - K * sx) / n
print(f"  fitted   CD = {CD0:.5f} + {K:.5f} CL^2")
ld_max = 0.5 / (CD0 * K) ** 0.5 if CD0 > 0 and K > 0 else float("nan")
print(f"  implies  (L/D)max = {ld_max:.2f}")
g = json.load(open(sys.argv[2]))["glide"]
print(f"  params.py derives (L/D)max = {g['l_over_d_max']:.2f}")
if ld_max == ld_max:
    err = 100 * (ld_max - g["l_over_d_max"]) / g["l_over_d_max"]
    print(f"  difference {err:+.1f} %")
    print()
    print("  Read this as a bracket, not a verdict: fully-turbulent RANS at")
    print("  Re 3e5 overpredicts CD0, and thin-airfoil theory underpredicts it.")
    print("  The truth is usually between them.")
PY
