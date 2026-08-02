#!/usr/bin/env bash
# Mesh and solve one angle of attack, then report CL, CD and L/D against the
# polar params.py derives.
#
# Usage:
#   bash cfd/run_case.sh              # whatever gen_cfd_case.py last wrote
#   PROCS=16 bash cfd/run_case.sh
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CASE="$HERE/case"
PROCS="${PROCS:-16}"

# Source OpenFOAM explicitly rather than relying on ~/.bashrc.
#
# The installer appends `source .../etc/bashrc` to ~/.bashrc, which works in a
# terminal and NOT in a script: Ubuntu's ~/.bashrc returns early for
# non-interactive shells, so the line is never reached. Running this script
# with `bash cfd/run_case.sh` from anything non-interactive then reports
# "OpenFOAM is not on PATH" on a machine where it is installed and working --
# the same class of trap as GALLIUM_DRIVER elsewhere in this project.
#
# 2412 first, deliberately. The `openfoam` metapackage currently pulls 2606,
# which is a RELEASE CANDIDATE; the dictionaries in this case are written for
# 2412 and reproducibility matters more than being current.
if ! command -v simpleFoam >/dev/null 2>&1; then
    for rc in /usr/lib/openfoam/openfoam2412/etc/bashrc \
              /usr/lib/openfoam/openfoam2406/etc/bashrc \
              /usr/lib/openfoam/openfoam2312/etc/bashrc; do
        [ -f "$rc" ] && { . "$rc" >/dev/null 2>&1; break; }
    done
fi
command -v simpleFoam >/dev/null 2>&1 || {
    echo "OpenFOAM is not on PATH. Run: bash cfd/install_openfoam.sh" >&2
    exit 1; }
echo "OpenFOAM: $(command -v simpleFoam)"
[ -f "$CASE/constant/triSurface/tri_tiltrotor.stl" ] || {
    echo "geometry missing -- run cad/gen_cfd_surface.py then cad/gen_cfd_case.py" >&2
    exit 1; }

cd "$CASE" || exit 1
LOG="$CASE/log"; mkdir -p "$LOG"

step() { echo; echo "=== $1 ==="; }

# Meshing is not idempotent: a leftover polyMesh from a previous alpha gets
# reused silently and you solve the old geometry at the new angle.
step "clean"
# postProcessing MUST go too. OpenFOAM does not overwrite an existing
# coefficient.dat -- it writes coefficient_<startTime>.dat alongside it. Leave
# the old one behind and the results step can read the PREVIOUS run's numbers,
# which is how a killed 117-iteration run got reported as the answer while the
# real 880-iteration run sat in the file next to it.
rm -rf processor* constant/polyMesh [1-9]* 0.* log/*.log postProcessing 2>/dev/null

step "blockMesh"
blockMesh > "$LOG/blockMesh.log" 2>&1 || { tail -25 "$LOG/blockMesh.log"; exit 1; }

step "surfaceFeatureExtract  (edges for snapping)"
# Try the modern name first, and keep the logs SEPARATE. Chaining both into one
# file meant the fallback's error overwrote the real one, so the reported
# failure was "cannot find surfaceFeatureExtractDict" when the actual cause was
# that `surfaceFeatures` does not exist in OpenFOAM 2412 at all. The error
# named the wrong problem entirely.
if command -v surfaceFeatureExtract >/dev/null 2>&1; then
    surfaceFeatureExtract > "$LOG/surfaceFeatureExtract.log" 2>&1 \
      || { tail -25 "$LOG/surfaceFeatureExtract.log"; exit 1; }
elif command -v surfaceFeatures >/dev/null 2>&1; then
    surfaceFeatures > "$LOG/surfaceFeatures.log" 2>&1 \
      || { tail -25 "$LOG/surfaceFeatures.log"; exit 1; }
else
    echo "neither surfaceFeatureExtract nor surfaceFeatures found" >&2; exit 1
fi

step "decomposePar"
decomposePar -force > "$LOG/decomposePar.log" 2>&1 \
  || { tail -25 "$LOG/decomposePar.log"; exit 1; }

step "snappyHexMesh on $PROCS ranks  (the long part: 20-60 min)"
mpirun -np "$PROCS" snappyHexMesh -overwrite -parallel \
    > "$LOG/snappyHexMesh.log" 2>&1 || { tail -40 "$LOG/snappyHexMesh.log"; exit 1; }
grep -E "^Layer mesh|Added|nCells" "$LOG/snappyHexMesh.log" | tail -5

step "re-decompose against the snapped mesh"
# NOT redundant. The first decomposePar ran on the background mesh, whose only
# patches were inlet/outlet/farfield, and it EXPANDS the "aircraft_.*" regex in
# 0/ into explicit per-patch entries for the patches that exist at that moment.
# snappyHexMesh then creates aircraft_fuselage, aircraft_wing and the rest --
# which have no entry, so simpleFoam dies with
#     Cannot find patchField entry for aircraft_fuselage
# after the mesh has already taken half an hour to build. Reconstructing and
# re-decomposing re-expands the regex against the final patch set.
reconstructParMesh -constant > "$LOG/reconstructMesh.log" 2>&1 \
  || { tail -20 "$LOG/reconstructMesh.log"; exit 1; }
rm -rf processor*
decomposePar -force > "$LOG/decomposePar2.log" 2>&1 \
  || { tail -20 "$LOG/decomposePar2.log"; exit 1; }

step "checkMesh"
mpirun -np "$PROCS" checkMesh -parallel > "$LOG/checkMesh.log" 2>&1
# A failed check is not automatically fatal -- a handful of skewed cells in the
# layer stack is normal -- but it must be seen, not buried in a log.
grep -E "\*\*\*|Mesh OK|cells:" "$LOG/checkMesh.log" | head -20

step "simpleFoam"
mpirun -np "$PROCS" simpleFoam -parallel > "$LOG/simpleFoam.log" 2>&1
tail -3 "$LOG/simpleFoam.log"

step "reconstruct"
reconstructPar -latestTime > "$LOG/reconstructPar.log" 2>&1

step "result"
# Newest by MODIFICATION TIME, not by name. `sort | tail -1` is locale
# dependent: under a UTF-8 locale it collates punctuation away, so
# "coefficient.dat" and "coefficient_0.dat" compare as "coefficientdat" vs
# "coefficient0dat" and the OLDER file wins. That silently reported a stale
# run's numbers. Sorting on mtime cannot be fooled by a filename.
COEF=$(find "$CASE/postProcessing/forceCoeffs" -name "coefficient*.dat" \
         -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
if [ -z "$COEF" ]; then
    echo "no force coefficients written -- check $LOG/simpleFoam.log"
    exit 1
fi
python3 - "$COEF" "$HERE/../sim/ros2/flight_params.json" <<'PY'
import sys, json, statistics
coef, fp = sys.argv[1], sys.argv[2]
hdr, rows = [], []
for line in open(coef):
    if line.startswith("#"):
        if "Time" in line:
            hdr = line.lstrip("#").split()
        continue
    p = line.split()
    if p:
        rows.append([float(x) for x in p])
if not rows:
    raise SystemExit("no rows in " + coef)

def col(name):
    return hdr.index(name) if name in hdr else None

# Average the last 15% of iterations: a converged SIMPLE run still wobbles a
# little, and the final single iteration is not the answer.
tail = rows[int(0.85 * len(rows)):] or rows[-1:]
ci, di = col("Cl"), col("Cd")
if ci is None or di is None:
    raise SystemExit(f"columns not found in header: {hdr}")
CL = statistics.fmean(r[ci] for r in tail)
CD = statistics.fmean(r[di] for r in tail)
spread = max(abs(r[di] - CD) for r in tail)

print(f"  iterations used   {len(rows)}  (averaged last {len(tail)})")
print(f"  CL                {CL:+8.4f}")
print(f"  CD                {CD:+8.5f}   (drift over the window {spread:.2e})")
if abs(CD) > 1e-9:
    print(f"  L/D               {CL/CD:+8.3f}")
g = json.load(open(fp))["glide"]
print()
print(f"  params.py polar:  (L/D)max {g['l_over_d_max']:.3f}, "
      f"L/D at cruise {g['l_over_d_at_cruise']:.3f}")
if spread > 0.05 * abs(CD):
    print("  WARNING: CD is still drifting >5% across the averaging window.")
    print("           Not converged -- raise --iterations and re-run.")
PY

echo
echo "logs in $LOG"
