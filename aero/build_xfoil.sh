#!/usr/bin/env bash
# Build XFOIL 6.99 from MIT source. No sudo needed (gfortran + libx11-dev only).
#
# THE TRAP THAT COSTS AN EVENING. XFOIL's bin/Makefile links against
# ../plotlib/libPlt_gDP.a -- the DOUBLE precision plot library -- so XFOIL
# itself must also be compiled with -fdefault-real-8. Build it without, and it
# compiles and links cleanly, runs, prints its banner, and then every single
# viscous solution comes back NaN:
#     TRCHEK2: N2 convergence failed
#     STFIND: Stagnation point not found
#     VISCAL: Convergence failed
# There is no error about precision anywhere. It looks exactly like an airfoil
# that will not converge at low Reynolds number, which is a thing that really
# happens, so it is very easy to blame the aerofoil instead of the build.
#
# Usage:  bash aero/build_xfoil.sh
set -euo pipefail

SRC="$HOME/xfoil_build"
mkdir -p "$SRC" && cd "$SRC"

if [ ! -d Xfoil ]; then
    curl -fsSL -o xfoil.tgz https://web.mit.edu/drela/Public/web/xfoil/xfoil6.99.tgz
    tar xzf xfoil.tgz
fi

echo "[1/3] plot library (double precision, to match the solver)"
cd "$SRC/Xfoil/plotlib"
cp -f config.make.gfortranDP config.make
make > /tmp/xfoil_plotlib.log 2>&1
ls libPlt_gDP.a

echo "[2/3] patching bin/Makefile so FFLAGS carries -fdefault-real-8"
cd "$SRC/Xfoil/bin"
python3 - <<'PY'
import re, pathlib
p = pathlib.Path("Makefile")
s = p.read_text()
# Only touch the active gfortran block; leave every commented variant alone.
s = re.sub(r"(?m)^FFLAGS = -O\s*$",
           "FFLAGS = -O2 $(DBL) -fallow-argument-mismatch -std=legacy", s)
s = re.sub(r"(?m)^FFLAGS = -O2(?!.*DBL).*$",
           "FFLAGS = -O2 $(DBL) -fallow-argument-mismatch -std=legacy", s)
# FFLOPT defaults to including $(CHK), which turns on -finit-real=inf and
# -ffpe-trap. Useful for debugging XFOIL itself, poison for using it.
s = re.sub(r"(?m)^FFLOPT = .*$",
           "FFLOPT = -O2 $(DBL) -fallow-argument-mismatch -std=legacy", s)
p.write_text(s)
for line in s.splitlines():
    if line.startswith(("FC =", "FFLAGS", "FFLOPT", "PLTOBJ")):
        print("   ", line)
PY

echo "[3/3] building"
make clean > /dev/null 2>&1 || true
# The final `install` step targets a hardcoded /home/codes/bin and fails.
# The binary is already built by then, so that failure is not fatal.
make xfoil > /tmp/xfoil_build.log 2>&1 || true
[ -x xfoil ] || { echo "build failed:"; tail -20 /tmp/xfoil_build.log; exit 1; }

mkdir -p "$HOME/.local/bin"
cp -f xfoil "$HOME/.local/bin/xfoil"
echo
echo "installed -> $HOME/.local/bin/xfoil"

echo
echo "smoke test: NACA 2410 at Re 223,398, Ncrit 5, headless"
cd /tmp && rm -f _smoke.txt
# `plop` then `G` turns the X11 graphics OFF. Without it XFOIL opens a plot
# window, which under WSLg composites solid black -- and there is then no
# reason to move to the xrdp session for batch work.
printf 'plop\nG\n\nnaca 2410\npane\noper\nvisc 223398\nvpar\nn 5\n\niter 200\npacc\n_smoke.txt\n\naseq 0 6 2\n\nquit\n' \
  | "$HOME/.local/bin/xfoil" > /dev/null 2>&1
if [ "$(grep -cE '^ +[0-9-]' _smoke.txt 2>/dev/null || echo 0)" -ge 2 ]; then
    echo "  PASS -- converged points:"
    grep -E '^ +[0-9-]' _smoke.txt
else
    echo "  FAIL -- still NaN. Check that FFLAGS really contains -fdefault-real-8."
    exit 1
fi
