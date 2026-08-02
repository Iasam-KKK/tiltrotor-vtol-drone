#!/usr/bin/env python3
"""Is the ANSWER converged, or just the residuals?

Residual convergence and force convergence are different things, and it is the
forces that the result rests on. Forces usually settle well before residuals
reach their targets -- so a run can be stopped early and still be right -- but
residuals can also plateau while CD is still drifting, which looks converged
and is not.

This reports the trend in CL and CD directly, which is the quantity of
interest. Run it any time, including mid-solve.

Usage:  python3 cfd/watch_forces.py
"""
from __future__ import annotations

import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
pat = os.path.join(HERE, "case", "postProcessing", "forceCoeffs", "*",
                   "coefficient*.dat")
files = sorted(glob.glob(pat))
if not files:
    sys.exit("no coefficient file yet -- has simpleFoam started?")

hdr, rows = [], []
for line in open(files[-1]):
    if line.startswith("#"):
        if "Time" in line:
            hdr = line.lstrip("#").split()
        continue
    p = line.split()
    if p:
        rows.append([float(x) for x in p])
if len(rows) < 10:
    sys.exit(f"only {len(rows)} rows so far")

ci, di = hdr.index("Cl"), hdr.index("Cd")
print(f"file: {os.path.relpath(files[-1], HERE)}")
print(f"iterations written: {len(rows)}")
print()
print(f"{'iter':>7} {'CL':>9} {'CD':>10}")
for frac in (0.2, 0.4, 0.6, 0.8, 0.9, 1.0):
    i = min(int(frac * len(rows)) - 1, len(rows) - 1)
    print(f"{int(rows[i][0]):7d} {rows[i][ci]:9.4f} {rows[i][di]:10.5f}")

tail = rows[int(0.9 * len(rows)):]
cd = [r[di] for r in tail]
cl = [r[ci] for r in tail]
mean_cd = sum(cd) / len(cd)
mean_cl = sum(cl) / len(cl)
print()
print(f"last 10% of iterations ({len(tail)} samples):")
print(f"  CD {mean_cd:.5f}  spread {max(cd) - min(cd):.2e} "
      f"= {(max(cd) - min(cd)) / abs(mean_cd) * 100:.2f}% of mean")
print(f"  CL {mean_cl:.4f}   spread {max(cl) - min(cl):.2e}")
print(f"  L/D {mean_cl / mean_cd:.3f}")
print()
if (max(cd) - min(cd)) / abs(mean_cd) < 0.005:
    print("  CD is settled to better than 0.5%. Further iterations will not")
    print("  change the answer -- residuals may still be falling, but the")
    print("  quantity you care about has stopped moving.")
else:
    print("  CD still moving. Not converged yet.")
