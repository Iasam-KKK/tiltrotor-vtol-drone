# CFD — measuring the drag polar

`params.py` **derives** a drag polar from thin-airfoil theory and the planform.
Nothing has ever **measured** it. `sim/verify_glide.sh`, which was supposed to,
does not work (see the project README). This directory is the attempt to close
that gap.

```bash
bash cfd/install_openfoam.sh          # once; then open a NEW shell
.venv-cad/Scripts/python.exe cad/gen_cfd_surface.py
.venv-cad/Scripts/python.exe cad/gen_cfd_case.py --alpha 0
bash cfd/run_case.sh                  # ~30-90 min
bash cfd/sweep_alpha.sh 0 2 4 6 8     # overnight; produces polar.csv
```

Everything under `case/` and `geometry/` is **generated** and git-ignored. The
two generators in `cad/` are the source, so the CFD geometry cannot drift away
from the flying model or the printed parts.

## Read this before believing a number

**The Reynolds number is the problem.** 329,000 at cruise, 223,000 at loiter,
186,000 at stall, all on the MAC. That is low-Re transitional flow. The real
boundary layer runs laminar over a meaningful fraction of the chord and often
separates into a bubble before reattaching turbulent.

A fully-turbulent k-ω SST run assumes turbulence from the leading edge and will
**overpredict skin-friction drag**, sometimes by tens of percent at this scale.
The case therefore defaults to **k-kL-ω**, which is transition-sensitive. That
is an improvement, not a solution: transition models at Re 2×10⁵ are themselves
sensitive to the inlet turbulence intensity, which here is a guess (0.1%).

So: **CFD is a bracket, not an oracle.** Thin-airfoil theory underpredicts CD0;
fully-turbulent RANS overpredicts it. Expect the truth between them, and say so
in any listing rather than quoting a CFD number as measurement.

**Cheaper tools answer most of this faster.** XFOIL gives 2-D section polars at
the exact Reynolds number in seconds, and AVL gives span loading, induced drag
and the stability derivatives this project still lacks, in minutes. Run those
first. CFD earns its cost on things they cannot do: fuselage interference, the
waisted pylon, nacelle drag, and prop-wing interaction.

## What the case is

| | |
|---|---|
| Solver | `simpleFoam`, steady incompressible RANS |
| Turbulence | `kkLOmega` (default) or `kOmegaSST` |
| Geometry | unpowered airframe — **props excluded on purpose** |
| Domain | 5 spans out, 10 spans downstream, `slip` farfield |
| Reference | Aref 0.52 m², lRef 0.26 m, CofR at the CG (the origin) |

**Props are excluded deliberately.** The polar being tested is the unpowered
one. A stationary disc in the mesh adds drag the polar never claimed.

**Internal parts are excluded**, measured rather than assumed: `booms`,
`structure`, `linkages` and `formers` all sit inside the wing or fuselage
envelope. See the table in `cad/gen_cfd_surface.py`.

## Things that will bite

- **`locationInMesh` must be outside every solid.** It is at `(3, 2, 2)`, ahead
  of and above the nose. A point inside the fuselage meshes the *interior* and
  produces a beautiful, meaningless result.
- **Meshing is not idempotent.** `run_case.sh` deletes `constant/polyMesh` and
  `processor*` first; without that you silently solve the previous alpha's mesh.
- **Changing alpha means regenerating the case**, not editing `U`. The angle
  also sets `liftDir` and `dragDir`; edit only the inlet and your CL and CD are
  resolved onto the wrong axes.
- **WSL memory.** It defaults to half the host — 15 GB of your 32 GB. snappy
  wants roughly 1 GB per million cells. Set `memory=24GB` in `%USERPROFILE%\.wslconfig`
  and `wsl --shutdown` before meshing.
- **Check y+ after the first run.** The case uses low-Re wall treatment with 8
  layers; if y+ is well above 1 the layers did not resolve and the drag is
  wrong. `simpleFoam -postProcess -func yPlus -latestTime`.
- **One alpha is not a polar.** CD0 and the induced factor only separate when
  you fit CD against CL² across several angles. That is what `sweep_alpha.sh`
  does, and its intercept is the number to compare.
