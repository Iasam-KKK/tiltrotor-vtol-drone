r"""
Emit the aircraft's outer mould line as one STL for CFD.

WHY THIS IS NOT JUST "cat *.stl". Three things have to be right or the mesh is
wrong in ways that still produce a plausible-looking drag number:

1. INTERNAL PARTS MUST GO. booms, structure, linkages and formers are all
   inside the wing or fuselage envelope -- measured, not assumed (see
   EXTERNAL/INTERNAL below). Leaving them in buries closed surfaces inside the
   solid body, and snappyHexMesh spends its refinement budget resolving
   geometry that no air ever touches.

2. PARTS ARE NOT ALL IN AIRCRAFT COORDINATES. The wing, fuselage and tail sit
   at the origin, but the control surfaces, nacelles and props each carry a
   translation, a tilt rotation, sometimes a mirror, and the printed parts are
   authored in MILLIMETRES (scale 0.001). Concatenating the raw files puts the
   nacelles inside the fuselage at 1000x size.

3. THE PROPELLERS MUST GO. The point of this case is to measure the UNPOWERED
   drag polar, because that is the thing params.py derives and nothing has yet
   tested. A stationary prop disc in the mesh adds drag that the polar never
   claimed, and the comparison becomes meaningless.

Transforms come from render/assembly.json, which is generated from params.py,
so the CFD geometry cannot drift away from the simulated or printed aircraft.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_cfd_surface.py
    .\.venv-cad\Scripts\python.exe ...\gen_cfd_surface.py --pose hover
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import params as P

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent

# Wetted. Everything the external flow actually sees.
EXTERNAL = [
    "fuselage",
    "wing",
    "tail",
    "aileron_left", "aileron_right",
    "ruddervator_left", "ruddervator_right",
    "nacelle_yoke_left", "nacelle_yoke_right",
    "nacelle_cradle_left", "nacelle_cradle_right",
    "tail_motor_mount",
]

# Excluded, with the measurement that justifies it (aircraft coordinates, m):
#   booms      z[+0.012,+0.058] inside wing z[-0.010,+0.126]      -> internal
#   structure  z[-0.014,+0.074] inside the wing box               -> internal
#   linkages   z[-0.006,+0.073] inside the wing box               -> internal
#   formers    y[-0.052,+0.052] == fuselage half-width            -> internal
#   prop_*     rotating; excluded so this measures the UNPOWERED polar
INTERNAL = ["booms", "structure", "linkages", "formers"]
ROTATING = ["prop_left", "prop_right", "prop_tail"]


def read_stl(path: Path):
    """Return a list of (v0, v1, v2). Handles binary and ASCII STL."""
    raw = path.read_bytes()
    # An ASCII STL starts with "solid", but so can a binary one, so trust the
    # length arithmetic instead: binary is exactly 84 + 50*n bytes.
    if len(raw) >= 84:
        n = struct.unpack("<I", raw[80:84])[0]
        if len(raw) == 84 + 50 * n:
            tris = []
            for i in range(n):
                off = 84 + 50 * i
                d = struct.unpack("<12f", raw[off:off + 48])
                tris.append(tuple(tuple(d[3 + v * 3 + a] for a in range(3))
                                  for v in range(3)))
            return tris

    tris, cur = [], []
    for line in raw.decode("utf-8", "replace").splitlines():
        s = line.strip().split()
        if s and s[0] == "vertex":
            cur.append((float(s[1]), float(s[2]), float(s[3])))
            if len(cur) == 3:
                tris.append(tuple(cur))
                cur = []
    return tris


def euler_xyz(rx, ry, rz):
    """Blender's default XYZ Euler order: R = Rz @ Ry @ Rx."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy,     cy * sx,                cy * cx),
    )


def place(tris, loc, rot, scale, mirror):
    R = euler_xyz(*rot)
    out = []
    for tri in tris:
        v = []
        for (x, y, z) in tri:
            x, y, z = x * scale, y * scale * (-1.0 if mirror else 1.0), z * scale
            v.append((
                R[0][0] * x + R[0][1] * y + R[0][2] * z + loc[0],
                R[1][0] * x + R[1][1] * y + R[1][2] * z + loc[1],
                R[2][0] * x + R[2][1] * y + R[2][2] * z + loc[2],
            ))
        # Mirroring flips handedness, so the outward normal inverts. Reverse
        # the winding to put it back -- OpenFOAM uses the winding, and an
        # inside-out nacelle is a hole in the aircraft.
        if mirror:
            v = [v[0], v[2], v[1]]
        out.append(tuple(v))
    return out


def normal(tri):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    m = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / m, ny / m, nz / m


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="cruise",
                    choices=["hover", "transition", "cruise"])
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    P.check()

    manifest = json.loads(
        (PROJECT / "render" / "assembly.json").read_text(encoding="utf-8"))
    parts = manifest[args.pose]["parts"]
    by_name = {p["name"]: p for p in parts}

    missing = [n for n in EXTERNAL if n not in by_name]
    if missing:
        raise SystemExit(f"manifest has no part(s): {missing}")

    outdir = PROJECT / "cfd" / "geometry"
    outdir.mkdir(parents=True, exist_ok=True)
    out = Path(args.out) if args.out else outdir / "tri_tiltrotor.stl"

    lines = []
    total = 0
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for name in EXTERNAL:
        p = by_name[name]
        src = PROJECT / p["mesh"]
        if not src.exists():
            raise SystemExit(f"missing mesh {src} -- run gen_geometry.py first")
        tris = place(read_stl(src), p["loc"], p["rot"],
                     p.get("scale", 1.0), p.get("mirror", False))
        # One named solid per part. snappyHexMesh reads these as regions, so
        # refinement and the force breakdown can be set per component.
        lines.append(f"solid {name}")
        for t in tris:
            nx, ny, nz = normal(t)
            lines.append(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}")
            lines.append("    outer loop")
            for (x, y, z) in t:
                lines.append(f"      vertex {x:.6e} {y:.6e} {z:.6e}")
                for a, c in enumerate((x, y, z)):
                    lo[a] = min(lo[a], c)
                    hi[a] = max(hi[a], c)
            lines.append("    endloop")
            lines.append("  endfacet")
        lines.append(f"endsolid {name}")
        total += len(tris)
        print(f"  {name:22s} {len(tris):6d} tris")

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")

    d = P.solve()
    print()
    print(f"wrote {out}")
    print(f"  {total} triangles, {len(EXTERNAL)} named regions, pose={args.pose}")
    print(f"  excluded internal: {', '.join(INTERNAL)}")
    print(f"  excluded rotating: {', '.join(ROTATING)}")
    print(f"  bbox  x[{lo[0]:+.3f},{hi[0]:+.3f}] "
          f"y[{lo[1]:+.3f},{hi[1]:+.3f}] z[{lo[2]:+.3f},{hi[2]:+.3f}] m")
    print()
    print("reference values for system/forceCoeffs -- copy these, do not retype:")
    print(f"  magUInf   {P.V_CRUISE:.4f}      // m/s, cruise")
    print(f"  rhoInf    {P.RHO:.4f}      // kg/m^3")
    print(f"  lRef      {d.mac:.6f}    // MAC, m")
    print(f"  Aref      {d.wing_area:.6f}    // wing reference area, m^2")
    print(f"  CofR      (0 0 0)      // CG is the origin by construction")
    print()
    print("what the polar in params.py predicts, for comparison:")
    print(f"  CD0        {d.cd0:.5f}")
    print(f"  (L/D)max   {d.l_over_d_max:.3f}  at {d.v_best_glide:.2f} m/s")
    print(f"  L/D cruise {d.l_over_d_cruise:.3f}  at {P.V_CRUISE:.2f} m/s")
    print(f"  Re at cruise (MAC)  {P.V_CRUISE * d.mac / 1.5e-5:,.0f}")


if __name__ == "__main__":
    main()
