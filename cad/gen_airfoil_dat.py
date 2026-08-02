r"""
Emit the aircraft's sections as airfoil coordinate files for XFOIL / XFLR5.

WHY THIS EXISTS. XFOIL, XFLR5 and AVL do not read STL, STEP or any mesh -- they
read a two-column list of x/c, y/c running from the trailing edge over the top,
round the nose, and back along the bottom (Selig format). Nothing in this repo
emitted that, so the only way to analyse the section was to retype "NACA 2410"
into a web tool and hope it matched what the wing was actually lofted from.

These come from the SAME naca4() that gen_geometry.py lofts the wing with, so
what XFOIL analyses is what the CAD built and the simulator flies.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_airfoil_dat.py
"""

from __future__ import annotations

from pathlib import Path

import params as P
from gen_geometry import naca4

PROJECT = Path(__file__).resolve().parent.parent
OUT = PROJECT / "aero"


def write_dat(code: str, name: str, n: int = 80) -> Path:
    # n=80 gives 160 points. Not arbitrary: AVL's READBL refuses more than its
    # compiled-in IBX limit and dies with "Too many airfoil points" at 320,
    # while XFOIL repanels to ~160 by default anyway. 160 serves both, so one
    # file works everywhere instead of needing a coarse and a fine variant.
    pts = naca4(code, n=n)
    path = OUT / f"{name}.dat"
    lines = [name]
    for x, y in pts:
        lines.append(f"  {x:.6f}  {y:.6f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    P.check()
    OUT.mkdir(parents=True, exist_ok=True)
    d = P.solve()
    nu = 1.5e-5

    written = []
    written.append((write_dat(P.WING_NACA, f"naca{P.WING_NACA}"), "wing"))
    written.append((write_dat(P.TAIL_NACA, f"naca{P.TAIL_NACA}"), "V-tail"))

    for path, what in written:
        pts = path.read_text(encoding="utf-8").strip().splitlines()
        print(f"  {path.name:16s} {len(pts) - 1:4d} points   ({what})")

    c_root, c_tip = P.wing_chords()
    print()
    print(f"wrote to {OUT}")
    print()
    print("Reynolds numbers to run these at -- the analysis is worthless at the")
    print("wrong Re, and this aircraft lives in the transitional range:")
    for label, v in (("cruise", P.V_CRUISE),
                     ("loiter (1.2 Vs)", 1.2 * d.v_stall),
                     ("stall", d.v_stall)):
        print(f"  {label:16s} {v:5.2f} m/s   "
              f"Re(MAC) {v * d.mac / nu:9,.0f}   "
              f"Re(tip) {v * c_tip / nu:9,.0f}")
    print()
    print(f"  root chord {c_root:.4f} m, tip chord {c_tip:.4f} m, MAC {d.mac:.4f} m")
    print()
    print("The TIP Reynolds number is the one that decides whether the tip")
    print("stalls first. It is the lowest number in the table, and the -2 deg")
    print("washout exists to keep that from mattering.")


if __name__ == "__main__":
    main()
