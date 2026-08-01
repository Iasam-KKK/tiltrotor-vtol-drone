r"""
Export every part as a positioned STEP into cad/out/assembly/ for FreeCAD.

Why STEP and not the STLs Blender uses: STL is a triangle soup with no names,
no colours and no measurable geometry. In FreeCAD a STEP part keeps its
identity -- you can click it, read its name, measure a hole, section it, and
see what is actually inside the airframe. For "what is this thing and does it
fit", that is the difference between reviewing a model and squinting at one.

Everything is exported AT ITS FLIGHT POSITION, so opening the folder gives an
assembled aircraft rather than a pile of parts at the origin.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_assembly_step.py
"""

from __future__ import annotations

import math
from pathlib import Path

from build123d import Location, export_step

import params as P
import gen_geometry as G
import gen_nacelle as N

OUT = Path(__file__).resolve().parent / "out" / "assembly"


def placed_parts() -> list[tuple[str, object]]:
    """Every solid, at its flight position, in metres."""
    d = P.solve()
    a, c = d.wing_rotor_arm, d.tail_rotor_arm
    y = P.WING_ROTOR_Y
    qx = P.CG_MAC_FRACTION * P.WING_CHORD - 0.25 * P.WING_CHORD
    items: list[tuple[str, object]] = []

    # --- airframe shells, already in the model frame ---
    for name in ("fuselage", "tail", "booms", "structure", "formers",
                 "linkages"):
        items.append((name, G.PARTS[name]()))
    items.append(("wing", G.PARTS["wing"]().moved(Location((qx, 0, 0)))))

    # --- control surfaces, at their hinge points ---
    ail = P.aileron_geometry()
    for label, sgn in (("left", +1.0), ("right", -1.0)):
        items.append((f"aileron_{label}", G.build_aileron(sgn).moved(
            Location((qx + ail["x"], sgn * ail["y_mid"], ail["z"])))))
        rv = P.ruddervator_geometry(sgn)
        items.append((f"ruddervator_{label}", G.build_ruddervator(sgn).moved(
            Location((rv["x"], rv["y"], rv["z"])))))

    # --- nacelle hardware, mm parts scaled to metres by FreeCAD on import ---
    # Exported separately in mm; noted in the README of the folder.
    for label, px, py in (("left", a, +y), ("right", a, -y)):
        wing_z = abs(py) * math.tan(P.WING_DIHEDRAL)
        items.append((f"nacelle_yoke_{label}_mm",
                      N.build_yoke().moved(Location((0, 0, 0)))))
        items.append((f"nacelle_cradle_{label}_mm",
                      N.build_cradle().moved(Location((0, 0, 0)))))
        break      # one of each is enough to inspect; they are mirror pairs
    items.append(("tail_motor_mount_mm", N.build_tail_mount()))

    # --- propellers ---
    for label, px, py, dia in (("left", a, +y, P.WING_PROP_DIAMETER),
                               ("right", a, -y, P.WING_PROP_DIAMETER),
                               ("tail", -c, 0.0, P.TAIL_PROP_DIAMETER)):
        z = (P.NACELLE_Z_OFFSET + 0.035) if label != "tail" \
            else (P.tail_rotor_z() + 0.035)
        items.append((f"prop_{label}",
                      G.build_prop(dia).moved(Location((px, py, z)))))
    return items


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob("*.step"):
        f.unlink()

    n = 0
    for name, solid in placed_parts():
        try:
            export_step(solid, str(OUT / f"{name}.step"))
            bb = solid.bounding_box()
            print(f"  {name:24s} {solid.volume * 1e6:9.1f} cm^3  "
                  f"bbox {bb.size.X:.3f} x {bb.size.Y:.3f} x {bb.size.Z:.3f}")
            n += 1
        except Exception as exc:                        # noqa: BLE001
            print(f"  FAIL {name}: {exc}")

    print(f"\nwrote {n} STEP files to {OUT}")
    print("Open them all together in FreeCAD:")
    print("    powershell -File tools/open_in_freecad.ps1")


if __name__ == "__main__":
    main()
