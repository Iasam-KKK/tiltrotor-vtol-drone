r"""
Where, in model z, is the wing skin at the boom station?

The boom is currently placed at `z_wing - 0.014`, i.e. 14 mm BELOW the chord
line, which leaves it hanging in the airstream under the wing. To bury it we
need the section's actual z extent at that station -- and that depends on how
section_points()'s 2-D Y maps into model z through the loft plane
    Plane(origin=..., x_dir=(1,0,0), z_dir=(0,1,0))
whose normal is +y. Whether sketch-Y becomes +z or -z decides whether NACA
2410's camber sits above or below the chord line in the model. Guessing that
sign would bury the boom out through the wrong surface, so: measure.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\probe_wing_section.py
"""

from __future__ import annotations

import math

import params as P
import gen_geometry as G


def main() -> None:
    c_root, c_tip = G.chords()
    semi = P.WING_SPAN / 2.0
    y = P.WING_ROTOR_Y
    frac = y / semi
    c_local = c_root + (c_tip - c_root) * frac
    z_wing = y * math.tan(P.WING_DIHEDRAL)

    print(f"chord root/tip      {c_root * 1000:.1f} / {c_tip * 1000:.1f} mm")
    print(f"boom station y      {y:.3f} m  (frac {frac:.3f})")
    print(f"local chord         {c_local * 1000:.1f} mm")
    print(f"dihedral rise       {z_wing * 1000:.1f} mm")

    # --- 2-D section, as the sketch sees it --------------------------------
    pts = G.section_points(P.WING_NACA, c_local, 0.0)
    ys = [p[1] for p in pts]
    print(f"\nsection 2-D Y range {min(ys) * 1000:+.1f} .. {max(ys) * 1000:+.1f} mm"
          f"   (thickness {(max(ys) - min(ys)) * 1000:.1f} mm)")

    # --- 3-D truth: build the wing and measure it --------------------------
    print("\nbuilding wing to measure the real z extent...")
    wing = G.build_wing()
    bb = wing.bounding_box()
    print(f"wing bbox x  {bb.min.X * 1000:+8.1f} .. {bb.max.X * 1000:+8.1f} mm")
    print(f"wing bbox y  {bb.min.Y * 1000:+8.1f} .. {bb.max.Y * 1000:+8.1f} mm")
    print(f"wing bbox z  {bb.min.Z * 1000:+8.1f} .. {bb.max.Z * 1000:+8.1f} mm")

    # A cambered section has MORE material above the chord line than below.
    # If sketch-Y maps to +z that shows up as |z_max| > |z_min| at the root,
    # before dihedral lifts the tips.
    print("\n--- interpretation ---")
    if abs(bb.max.Z) > abs(bb.min.Z):
        print("  z_max dominates -> sketch Y maps to +z, camber is ABOVE the")
        print("  chord line, as a normal aerofoil. Bury the boom at +yc.")
    else:
        print("  z_min dominates -> sketch Y maps to -z: the section is")
        print("  INVERTED in the model. Bury the boom at -yc.")

    # Where the boom must sit to be fully inside the skin at 30% chord.
    m, p_, t = 0.02, 0.4, 0.10
    x30 = 0.30
    yt = 5 * t * (0.2969 * math.sqrt(x30) - 0.1260 * x30 - 0.3516 * x30 ** 2
                  + 0.2843 * x30 ** 3 - 0.1036 * x30 ** 4)
    yc = m / p_ ** 2 * (2 * p_ * x30 - x30 ** 2)
    print(f"\nat 30% chord: camber {yc * c_local * 1000:+.1f} mm, "
          f"half-thickness {yt * c_local * 1000:.1f} mm")
    print(f"  upper skin {(yc + yt) * c_local * 1000:+.1f} mm, "
          f"lower skin {(yc - yt) * c_local * 1000:+.1f} mm")
    r = P.BOOM_DIA_MM / 2000.0
    print(f"  boom radius {r * 1000:.1f} mm -> clearance each side "
          f"{(yt * c_local - r) * 1000:.1f} mm")
    print(f"\n  CURRENT boom centre  {(z_wing - 0.014) * 1000:+.1f} mm "
          f"(relative to local chord line: -14.0 mm)")
    print(f"  BURIED boom centre   {(z_wing + yc * c_local) * 1000:+.1f} mm "
          f"(relative: {yc * c_local * 1000:+.1f} mm)")


if __name__ == "__main__":
    main()
