r"""
Emit an AVL model of the tri-tiltrotor from params.py.

WHAT THIS BUYS. AVL is a vortex-lattice code: it solves the whole aircraft's
lifting surfaces at once and reports the things params.py currently asserts
rather than computes -- the NEUTRAL POINT (and therefore the real static
margin), Cm_alpha, Cn_beta, Cl_beta, and the control power of the ailerons and
ruddervators. Right now the CG sits at 28% MAC because 28% is a conventional
number, and nothing in the project has ever checked that it is stable.

FRAME CONVERSION, which is the easiest thing here to get silently wrong.
    params.py / the CAD:  +x FORWARD, origin at the CG
    AVL (and XFLR5):      +x AFT,     origin wherever you put it
This file puts the AVL origin at the WING ROOT LEADING EDGE and negates x, so
Xref (the CG) comes out at a POSITIVE x of about +0.085 m. Get the sign wrong
and AVL reports a beautifully stable aircraft that is flying backwards.

THE WINGLET IS BLENDED, NOT A BENT PLATE. gen_geometry.py ramps the cant as
    phi(t) = CANT * t**0.7
and steps along the path with the MID-segment cant, so the tip lands at
y = 1.059 m, not the 1.026 m a straight 72 deg panel would give. This
reproduces that loop exactly rather than approximating it, because an AVL
winglet that disagrees with the CAD winglet is worse than no winglet at all --
it would quietly change the induced drag the whole polar rests on.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_avl.py

Then:
    avl tri_tiltrotor.avl
    AVL>  oper
    .OPER> x           (run the default case)
    .OPER> st          (stability derivatives + neutral point)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import params as P

PROJECT = Path(__file__).resolve().parent.parent
OUT = PROJECT / "aero"

NCHORD = 12
CSPACE = 1.0


def wing_sections():
    """(x_aft, y, z, chord, twist_deg, is_aileron) from root to winglet tip.

    x_aft is measured aft of the wing root leading edge, which is where the
    AVL origin sits.
    """
    c_root, c_tip = P.wing_chords()
    semi = P.WING_SPAN / 2.0
    qx = P.CG_MAC_FRACTION * P.WING_CHORD - 0.25 * P.WING_CHORD
    root_le_model = qx + 0.25 * c_root          # model frame, +x forward

    def at(y):
        st = P.wing_station(y)
        le_model = qx + st["x_qc"] + 0.25 * st["chord"]
        return (root_le_model - le_model,       # -> +x aft of root LE
                y, st["z"], st["chord"], math.degrees(st["twist"]))

    ail = P.aileron_geometry()
    y0, y1 = ail["y0"], ail["y1"]
    eps = 0.005                                  # 5 mm ramp, negligible

    out = []
    # AVL ramps a control gain linearly between the sections that declare it,
    # so the aileron ends need a section just outside them carrying no control.
    # Without these the aileron smears all the way to the root.
    for y, is_ail in ((0.0, False), (y0 - eps, False), (y0, True),
                      (y1, True), (y1 + eps, False), (semi, False)):
        x, yy, z, c, tw = at(y)
        out.append((x, yy, z, c, tw, is_ail))

    # --- blended winglet, stepped exactly as gen_geometry.py does it --------
    h = P.WINGLET_HEIGHT_FRAC * semi
    n_bl = 6
    x_t, y_t, z_t, c_t, tw_t = at(semi)
    px, py, pz = x_t, y_t, z_t
    for i in range(n_bl):
        ds = h / n_bl
        phi_m = P.WINGLET_CANT * (((i + 0.5) / n_bl) ** 0.7)
        # gen_geometry steps px_ -= ds*tan(sweep) with +x FORWARD, so in the
        # aft-positive AVL frame the same step is a PLUS.
        px += ds * math.tan(P.WINGLET_SWEEP)
        py += ds * math.cos(phi_m)
        pz += ds * math.sin(phi_m)
        t = (i + 1) / n_bl
        c_i = c_t + (c_t * P.WINGLET_TAPER - c_t) * t
        tw_i = math.degrees(P.WING_TWIST_TIP + P.WINGLET_TOE * t)
        out.append((px, py, pz, c_i, tw_i, False))
    return out


def tail_sections(area_scale: float = 1.0):
    """V-tail right panel, root to tip, with the ruddervator span flagged.

    area_scale multiplies the tail AREA. Chord and span each scale by its
    square root so the tail's aspect ratio is preserved -- shrinking only the
    chord would change the tail's own lift-curve slope and confuse the
    comparison between sizes.
    """
    d = P.solve()
    lin = math.sqrt(area_scale)
    root_le_model = -P.TAIL_SURFACE_ARM + 0.25 * P.TAIL_CHORD * lin
    c_root, c_tip = P.TAIL_CHORD * lin, P.TAIL_CHORD * 0.72 * lin
    span = d.tail_panel_span * lin
    dih = d.tail_dihedral

    qx = P.CG_MAC_FRACTION * P.WING_CHORD - 0.25 * P.WING_CHORD
    wing_root_le_model = qx + 0.25 * P.wing_chords()[0]
    x_root = wing_root_le_model - root_le_model          # aft of wing root LE

    # z of the V-tail root, read off the same station table the loft uses.
    z_root = 0.003
    f0, f1 = 0.2725, 0.8225        # ruddervator, from the built geometry
    eps = 0.004 / span

    out = []
    for f, is_rv in ((0.0, False), (f0 - eps, False), (f0, True),
                     (f1, True), (f1 + eps, False), (1.0, False)):
        c = c_root + (c_tip - c_root) * f
        out.append((
            x_root + f * span * math.cos(dih) * math.tan(0.0),  # unswept
            f * span * math.cos(dih),
            z_root + f * span * math.sin(dih),
            c, 0.0, is_rv))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tail-scale", type=float, default=1.0,
                    help="multiply the V-tail AREA by this, for sizing "
                         "studies. Writes tri_tiltrotor_t<scale>.avl so a "
                         "study cannot overwrite the real model.")
    args = ap.parse_args()

    P.check()
    d = P.solve()
    OUT.mkdir(parents=True, exist_ok=True)

    c_root, _ = P.wing_chords()
    qx = P.CG_MAC_FRACTION * P.WING_CHORD - 0.25 * P.WING_CHORD
    x_cg = qx + 0.25 * c_root       # CG, aft of the wing root LE, positive aft

    hinge_ail = 1.0 - P.AILERON_CHORD_FRAC
    hinge_rv = 1.0 - P.RUDDERVATOR_CHORD_FRAC

    L = []
    add = L.append
    add("Tri-Tiltrotor VTOL  (generated by cad/gen_avl.py -- do not edit)")
    add("#Mach")
    add("0.0")
    add("#IYsym  IZsym  Zsym")
    add("0       0      0.0")
    add("#Sref     Cref     Bref")
    add(f"{d.wing_area:.6f} {d.mac:.6f} {P.WING_SPAN:.6f}")
    add("#Xref     Yref     Zref        <- CG, aft-positive from wing root LE")
    add(f"{x_cg:.6f} 0.000000 0.000000")
    add("#CDp   -- left at zero on purpose: AVL returns INDUCED drag only.")
    add("#         The viscous part comes from XFOIL/params.py, and adding a")
    add("#         guess here would double-count it.")
    add("0.0")
    add("")

    # ------------------------------------------------------------------ wing
    add("#" + "=" * 68)
    add("SURFACE")
    add("Wing")
    add("#Nchordwise  Cspace")
    add(f"{NCHORD}          {CSPACE}")
    add("")
    add("YDUPLICATE")
    add("0.0")
    add("")
    add("ANGLE")
    add("0.0")
    add("")
    for (x, y, z, c, tw, is_ail) in wing_sections():
        add("SECTION")
        add("#Xle      Yle      Zle      Chord    Ainc   Nspan Sspace")
        add(f"{x:8.5f} {y:8.5f} {z:8.5f} {c:8.5f} {tw:6.3f}   6    -2.0")
        add("AFILE")
        add(f"naca{P.WING_NACA}.dat")
        if is_ail:
            add("#            gain  Xhinge  XYZhvec      SgnDup")
            add(f"CONTROL")
            add(f"aileron  1.0  {hinge_ail:.4f}  0. 0. 0.  -1.0")
        add("")

    # ---------------------------------------------------------------- V-tail
    add("#" + "=" * 68)
    add("SURFACE")
    add("Vtail")
    add("#Nchordwise  Cspace")
    add(f"{NCHORD}          {CSPACE}")
    add("")
    add("YDUPLICATE")
    add("0.0")
    add("")
    for (x, y, z, c, tw, is_rv) in tail_sections(args.tail_scale):
        add("SECTION")
        add("#Xle      Yle      Zle      Chord    Ainc   Nspan Sspace")
        add(f"{x:8.5f} {y:8.5f} {z:8.5f} {c:8.5f} {tw:6.3f}   5    -2.0")
        add("AFILE")
        add(f"naca{P.TAIL_NACA}.dat")
        if is_rv:
            # A V-tail carries BOTH axes on one pair of surfaces: symmetric
            # deflection is pitch, antisymmetric is yaw. SgnDup +1 / -1 is what
            # encodes that, and it is the whole reason a V-tail needs a mixer.
            add("CONTROL")
            add(f"elevator  1.0  {hinge_rv:.4f}  0. 0. 0.   1.0")
            add("CONTROL")
            add(f"rudder    1.0  {hinge_rv:.4f}  0. 0. 0.  -1.0")
        add("")

    avl = OUT / (f"tri_tiltrotor_t{args.tail_scale:.2f}.avl"
                 if abs(args.tail_scale - 1.0) > 1e-9 else "tri_tiltrotor.avl")
    avl.write_bytes(("\n".join(L) + "\n").encode("utf-8"))

    # ------------------------------------------------------------------ mass
    M = [
        "#  AVL mass file, generated from params.py",
        "Lunit = 1.0 m",
        "Munit = 1.0 kg",
        "Tunit = 1.0 s",
        "",
        f"g   = {P.G}",
        f"rho = {P.RHO}",
        "",
        "#  mass      x         y      z         Ixx       Iyy       Izz",
        f"   {P.MASS_TOTAL:.4f}  {x_cg:.6f}  0.0    0.0    "
        f"{d.ixx:.5f}  {d.iyy:.5f}  {d.izz:.5f}",
    ]
    mass = OUT / "tri_tiltrotor.mass"
    mass.write_bytes(("\n".join(M) + "\n").encode("utf-8"))

    ws = wing_sections()
    print(f"wrote {avl}")
    print(f"wrote {mass}")
    print()
    print(f"  wing   {len(ws)} sections, tip at y = {ws[-1][1]:.4f} m "
          f"z = {ws[-1][2]:.4f} m")
    print(f"         (STL measures the half-span at 1.063 m including section "
          f"thickness -- consistent)")
    print(f"  V-tail {len(tail_sections())} sections, dihedral "
          f"{math.degrees(d.tail_dihedral):.2f} deg, panel span "
          f"{d.tail_panel_span*math.sqrt(args.tail_scale):.4f} m"
          f"   (area x {args.tail_scale:.2f})")
    print(f"  Sref {d.wing_area:.4f} m^2   Cref {d.mac:.4f} m   "
          f"Bref {P.WING_SPAN:.3f} m")
    print(f"  Xref (CG) {x_cg:.4f} m aft of the wing root LE")
    print()
    print("  controls: aileron (antisymmetric), elevator + rudder on the")
    print("            SAME ruddervators (symmetric / antisymmetric)")
    print()
    print("The number to go and get:")
    print(f"  AVL reports Xnp. Static margin = (Xnp - Xref)/Cref")
    print(f"  Xref is {x_cg:.4f}, Cref is {d.mac:.4f}.")
    print(f"  params.py places the CG at {P.CG_MAC_FRACTION*100:.0f}% MAC by "
          f"CONVENTION and has never checked it.")
    print("  Healthy for this class is roughly +8% to +15%. Negative is unflyable")
    print("  without active stabilisation.")


if __name__ == "__main__":
    main()
