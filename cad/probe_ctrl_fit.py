r"""
Does the aileron cutter actually sit inside the wing?

Symptom: build_wing() subtracts the two inflated ailerons, but the wing volume
only dropped 57.5 cm^3 while the two aileron solids are 478 cm^3 together. If
the cut were landing correctly the wing should lose roughly the aileron volume.
So the surfaces are probably NOT where the wing is -- which would also explain
them rendering as panels sitting on the wing.

Compare bounding boxes; they are enough to catch a misplacement of this size.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\probe_ctrl_fit.py
"""

from __future__ import annotations

import params as P
import gen_geometry as G


def bb(label, solid):
    b = solid.bounding_box()
    print(f"  {label:22s} x {b.min.X:+.4f}..{b.max.X:+.4f}  "
          f"y {b.min.Y:+.4f}..{b.max.Y:+.4f}  z {b.min.Z:+.4f}..{b.max.Z:+.4f}")
    return b


def main() -> None:
    a = P.aileron_geometry()
    print("aileron_geometry() (wing frame):")
    for k in ("y0", "y1", "y_mid", "span", "hinge", "x", "z"):
        print(f"  {k:8s} {a[k]:+.5f}")

    print("\nwing station at y_mid:")
    st = P.wing_station(a["y_mid"])
    for k, v in st.items():
        print(f"  {k:8s} {v:+.5f}")

    print("\nbounding boxes, wing frame:")
    wing_uncut = G.build_wing.__wrapped__() if hasattr(G.build_wing, "__wrapped__") \
        else None
    ail_o = G.build_aileron(+1.0, at_origin=True)
    ail_p = G.build_aileron(+1.0, gap=0.0015, at_origin=False)
    bb("aileron at origin", ail_o)
    bb("aileron positioned", ail_p)

    wing = G.build_wing()
    wb = bb("wing (already cut)", wing)

    ab = ail_p.bounding_box()
    print("\noverlap of positioned aileron with wing bbox:")
    for ax, lo_w, hi_w, lo_a, hi_a in (
        ("x", wb.min.X, wb.max.X, ab.min.X, ab.max.X),
        ("y", wb.min.Y, wb.max.Y, ab.min.Y, ab.max.Y),
        ("z", wb.min.Z, wb.max.Z, ab.min.Z, ab.max.Z),
    ):
        lo, hi = max(lo_w, lo_a), min(hi_w, hi_a)
        span = hi - lo
        print(f"  {ax}: aileron {lo_a:+.4f}..{hi_a:+.4f} vs wing "
              f"{lo_w:+.4f}..{hi_w:+.4f} -> overlap {span:+.4f}"
              f"{'  <-- NO OVERLAP' if span <= 0 else ''}")

    print(f"\naileron volume {ail_p.volume * 1e6:.1f} cm^3")

    # The decisive measurement: how much of the cutter is actually inside the
    # uncut wing? Bounding boxes overlapping proves nothing about the solids.
    raw = G.build_wing(cut=False)
    inter = raw & ail_p
    cut = raw - ail_p
    print(f"\nuncut wing volume     {raw.volume * 1e6:9.1f} cm^3")
    print(f"intersection w/ cutter{inter.volume * 1e6:9.1f} cm^3")
    print(f"wing after one cut    {cut.volume * 1e6:9.1f} cm^3")
    print(f"removed               {(raw.volume - cut.volume) * 1e6:9.1f} cm^3")
    frac = inter.volume / ail_p.volume if ail_p.volume else 0.0
    print(f"\ncutter inside the wing: {frac * 100:.1f}%")
    if frac < 0.8:
        print("  -> the surface is mostly OUTSIDE the wing: it would render as")
        print("     a slab attached to the skin, which is the reported symptom.")

    # --- which way does sketch Y map into model z? --------------------------
    # probe_wing_section.py answered this from the WING BOUNDING BOX, which is
    # dominated by the winglet rising to +152 mm -- so its "camber is above the
    # chord line" conclusion was unfounded. Isolate the mapping by extruding a
    # single root section through the SAME plane construction build_wing uses,
    # with no winglet, no dihedral and no twist to confuse it.
    from build123d import BuildPart, BuildSketch, BuildLine, Plane, Polyline, \
        make_face, extrude

    c_root, _ = P.wing_chords()
    with BuildPart() as slab:
        plane = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
        with BuildSketch(plane):
            with BuildLine():
                Polyline(*G.section_points(P.WING_NACA, c_root, 0.0), close=True)
            make_face()
        extrude(amount=0.02)
    sb = slab.part.bounding_box()
    pts = G.section_points(P.WING_NACA, c_root, 0.0)
    y2d = [p[1] for p in pts]
    print("\n--- section orientation, isolated ---")
    print(f"  sketch Y range   {min(y2d) * 1000:+.1f} .. {max(y2d) * 1000:+.1f} mm")
    print(f"  model  z range   {sb.min.Z * 1000:+.1f} .. {sb.max.Z * 1000:+.1f} mm")
    # A cambered section has MORE material above its chord line than below, so
    # in a correctly-oriented model |z_max| > |z_min|. Testing the extents
    # directly is the honest criterion; comparing against the sketch's own Y is
    # not, because the plane's Y flip is exactly what is being measured.
    if abs(sb.max.Z) > abs(sb.min.Z):
        print("  => camber is ABOVE the chord line. Section is right way up.")
    else:
        print("  => camber points DOWN. The section is INVERTED in the model,")
        print("     and every camber offset used for the boom burial and the")
        print("     control-surface hinges has the WRONG SIGN.")


if __name__ == "__main__":
    main()
