r"""
Emit render/assembly.json — where every part sits, straight from params.py.

Blender must not carry its own copy of the geometry numbers. If it did, a
change to params.py would move the aircraft in the simulator and leave the
renders showing the old one, and nobody would notice until a client compared
the drawing to the picture.

Units: the airframe meshes are in METRES, the nacelle mechanism meshes are in
MILLIMETRES (that is what the slicer and the fastener specs speak). The scale
factor is recorded per part rather than assumed.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_manifest.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import params as P
import gen_sdf as G

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "render" / "assembly.json"

MESHES = "sim/models/tri_tiltrotor/meshes"
CADOUT = "cad/out"


def build(tilt_deg: float) -> dict:
    """Assembly at a given nacelle tilt. 0 = hover, 90 = cruise."""
    d = P.solve()
    g = G.geometry()
    a, c = d.wing_rotor_arm, d.tail_rotor_arm
    rz = P.NACELLE_Z_OFFSET
    y = P.WING_ROTOR_Y
    tilt = math.radians(tilt_deg)

    parts = [
        # --- airframe, metres, already positioned in the model frame ---
        dict(name="fuselage", mesh=f"{MESHES}/fuselage.stl", scale=1.0,
             loc=[0, 0, 0], rot=[0, 0, 0], material="body"),
        dict(name="wing", mesh=f"{MESHES}/wing.stl", scale=1.0,
             loc=[g["quarter_x"], 0, 0], rot=[0, 0, 0], material="skin"),
        dict(name="tail", mesh=f"{MESHES}/tail.stl", scale=1.0,
             loc=[0, 0, 0], rot=[0, 0, 0], material="skin"),
        # Carbon booms. Without these the nacelles hang in mid-air -- the
        # mounting hardware was in the BOM long before it was in the geometry.
        # Now BURIED inside the wing section rather than slung 13.7 mm beneath
        # it, so only the stub ahead of the leading edge is visible.
        dict(name="booms", mesh=f"{MESHES}/booms.stl", scale=1.0,
             loc=[0, 0, 0], rot=[0, 0, 0], material="carbon"),
    ]

    # --- control surfaces ---------------------------------------------------
    # Lofted aerofoil slices hinged on the real hinge line, not the grey boxes
    # that used to float near the trailing edge. Placed from the SAME
    # params call the meshes were built about, and drawn undeflected (the
    # renders are of a parked aircraft).
    ail = P.aileron_geometry()
    for label, sgn in (("left", +1.0), ("right", -1.0)):
        parts.append(dict(
            name=f"aileron_{label}",
            mesh=f"{MESHES}/aileron_{label}.stl", scale=1.0,
            loc=[g["quarter_x"] + ail["x"], sgn * ail["y_mid"], ail["z"]],
            rot=[0, 0, 0], material="skin"))
    for label, sgn in (("left", +1.0), ("right", -1.0)):
        rv = P.ruddervator_geometry(sgn)
        parts.append(dict(
            name=f"ruddervator_{label}",
            mesh=f"{MESHES}/ruddervator_{label}.stl", scale=1.0,
            loc=[rv["x"], rv["y"], rv["z"]],
            rot=[0, 0, 0], material="skin"))

    # --- nacelles and props ---
    # The tilt joint rotates about +y. At 0 rad the rotor axis is +z (up); at
    # +pi/2 it is +x (forward). Same convention as the SDF, same as PX4's
    # CA_SV_TL*: "an angle of zero means upwards".
    for label, px, py, prop, mirror in (
        ("left",  a,  +y, "prop_wing", False),
        ("right", a,  -y, "prop_wing", True),
        ("tail", -c,  0.0, "prop_tail", False),
    ):
        is_tail = label == "tail"
        # The TAIL rotor is FIXED and hover-only -- it never tilts, whatever
        # pose is being rendered. Locking it at zero, not 90: zero is UP.
        tilt = 0.0 if (is_tail and not P.TAIL_TILTS) else math.radians(tilt_deg)

        if is_tail:
            # Sits on the pylon at the fuselage waist, height derived from the
            # actual station table.
            hub_z = P.tail_rotor_z()
            yoke_z = hub_z - 0.030
        else:
            # The wing RISES with dihedral, so a nacelle pinned to a fixed z
            # floats free of it -- 28 mm of daylight at y = 0.400 m, obvious in
            # a close-up. Follow the wing surface.
            wing_z = abs(py) * math.tan(P.WING_DIHEDRAL)
            yoke_z = wing_z - 0.018
            hub_z = yoke_z + 0.030

        parts.append(dict(
            name=f"nacelle_yoke_{label}", mesh=f"{CADOUT}/nacelle_yoke.stl",
            scale=0.001, loc=[px, py, yoke_z], rot=[0, 0, 0],
            material="printed"))
        parts.append(dict(
            name=f"nacelle_cradle_{label}", mesh=f"{CADOUT}/nacelle_cradle.stl",
            scale=0.001, loc=[px, py, hub_z], rot=[0, tilt, 0],
            material="printed"))
        # Propeller: blades lie in the plane normal to the rotor axis, so it
        # inherits the same tilt. Offset along the (tilted) axis so it sits in
        # front of the motor rather than inside it.
        d_ax = 0.035
        parts.append(dict(
            name=f"prop_{label}", mesh=f"{MESHES}/{prop}.stl", scale=1.0,
            loc=[px + d_ax * math.sin(tilt), py, hub_z + d_ax * math.cos(tilt)],
            rot=[0, tilt, 0], material="prop", mirror=mirror))

    return dict(
        generated_from="cad/params.py",
        units="metres",
        tilt_deg=tilt_deg,
        aircraft=dict(
            mtow_kg=P.MASS_TOTAL,
            span_m=P.WING_SPAN,
            wing_area_m2=round(d.wing_area, 4),
            airfoil=f"NACA {P.WING_NACA}",
            stall_ms=round(d.v_stall, 2),
            cruise_ms=P.V_CRUISE,
        ),
        parts=parts,
    )


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    # Three poses: hover, mid-transition, cruise. The mid one is the hero.
    manifests = {
        "hover": build(0.0),
        "transition": build(45.0),
        "cruise": build(90.0),
    }
    OUT.write_text(json.dumps(manifests, indent=2), encoding="utf-8")

    n = len(manifests["hover"]["parts"])
    print(f"wrote {OUT}  ({n} parts x 3 poses)")


if __name__ == "__main__":
    main()
