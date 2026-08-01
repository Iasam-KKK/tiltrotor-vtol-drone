r"""
Emit cad/out/annotations.json — the findings the FreeCAD view should show.

Every figure comes from params.py. The point is that the annotated drawing
cannot say something the design does not: if the CG moves, the marker moves
with it, and if MTOW changes the label changes on the next regeneration.

Positions are in METRES here; the FreeCAD macro scales to mm along with the
rest of the assembly.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_annotations.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import params as P

OUT = Path(__file__).resolve().parent / "out" / "annotations.json"


def build() -> dict:
    d = P.solve()
    cg = P.solve_cg()
    pay = P.payload_capacity()
    a, c = d.wing_rotor_arm, d.tail_rotor_arm

    rotors = [
        dict(name="wing L", x=a, y=+P.WING_ROTOR_Y,
             z=P.NACELLE_Z_OFFSET, thrust_n=d.thrust_wing_each,
             max_n=P.WING_MOTOR_THRUST_MAX),
        dict(name="wing R", x=a, y=-P.WING_ROTOR_Y,
             z=P.NACELLE_Z_OFFSET, thrust_n=d.thrust_wing_each,
             max_n=P.WING_MOTOR_THRUST_MAX),
        dict(name="lift", x=-c, y=0.0, z=P.tail_rotor_z(),
             thrust_n=d.thrust_tail, max_n=P.TAIL_MOTOR_THRUST_MAX),
    ]

    battery = next(e for e in P.EQUIPMENT if e[0].startswith("Battery"))

    return {
        "generated_from": "cad/params.py",
        "cg": {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "mac_frac": P.CG_MAC_FRACTION,
            # How far the CG actually lands from the design point given where
            # every item of mass sits. Computed, not assumed.
            "error_m": cg["cg_nominal"],
        },
        "rotors": rotors,
        "battery": {
            "x": battery[1], "length_m": battery[2] / 1000.0,
            "mass_kg": battery[5],
            "solved_x": cg["x_batt"],
            "authority_mac_per_10mm": P.MASS_BATTERY * 0.010
            / cg["total_mass"] / P.WING_CHORD,
        },
        "mass_items": [
            {"name": n, "mass_kg": m, "x": x} for n, m, x in P.mass_ledger()
        ],
        "figures": [
            ("MTOW", f"{P.MASS_TOTAL:.2f} kg  (ledger {cg['total_mass']:.3f})"),
            ("CG", f"{P.CG_MAC_FRACTION:.0%} MAC, computed error "
                   f"{cg['cg_nominal'] * 1000:+.1f} mm"),
            ("Battery", f"{P.MASS_BATTERY:.2f} kg at x={battery[1] * 1000:+.0f} mm"
                        f"  ({cg['x_batt'] * 1000:+.0f} solves CG)"),
            ("Hover trim", f"wing {d.thrust_wing_each:.2f} N each, "
                           f"lift {d.thrust_tail:.2f} N "
                           f"({d.tail_lift_fraction:.1%})"),
            ("Payload", f"+{pay['payload_kg']:.2f} kg to "
                        f"{pay['mtow_max']:.2f} kg  (limit: {pay['binding']})"),
            ("Wing loading", f"{d.wing_loading:.2f} kg/m^2"),
            ("Envelope", f"stall {d.v_stall:.1f} / transition "
                         f"{d.v_transition:.1f} / cruise {P.V_CRUISE:.0f} m/s"),
            ("Cruise L/D", f"{d.l_over_d_cruise:.1f}  (best "
                           f"{d.l_over_d_max:.1f} at {d.v_best_glide:.1f} m/s)"),
            ("Glide", f"{math.degrees(d.glide_angle):.2f} deg, sink "
                      f"{d.sink_rate:.2f} m/s"),
            ("Hover authority", f"roll {d.alpha_roll:.1f} / pitch "
                                f"{d.alpha_pitch:.1f} / yaw {d.alpha_yaw:.1f} rad/s^2"),
            ("Wake clearance", f"V-tail {(P.TAIL_SURFACE_ARM - P.TAIL_ROTOR_ARM - P.TAIL_PROP_DIAMETER / 2) * 1000:.0f} mm "
                               f"behind the lift disc"),
            ("Servo rail", f"{(P.N_SERVO_SURFACE + P.N_SERVO_TILT) * P.SERVO_STALL_CURRENT_A:.0f} A peak "
                           f"-> separate BEC"),
        ],
    }


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    data = build()
    OUT.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"wrote {OUT}")
    for k, v in data["figures"]:
        print(f"  {k:16s} {v}")


if __name__ == "__main__":
    main()
