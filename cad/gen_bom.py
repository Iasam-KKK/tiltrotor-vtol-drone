r"""
Generate docs/BOM.csv from params.py.

Specifications that the design actually constrains -- prop diameter, motor
thrust, bearing size, shaft diameter, servo torque, boom diameter -- are read
from params.py rather than retyped. If a parameter changes, the BOM changes
with it, and a part that no longer satisfies its invariant fails check() before
this file is ever written.

Prices are indicative hobby-market USD, and are the one thing here that is NOT
derived. They are marked as estimates in the output.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_bom.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import params as P

OUT = Path(__file__).resolve().parent.parent / "docs" / "BOM.csv"


def rows() -> list[dict]:
    d = P.solve()
    mm = 25.4  # in -> mm

    return [
        # --- propulsion ---
        dict(part="Brushless motor, wing nacelle",
             spec=f"28xx class, >= {P.WING_MOTOR_THRUST_MAX:.0f} N static thrust "
                  f"({P.WING_MOTOR_THRUST_MAX / 9.81:.2f} kgf)",
             qty=2, supplier="T-Motor / SunnySky / generic", unit_usd=28.00,
             note=f"hover draw {d.thrust_wing_each:.1f} N each"),
        dict(part="Brushless motor, tail nacelle",
             spec=f"22xx class, >= {P.TAIL_MOTOR_THRUST_MAX:.0f} N static thrust",
             qty=1, supplier="T-Motor / SunnySky / generic", unit_usd=19.00,
             note=f"hover draw {d.thrust_tail:.1f} N"),
        dict(part="Propeller, wing",
             spec=f"{P.WING_PROP_DIAMETER * 1000 / mm:.0f} in "
                  f"({P.WING_PROP_DIAMETER * 1000:.0f} mm), CW and CCW pair",
             qty=2, supplier="APC / T-Motor", unit_usd=6.50,
             note="counter-rotating pair, torque cancels"),
        dict(part="Propeller, tail",
             spec=f"{P.TAIL_PROP_DIAMETER * 1000 / mm:.0f} in "
                  f"({P.TAIL_PROP_DIAMETER * 1000:.0f} mm)",
             qty=1, supplier="APC / T-Motor", unit_usd=5.00,
             note="reaction torque uncompensated; trimmed by wing vectoring"),
        dict(part="ESC", spec="30 A BLHeli_32 / AM32", qty=3,
             supplier="generic", unit_usd=14.00, note=""),

        # --- tilt mechanism ---
        # qty was 3, left over from the tilting-tail layout. The tail rotor is
        # fixed (TAIL_TILTS = False), so only the two wing nacelles tilt.
        dict(part="Tilt servo",
             spec=f"digital metal-gear, >= {P.SERVO_STALL_TORQUE_KGCM:.0f} kg.cm",
             qty=P.N_SERVO_TILT, supplier="Savox / MKS / generic",
             unit_usd=22.00,
             note="8.0x margin over computed tilt load; wing nacelles only"),
        dict(part="Bearing, tilt axis",
             spec=f"686ZZ, {P.TILT_SHAFT_DIA_MM:.0f} x {P.BEARING_OD_MM:.0f} x "
                  f"{P.BEARING_WIDTH_MM:.0f} mm",
             qty=6, supplier="generic", unit_usd=1.20,
             note="two per nacelle; clearance fit, not press"),
        dict(part="Tilt shaft",
             spec=f"{P.TILT_SHAFT_DIA_MM:.0f} mm stainless, 60 mm long",
             qty=P.N_SERVO_TILT, supplier="generic", unit_usd=1.50, note=""),

        # --- control surfaces ---
        # ⚠ THESE WERE MISSING ENTIRELY. The BOM listed tilt servos and no
        # control-surface servos at all, while the PX4 airframe has allocated
        # four of them (CA_SV_CS0..3) since the V-tail was adopted. An aircraft
        # built to the old list would have had no roll, pitch or yaw control in
        # forward flight.
        dict(part="Control surface servo",
             spec=f"digital metal-gear, >= {P.SURFACE_SERVO_TORQUE_KGCM:.0f} kg.cm, "
                  f"9 g class",
             qty=P.N_SERVO_SURFACE, supplier="Savox / MKS / generic",
             unit_usd=13.00,
             note="2 aileron + 2 ruddervator; mounts in the wing/tail bays"),
        dict(part="Servo pushrod + clevis", spec="M2 threaded rod, ball link",
             qty=P.N_SERVO_SURFACE + P.N_SERVO_TILT, supplier="generic",
             unit_usd=1.80, note="one per servo"),
        dict(part="BEC, servo rail",
             spec=f"6S in, 5-6 V out, >= {int(P.N_SERVO_SURFACE * P.SERVO_STALL_CURRENT_A):d} A continuous",
             qty=1, supplier="Mateksys / generic", unit_usd=18.00,
             note=f"{P.N_SERVO_SURFACE + P.N_SERVO_TILT} servos x "
                  f"{P.SERVO_STALL_CURRENT_A:.1f} A stall = "
                  f"{(P.N_SERVO_SURFACE + P.N_SERVO_TILT) * P.SERVO_STALL_CURRENT_A:.0f} A peak; "
                  f"the FC regulator gives {P.FC_INTERNAL_BEC_MAX_A:.1f} A"),

        # --- structure ---
        dict(part="Carbon boom",
             spec=f"{P.BOOM_DIA_MM:.0f} mm OD tube, 400 mm",
             qty=2, supplier="generic", unit_usd=8.00,
             note="wing nacelle mounting"),
        dict(part="Printed nacelle cradle", spec="PETG or ASA, see cad/out/",
             qty=3, supplier="self-printed", unit_usd=0.60,
             note="6.8 cm3 each"),
        dict(part="Printed nacelle yoke", spec="PETG or ASA, see cad/out/",
             qty=3, supplier="self-printed", unit_usd=1.20,
             note="14.2 cm3 each"),
        dict(part="Printed fit coupon", spec="PETG, test print before committing",
             qty=1, supplier="self-printed", unit_usd=0.70,
             note="verifies M3 pattern and bearing seat"),

        # --- avionics ---
        dict(part="Flight controller", spec="Pixhawk 6C or 6X, PX4 v1.17.0",
             qty=1, supplier="Holybro", unit_usd=200.00,
             note=f"{P.N_SERVO_SURFACE + P.N_SERVO_TILT} servo + 3 motor "
                  f"outputs required"),
        dict(part="Camera, nose",
             spec=f"{P.CAMERA_WIDTH}x{P.CAMERA_HEIGHT} @ {P.CAMERA_FPS} fps, "
                  f"{math.degrees(P.CAMERA_HFOV):.0f} deg HFOV",
             qty=1, supplier="Arducam / generic", unit_usd=45.00,
             note=f"{P.CAMERA_MASS * 1000:.0f} g, pitched "
                  f"{math.degrees(P.CAMERA_TILT_DOWN):.0f} deg down; forward of "
                  f"every rotor disc"),
        dict(part="GPS / compass", spec="M9N or better", qty=1,
             supplier="Holybro", unit_usd=60.00, note=""),
        dict(part="Airspeed sensor", spec="differential pitot, I2C", qty=1,
             supplier="Holybro / Matek", unit_usd=55.00,
             note="NOT optional: PX4 gates transition on airspeed"),
        dict(part="Battery",
             spec=f"6S Li-ion, {P.MASS_BATTERY * 1000:.0f} g",
             qty=1, supplier="generic", unit_usd=95.00,
             note=f"{P.MASS_BATTERY / P.MASS_TOTAL * 100:.0f}% of MTOW"),

        # --- fasteners ---
        dict(part="M3 socket cap screws", spec="assorted 8-25 mm", qty=1,
             supplier="generic", unit_usd=9.00, note="kit"),
    ]


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    data = rows()
    OUT.parent.mkdir(parents=True, exist_ok=True)

    fields = ["part", "spec", "qty", "supplier", "unit_usd", "ext_usd", "note"]
    total = 0.0
    with OUT.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in data:
            r = dict(r)
            r["ext_usd"] = round(r["qty"] * r["unit_usd"], 2)
            total += r["ext_usd"]
            w.writerow(r)
        w.writerow(dict(part="TOTAL (indicative, excl. printing and shipping)",
                        spec="", qty="", supplier="", unit_usd="",
                        ext_usd=round(total, 2), note="prices are estimates"))

    print(f"wrote {OUT}  ({len(data)} line items, ~${total:,.2f})")


if __name__ == "__main__":
    main()
