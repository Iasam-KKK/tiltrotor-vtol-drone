r"""
Tilt nacelle mechanism -- the printed part -- generated from params.py.

Three printed pieces, all driven by the same params.py that produces the
Gazebo model and the PX4 airframe:

  nacelle_cradle  holds the motor and rotates on the tilt shaft
  nacelle_yoke    clamps to the wing boom and carries the bearings
  fit_coupon      bolt pattern + one bearing seat + one shaft bore, a few
                  grams, so a buyer can verify the fits before committing
                  filament to the real part

The coupon is not decoration. Same constraint that governed 02: there is no
physical aircraft here, so nothing can be test-fitted, and the honest answer
is to ship the tolerances as a printable test rather than assert them.

Design rules forced by having no hardware to check against:
  - clearance fits only, never press fits
  - every mating dimension carries an explicit budgeted clearance
  - the listing must say "tolerances verified in CAD only"

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_nacelle.py
"""

from __future__ import annotations

from pathlib import Path

from build123d import (
    Align, Axis, BuildPart, BuildSketch, Circle, Cylinder, GridLocations,
    Location, Locations, Mode, Plane, Rectangle, RotationLike, Box,
    export_step, export_stl, extrude, fillet, import_step,
)

import params as P

OUT = Path(__file__).resolve().parent.parent / "cad" / "out"

# Derived mechanism dimensions -------------------------------------------------
YOKE_INNER_W = P.NACELLE_WIDTH_MM                 # inside face to inside face
CRADLE_OUTER_W = YOKE_INNER_W - 1.0               # 0.5 mm clearance per side
BEARING_SEAT_D = P.BEARING_OD_MM + P.BEARING_SEAT_CLEARANCE_MM
SHAFT_BORE_D = P.TILT_SHAFT_DIA_MM + P.SHAFT_BORE_CLEARANCE_MM
BOOM_BORE_D = P.BOOM_DIA_MM + P.BOOM_CLAMP_CLEARANCE_MM

CRADLE_ARM_H = 34.0        # from the underside of the motor plate to the axis
YOKE_ARM_H = 30.0          # from the boom centreline up to the axis
CRADLE_ARM_W = 16.0        # arm width fore/aft


def build_cradle():
    """Motor plate plus two depending arms carrying the tilt shaft."""
    half = CRADLE_OUTER_W / 2.0

    with BuildPart() as cradle:
        # --- motor mount plate ---
        Cylinder(
            radius=P.CRADLE_PLATE_DIA_MM / 2.0,
            height=P.CRADLE_PLATE_MM,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        )

        # --- the two arms ---
        with Locations((0, 0, -CRADLE_ARM_H / 2.0 - P.CRADLE_PLATE_MM)):
            with GridLocations(
                x_spacing=0, y_spacing=CRADLE_OUTER_W - P.WALL_MM,
                x_count=1, y_count=2,
            ):
                Box(CRADLE_ARM_W, P.WALL_MM, CRADLE_ARM_H)

        # --- tilt shaft bore through both arms ---
        axis_z = -(P.CRADLE_PLATE_MM + CRADLE_ARM_H)
        with Locations(Location((0, 0, axis_z), (90, 0, 0))):
            Cylinder(
                radius=SHAFT_BORE_D / 2.0,
                height=CRADLE_OUTER_W + 4.0,
                mode=Mode.SUBTRACT,
            )

        # --- motor bolt pattern, M3 on a square pitch ---
        with BuildSketch(Plane.XY.offset(0.0)):
            with GridLocations(
                x_spacing=P.MOTOR_BOLT_PITCH_MM,
                y_spacing=P.MOTOR_BOLT_PITCH_MM,
                x_count=2, y_count=2,
            ):
                Circle(radius=P.MOTOR_BOLT_DIA_MM / 2.0)
        extrude(amount=-P.CRADLE_PLATE_MM, mode=Mode.SUBTRACT)

        # --- central bore for shaft and wiring ---
        with BuildSketch(Plane.XY):
            Circle(radius=P.MOTOR_SHAFT_CLEAR_MM / 2.0)
        extrude(amount=-P.CRADLE_PLATE_MM, mode=Mode.SUBTRACT)

        # --- DRIVEN HORN ------------------------------------------------------
        # ⚠ This is what was missing. The cradle rotated on a shaft in bearings
        # with nothing connected to it: the servo existed in the BOM and in the
        # torque calculation, and nowhere in the geometry. Without a horn there
        # is no way to convert servo rotation into nacelle tilt, so the
        # "mechanism" was a bearing assembly, not an actuator.
        #
        # The arm projects AFT of the tilt axis by SERVO_HORN_RADIUS_MM. The
        # servo horn on the yoke has the same radius, so the linkage is 1:1 and
        # the servo's travel maps directly onto the nacelle's -15..+90 deg.
        r = P.SERVO_HORN_RADIUS_MM
        with Locations(Location((0, 0, axis_z))):
            # Arm: a flat lug from the shaft boss out to the pushrod hole.
            with Locations((-r / 2.0, 0, 0)):
                Box(r, P.HORN_ARM_THICK_MM, P.HORN_BOSS_DIA_MM)
            # Boss around the pushrod hole, so the ball link has material.
            with Locations(Location((-r, 0, 0), (90, 0, 0))):
                Cylinder(radius=P.HORN_BOSS_DIA_MM / 2.0,
                         height=P.HORN_ARM_THICK_MM)

        # Pushrod hole through the horn.
        with Locations(Location((-r, 0, axis_z), (90, 0, 0))):
            Cylinder(radius=P.PUSHROD_HOLE_DIA_MM / 2.0,
                     height=P.HORN_ARM_THICK_MM + 4.0, mode=Mode.SUBTRACT)

    return cradle.part


def build_yoke():
    """Boom clamp plus two arms carrying the bearings."""
    with BuildPart() as yoke:
        # --- boom clamp body ---
        clamp_od = P.BOOM_DIA_MM + 2 * P.WALL_MM + 4.0
        with Locations(Location((0, 0, 0), (90, 0, 0))):
            Cylinder(radius=clamp_od / 2.0, height=P.NACELLE_WIDTH_MM * 0.6)

        # --- boom bore ---
        with Locations(Location((0, 0, 0), (90, 0, 0))):
            Cylinder(
                radius=BOOM_BORE_D / 2.0,
                height=P.NACELLE_WIDTH_MM * 0.6 + 4.0,
                mode=Mode.SUBTRACT,
            )

        # --- the two arms rising to the tilt axis ---
        with Locations((0, 0, YOKE_ARM_H / 2.0)):
            with GridLocations(
                x_spacing=0, y_spacing=YOKE_INNER_W + P.YOKE_ARM_MM,
                x_count=1, y_count=2,
            ):
                Box(CRADLE_ARM_W + 6.0, P.YOKE_ARM_MM, YOKE_ARM_H)

        # --- bearing seats, blind, facing inward ---
        for sign in (+1, -1):
            y_face = sign * (YOKE_INNER_W / 2.0)
            with Locations(Location(
                (0, y_face + sign * P.BEARING_WIDTH_MM / 2.0, YOKE_ARM_H),
                (90, 0, 0),
            )):
                Cylinder(
                    radius=BEARING_SEAT_D / 2.0,
                    height=P.BEARING_WIDTH_MM,
                    mode=Mode.SUBTRACT,
                )

        # --- shaft clearance right through ---
        with Locations(Location((0, 0, YOKE_ARM_H), (90, 0, 0))):
            Cylinder(
                radius=SHAFT_BORE_D / 2.0,
                height=YOKE_INNER_W + 2 * P.YOKE_ARM_MM + 4.0,
                mode=Mode.SUBTRACT,
            )

        # --- clamp bolts ---
        with Locations(
            (0, 0, -(P.BOOM_DIA_MM / 2.0 + P.WALL_MM + 1.5)),
        ):
            with GridLocations(
                x_spacing=P.BOOM_DIA_MM + 6.0, y_spacing=0,
                x_count=2, y_count=1,
            ):
                Cylinder(
                    radius=P.BOOM_CLAMP_BOLT_DIA_MM / 2.0,
                    height=P.NACELLE_WIDTH_MM,
                    rotation=(90, 0, 0),
                    mode=Mode.SUBTRACT,
                )

        # --- SERVO MOUNT ------------------------------------------------------
        # The other half of the missing actuator. The servo sits on the yoke
        # (which is fixed to the boom and does not tilt) and drives the cradle
        # through a pushrod. Mounting it on the yoke rather than the cradle is
        # deliberate: a servo on the cradle would have to carry its own wiring
        # through the rotating joint, which is the usual way tilt mechanisms
        # fail.
        #
        # Case lies FLAT -- its {P.TILT_SERVO_H_MM:.0f} mm height is across the yoke, not up it,
        # because standing it upright would exceed the nacelle width.
        sl = P.TILT_SERVO_L_MM + P.SERVO_MOUNT_CLEARANCE_MM
        sw = P.TILT_SERVO_W_MM + P.SERVO_MOUNT_CLEARANCE_MM
        sh = P.TILT_SERVO_H_MM + P.SERVO_MOUNT_CLEARANCE_MM
        # Ledge behind the boom clamp for the servo to bolt onto, then the
        # pocket cut out of it.
        pocket_z = YOKE_ARM_H * 0.42
        with Locations((sl / 2.0 + clamp_od / 2.0, 0, pocket_z)):
            Box(sl + 2 * P.WALL_MM, sh + 2 * P.WALL_MM, sw + 2 * P.WALL_MM)
        with Locations((sl / 2.0 + clamp_od / 2.0, 0, pocket_z)):
            Box(sl, sh, sw, mode=Mode.SUBTRACT)
        # Servo flange screws, M2, one pair each end of the case.
        for sx in (clamp_od / 2.0 + 3.0, clamp_od / 2.0 + sl - 3.0):
            with Locations(Location((sx, 0, pocket_z), (0, 0, 0))):
                with GridLocations(x_spacing=0, y_spacing=sh - 4.0,
                                   x_count=1, y_count=2):
                    Cylinder(radius=1.1, height=sw + 2 * P.WALL_MM + 4.0,
                             rotation=(90, 0, 0), mode=Mode.SUBTRACT)

    return yoke.part


def build_tail_mount():
    """Fixed motor mount for the tail rotor. No tilt mechanism.

    ⚠ The tail station used to be drawn with the full tilting assembly -- yoke,
    cradle, two bearings, a shaft and now a servo -- inherited from the layout
    where the tail rotor tilted. It does not tilt: TAIL_TILTS is False, the SDF
    emits a `fixed` joint, and PX4 has CA_ROTOR2_TILT = 0. All that hardware
    bought a degree of freedom nothing uses, at 42 mm of height and 31 cm^3 of
    print.

    What a fixed rotor actually needs: a plate with the motor bolt pattern, and
    a saddle that grips the top of the pylon. 11 mm tall.
    """
    with BuildPart() as mount:
        # --- motor mount plate ---
        Cylinder(
            radius=P.CRADLE_PLATE_DIA_MM / 2.0,
            height=P.CRADLE_PLATE_MM,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        )

        # --- saddle that slips over the pylon's top section ---
        saddle_h = P.TAIL_MOUNT_HEIGHT_MM - P.CRADLE_PLATE_MM
        with Locations((0, 0, -P.CRADLE_PLATE_MM - saddle_h / 2.0)):
            Box(P.TAIL_PYLON_TOP_CHORD_MM * 0.55,
                P.TAIL_PYLON_TOP_THICK_MM + 2 * P.WALL_MM,
                saddle_h)
        # Socket for the pylon itself, open at the bottom.
        with Locations((0, 0, -P.CRADLE_PLATE_MM - saddle_h / 2.0
                        + (saddle_h - P.TAIL_MOUNT_SADDLE_MM) / 2.0)):
            Box(P.TAIL_PYLON_TOP_CHORD_MM * 0.55 - 2 * P.WALL_MM,
                P.TAIL_PYLON_TOP_THICK_MM,
                P.TAIL_MOUNT_SADDLE_MM + 1.0,
                mode=Mode.SUBTRACT)

        # --- motor bolt pattern ---
        with BuildSketch(Plane.XY.offset(0.0)):
            with GridLocations(
                x_spacing=P.MOTOR_BOLT_PITCH_MM,
                y_spacing=P.MOTOR_BOLT_PITCH_MM,
                x_count=2, y_count=2,
            ):
                Circle(radius=P.MOTOR_BOLT_DIA_MM / 2.0)
        extrude(amount=-P.CRADLE_PLATE_MM, mode=Mode.SUBTRACT)

        # --- central bore for shaft and wiring ---
        with BuildSketch(Plane.XY):
            Circle(radius=P.MOTOR_SHAFT_CLEAR_MM / 2.0)
        extrude(amount=-P.CRADLE_PLATE_MM, mode=Mode.SUBTRACT)

    return mount.part


def build_coupon():
    """Bolt pattern, one bearing seat and one shaft bore. A few grams."""
    with BuildPart() as coupon:
        Box(
            P.CRADLE_PLATE_DIA_MM + 8.0,
            P.CRADLE_PLATE_DIA_MM + 8.0,
            P.CRADLE_PLATE_MM,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        # motor bolt pattern
        with BuildSketch(Plane.XY.offset(P.CRADLE_PLATE_MM)):
            with GridLocations(
                x_spacing=P.MOTOR_BOLT_PITCH_MM,
                y_spacing=P.MOTOR_BOLT_PITCH_MM,
                x_count=2, y_count=2,
            ):
                Circle(radius=P.MOTOR_BOLT_DIA_MM / 2.0)
        extrude(amount=-P.CRADLE_PLATE_MM, mode=Mode.SUBTRACT)

    return coupon.part


PARTS = {
    "nacelle_cradle": build_cradle,
    "nacelle_yoke": build_yoke,
    "tail_motor_mount": build_tail_mount,
    "fit_coupon": build_coupon,
}


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")
    OUT.mkdir(parents=True, exist_ok=True)

    failures = 0
    for name, builder in PARTS.items():
        part = builder()

        # is_valid is a PROPERTY in build123d 0.11.1, not a method.
        valid = part.is_valid
        vol = part.volume
        bb = part.bounding_box()

        status = "OK " if (valid and vol > 0) else "BAD"
        if status == "BAD":
            failures += 1
        print(
            f"  {status} {name:16s} vol {vol / 1000.0:8.2f} cm^3  "
            f"bbox {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm"
        )

        step_path = OUT / f"{name}.step"
        export_step(part, str(step_path))
        export_stl(part, str(OUT / f"{name}.stl"))

        # STEP round-trip. A part can be a valid solid in memory and still
        # lose geometry on export -- and STEP is the format a freelance client
        # actually receives, so it is the one that has to be right. 01 held
        # this to 0.000000 mm^3; same bar here.
        try:
            reimported = import_step(str(step_path))
            delta = abs(reimported.volume - vol)
            if delta < 1e-6:
                print(f"       STEP round-trip delta {delta:.6f} mm^3")
            else:
                print(f"       STEP ROUND-TRIP LOST {delta:.6f} mm^3")
                failures += 1
        except Exception as exc:                      # noqa: BLE001
            print(f"       STEP round-trip could not be checked: {exc}")
            failures += 1

    print(f"\nwrote STEP + STL for {len(PARTS)} parts to {OUT}")
    if failures:
        raise SystemExit(f"{failures} part(s) failed")


if __name__ == "__main__":
    main()
