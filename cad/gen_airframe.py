r"""
Generate the PX4 airframe file for the tri-tiltrotor from params.py.

Every parameter value traces to params.py or to the actuator map in gen_sdf.py,
so the SDF and the airframe cannot disagree about which output drives what.
That mismatch is silent when it happens -- the aircraft simply misbehaves --
which is exactly why both are generated from one source.

All parameter names and semantics below were read out of the pinned PX4
v1.17.0 tree, not from the docs for main:
  src/modules/control_allocator/module.yaml
  ROMFS/px4fmu_common/init.d-posix/airframes/4020_gz_tiltrotor

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_airframe.py
"""

from __future__ import annotations

import math
from pathlib import Path

import params as P
import gen_sdf as G

AIRFRAME_ID = 4030          # verified free in the v1.17.0 tree
AIRFRAME_NAME = "gz_tri_tiltrotor"
OUT = Path(__file__).resolve().parent.parent / "sim" / "airframes"


def norm_tilt(angle_rad: float, mina_deg: float, maxa_deg: float) -> float:
    """Map a physical tilt angle to PX4's 0..1 normalised servo position."""
    a = math.degrees(angle_rad)
    return (a - mina_deg) / (maxa_deg - mina_deg)


def build() -> str:
    d = P.solve()

    # Wing nacelles carry vectored yaw, so they travel aft of vertical.
    wing_mina = -math.degrees(P.TILT_YAW_TRAVEL)
    wing_maxa = math.degrees(P.TILT_ANGLE_CRUISE)
    # The tail nacelle has no yaw role and stops at vertical.
    tail_mina = math.degrees(P.TILT_ANGLE_HOVER)
    tail_maxa = math.degrees(P.TILT_ANGLE_CRUISE)

    tilt_mc = norm_tilt(P.TILT_ANGLE_HOVER, wing_mina, wing_maxa)
    tilt_fw = norm_tilt(P.TILT_ANGLE_CRUISE, wing_mina, wing_maxa)

    # Rotor speed limits, consistent with the motorConstant used in the SDF.
    max_rot_wing = math.sqrt(P.WING_MOTOR_THRUST_MAX / 2.0e-05)
    max_rot_tail = math.sqrt(P.TAIL_MOTOR_THRUST_MAX / 1.2e-05)

    a = d.wing_rotor_arm
    c = d.tail_rotor_arm
    y = P.WING_ROTOR_Y

    # PX4's own default, kept for the wing so the absolute scale is unchanged;
    # the tail is scaled by the real thrust ratio. See the CT block below.
    CT_REF = 6.5
    ct_tail = CT_REF * (P.TAIL_MOTOR_THRUST_MAX / P.WING_MOTOR_THRUST_MAX)

    L = []
    add = L.append

    add(f"""#!/bin/sh
#
# @name Tri-Tiltrotor VTOL
#
# @type VTOL Tiltrotor
#
# GENERATED FILE. DO NOT EDIT BY HAND.
# Source of truth: cad/params.py   Generator: cad/gen_airframe.py
#
# Three tilting nacelles: two on the wing ahead of the CG, one at the tail
# behind it. The rotors straddle the CG, which is what makes hover pitch
# trimmable -- a layout with both rotors aft of the CG cannot be trimmed.
#
# Hover trim solved in params.py, not tuned by hand:
#   wing rotors {d.thrust_wing_each:.2f} N each, tail rotor {d.thrust_tail:.2f} N
#   tail carries {d.tail_lift_fraction * 100:.1f}% of hover lift
#

. ${{R}}etc/init.d/rc.vtol_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=default}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={G.MODEL_NAME}}}

param set-default SIM_GZ_EN 1

param set-default MAV_TYPE 21
param set-default CA_AIRFRAME 3
""")

    add(f"""
# ---------------------------------------------------------------------------
# Rotors.  PX4 body frame is FRD: +x forward, +y RIGHT, +z down.
# The Gazebo model frame is FLU (+y LEFT), so lateral signs are inverted here
# relative to the SDF. Getting this wrong mirrors the aircraft and inverts roll.
# ---------------------------------------------------------------------------
param set-default CA_ROTOR_COUNT 3

# CA_ROTOR{{i}}_CT -- "Thrust = CT * u^2", u being the output signal, per
# src/modules/control_allocator/module.yaml in the pinned v1.17.0 tree.
#
# ⚠ THIS BLOCK USED TO OMIT CT, AND THAT CAUSED THE NOSE-UP ON TAKEOFF.
# The default is 6.5 for every rotor. Stock 4020_gz_tiltrotor also omits it and
# is fine, because all four of ITS rotors are identical: a wrong-but-equal CT
# is common mode, it just rescales total thrust and the altitude loop absorbs
# it. Our three rotors are NOT identical -- {P.WING_MOTOR_THRUST_MAX:.0f} N on the wing against
# {P.TAIL_MOTOR_THRUST_MAX:.0f} N at the tail -- so one shared CT biases the wing/tail SPLIT.
#
# PX4 asked the tail for {d.thrust_tail:.2f} N and the tail delivered about
# {100 * P.TAIL_MOTOR_THRUST_MAX / P.WING_MOTOR_THRUST_MAX:.0f}% of it. The missing up-force acts {c:.3f} m BEHIND the CG,
# which is an unopposed nose-up moment of roughly
# {d.thrust_tail * (1 - P.TAIL_MOTOR_THRUST_MAX / P.WING_MOTOR_THRUST_MAX) * c:.2f} N.m -- tail drops, nose rises.
#
# Only the RATIO matters (the allocator normalises the absolute scale), so the
# wing keeps PX4's 6.5 and the tail is scaled by the true thrust ratio.
param set-default CA_ROTOR0_CT {CT_REF:.4f}
param set-default CA_ROTOR1_CT {CT_REF:.4f}
param set-default CA_ROTOR2_CT {ct_tail:.4f}

# Rotor 0 -- left wing nacelle (ccw)
param set-default CA_ROTOR0_PX {a:.4f}
param set-default CA_ROTOR0_PY {-y:.4f}
param set-default CA_ROTOR0_KM 0.05

# Rotor 1 -- right wing nacelle (cw)
param set-default CA_ROTOR1_PX {a:.4f}
param set-default CA_ROTOR1_PY {y:.4f}
param set-default CA_ROTOR1_KM -0.05

# Rotor 2 -- tail nacelle (ccw)
param set-default CA_ROTOR2_PX {-c:.4f}
param set-default CA_ROTOR2_PY 0.0000
param set-default CA_ROTOR2_KM 0.05

# Rotor -> tilt servo assignment. CA_ROTOR{{i}}_TILT is 1-INDEXED
# (0 means "not tilting"), while CA_SV_TL{{i}}_* are 0-indexed.
param set-default CA_ROTOR0_TILT 1
param set-default CA_ROTOR1_TILT 2
param set-default CA_ROTOR2_TILT {3 if P.TAIL_TILTS else 0}
""")

    add(f"""
# ---------------------------------------------------------------------------
# Tilt servos.  PX4: "An angle of zero means upwards."  TD 0 = Towards Front.
# CT: 0 None, 1 Yaw, 2 Pitch, 3 Yaw and Pitch.
#
# The wing pair carries yaw (differential tilt about the CG gives a couple
# about z) and travels {abs(wing_mina):.0f} deg aft of vertical to do it.
# The tail nacelle carries pitch. It cannot contribute yaw at all: a
# centreline force in the x-z plane produces no moment about z.
# ---------------------------------------------------------------------------
param set-default CA_SV_TL_COUNT {3 if P.TAIL_TILTS else 2}

param set-default CA_SV_TL0_MINA {wing_mina:.0f}
param set-default CA_SV_TL0_MAXA {wing_maxa:.0f}
param set-default CA_SV_TL0_TD 0
param set-default CA_SV_TL0_CT 1

param set-default CA_SV_TL1_MINA {wing_mina:.0f}
param set-default CA_SV_TL1_MAXA {wing_maxa:.0f}
param set-default CA_SV_TL1_TD 0
param set-default CA_SV_TL1_CT 1
""")

    if P.TAIL_TILTS:
        add(f"""param set-default CA_SV_TL2_MINA {tail_mina:.0f}
param set-default CA_SV_TL2_MAXA {tail_maxa:.0f}
param set-default CA_SV_TL2_TD 0
param set-default CA_SV_TL2_CT 2
""")
    else:
        add(f"""# Rotor 2 is FIXED and hover-only: CA_ROTOR2_TILT = 0 above, and there is no
# third tilt servo. It carries pitch by thrust modulation alone, and stops in
# cruise -- a folding prop streamlines it against the pylon.
# Hover trim: {d.thrust_tail:.2f} N, {d.tail_lift_fraction * 100:.1f}% of lift.
""")

    gam = d.tail_dihedral
    add(f"""
# ---------------------------------------------------------------------------
# Control surfaces.  Types: 1 Left Aileron, 2 Right Aileron, 7 Left V-Tail,
# 8 Right V-Tail.
#
# The wing surfaces are AILERONS, not elevons: the V-tail carries pitch, so
# putting pitch on the wing as well would double-book it.
#
# The V-tail panels sit at {math.degrees(gam):.1f} deg dihedral, derived in
# params.py from the pitch and yaw effectiveness required. Each ruddervator
# therefore contributes cos({math.degrees(gam):.1f}) to pitch and
# sin({math.degrees(gam):.1f}) to yaw -- deflected together they are an
# elevator, differentially they are a rudder.
# ---------------------------------------------------------------------------
param set-default CA_SV_CS_COUNT 4

param set-default CA_SV_CS0_TYPE 1
param set-default CA_SV_CS0_TRQ_R -0.5

param set-default CA_SV_CS1_TYPE 2
param set-default CA_SV_CS1_TRQ_R 0.5

param set-default CA_SV_CS2_TYPE 7
param set-default CA_SV_CS2_TRQ_P {math.cos(gam):.4f}
param set-default CA_SV_CS2_TRQ_Y {math.sin(gam):.4f}

param set-default CA_SV_CS3_TYPE 8
param set-default CA_SV_CS3_TRQ_P {math.cos(gam):.4f}
param set-default CA_SV_CS3_TRQ_Y {-math.sin(gam):.4f}
""")

    add(f"""
# ---------------------------------------------------------------------------
# Gazebo bridge -- motors.  SIM_GZ_EC_FUNC{{n}} = 100 + motor number.
# Output n (1-indexed) drives SDF motorNumber n-1.
# ---------------------------------------------------------------------------
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103

param set-default SIM_GZ_EC_MIN1 10
param set-default SIM_GZ_EC_MIN2 10
param set-default SIM_GZ_EC_MIN3 10

param set-default SIM_GZ_EC_MAX1 {max_rot_wing:.0f}
param set-default SIM_GZ_EC_MAX2 {max_rot_wing:.0f}
param set-default SIM_GZ_EC_MAX3 {max_rot_tail:.0f}
""")

    add(f"""
# ---------------------------------------------------------------------------
# Gazebo bridge -- servos.  ORDER IS LOAD-BEARING: PX4 allocates control
# surfaces first, then tilt servos. Stock 4020_gz_tiltrotor demonstrates this
# (3 surfaces on servos 1-3, then tilt angle params on servos 4-5).
#
#   servo 1..4 -> CA_SV_CS0..3   (aileron L, aileron R, V-tail L, V-tail R)
#   servo 5..7 -> CA_SV_TL0..2   (tilt L, tilt R, tilt tail)
#
# gz sub_topic servo_N corresponds to SIM_GZ_SV_*{{N+1}}.
# ---------------------------------------------------------------------------
param set-default SIM_GZ_SV_FUNC1 201
param set-default SIM_GZ_SV_FUNC2 202
param set-default SIM_GZ_SV_FUNC3 203
param set-default SIM_GZ_SV_FUNC4 204
param set-default SIM_GZ_SV_FUNC5 205
param set-default SIM_GZ_SV_FUNC6 206
{"param set-default SIM_GZ_SV_FUNC7 207" if P.TAIL_TILTS else ""}
param set-default SIM_GZ_SV_MINA5 {wing_mina:.0f}
param set-default SIM_GZ_SV_MAXA5 {wing_maxa:.0f}
param set-default SIM_GZ_SV_MINA6 {wing_mina:.0f}
param set-default SIM_GZ_SV_MAXA6 {wing_maxa:.0f}
{f"param set-default SIM_GZ_SV_MINA7 {tail_mina:.0f}" if P.TAIL_TILTS else ""}
{f"param set-default SIM_GZ_SV_MAXA7 {tail_maxa:.0f}" if P.TAIL_TILTS else ""}
""")

    add(f"""
# ---------------------------------------------------------------------------
# VTOL and fixed-wing envelope.  These come straight out of params.py's
# aerodynamic solve rather than being guessed:
#   stall {d.v_stall:.2f} m/s -> transition {d.v_transition:.2f} m/s -> cruise {P.V_CRUISE:.1f} m/s
# ---------------------------------------------------------------------------
param set-default VT_TYPE 1
param set-default VT_TILT_MC {tilt_mc:.4f}
param set-default VT_TILT_TRANS 0.6
param set-default VT_TILT_FW {tilt_fw:.4f}
param set-default VT_ARSP_TRANS {d.v_transition:.1f}
param set-default VT_ARSP_BLEND {d.v_stall * 1.10:.1f}
param set-default VT_B_TRANS_DUR 5.0
param set-default VT_FWD_THRUST_EN 4
param set-default VT_FWD_THRUST_SC 0.6

param set-default FW_AIRSPD_STALL {d.v_stall:.1f}
param set-default FW_AIRSPD_MIN {d.v_stall * 1.25:.1f}
param set-default FW_AIRSPD_TRIM {P.V_CRUISE:.1f}
param set-default FW_AIRSPD_MAX {P.V_CRUISE * 1.5:.1f}

param set-default MC_AIRMODE 1
param set-default MIS_TAKEOFF_ALT 10
""")

    return "".join(L)


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    OUT.mkdir(parents=True, exist_ok=True)
    fn = OUT / f"{AIRFRAME_ID}_{AIRFRAME_NAME}"
    text = build()
    fn.write_text(text, encoding="utf-8", newline="\n")

    n_params = text.count("param set-default")
    print(f"wrote {fn}  ({len(text.splitlines())} lines, {n_params} params)")

    # Cross-check the actuator map against gen_sdf so the two cannot drift.
    expect = {
        G.SERVO_AILERON_LEFT: 0, G.SERVO_AILERON_RIGHT: 1,
        G.SERVO_VTAIL_LEFT: 2, G.SERVO_VTAIL_RIGHT: 3,
        G.SERVO_TILT_LEFT: 4, G.SERVO_TILT_RIGHT: 5,
    }
    if P.TAIL_TILTS:
        expect[G.SERVO_TILT_TAIL] = 6
    assert expect == {k: k for k in expect}, (
        "gen_sdf servo map is not surfaces-then-tilts; PX4 allocates in that "
        "order and the SDF must match")
    n_tilt = 3 if P.TAIL_TILTS else 2
    print(f"actuator map agrees with gen_sdf: surfaces 0-3, "
          f"{n_tilt} tilts 4-{3 + n_tilt}")


if __name__ == "__main__":
    main()
