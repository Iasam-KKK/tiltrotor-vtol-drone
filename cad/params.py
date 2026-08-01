r"""
Single source of truth for the tri-tiltrotor VTOL.

Every dimension, mass and aerodynamic coefficient in this project lives HERE and
nowhere else. The Gazebo SDF, the PX4 airframe file, the URDF and the printed
nacelle CAD are all *generated* from this module, so the simulated aircraft and
the printed part cannot drift apart.

This is the same structure that worked on 01-nav2-deck. `check()` runs the design
invariants in plain arithmetic BEFORE any CAD kernel or simulator starts, and
refuses to emit an aircraft that cannot hover, cannot trim, or whose props
intersect. A geometry error should fail here in 30 ms, not after a 90 s build.

Axes (PX4/ROS FRD body frame, right-handed):
    +x forward (nose)     +y right (starboard)     +z down

Longitudinal stations are quoted as distances FROM THE CG, positive forward.

Run directly to print the design report:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\params.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------

G = 9.80665          # m/s^2
RHO = 1.2041         # kg/m^3, ISA sea level 20 C -- matches PX4's stock SDF


# ---------------------------------------------------------------------------
# Design inputs.  These are the only numbers a human should edit.
# ---------------------------------------------------------------------------

# --- Mass -------------------------------------------------------------------
# Every item is listed separately. Folding the battery into "fuselage" is how
# a mass budget silently stops closing -- check() enforces that these sum to
# MTOW, and it caught exactly that error on the first run of this file.
# ⚠ 4.80 -> 4.64 kg, reconciled against mass_ledger(). The drop is almost
# entirely the tail nacelle losing its tilt hardware (0.25 -> 0.10 kg). MTOW is
# no longer a number typed at the top and defended by adjusting others to match:
# the "declared MTOW matches the mass ledger" invariant now checks it against
# the itemised list.
MASS_TOTAL = 4.64            # kg, MTOW
MASS_WING_PANEL = 0.55       # kg, each of two wing panels
MASS_FUSELAGE = 0.85         # kg, fuselage structure only
MASS_AVIONICS = 0.35         # kg, FC + GPS + ESCs + wiring
MASS_BATTERY = 1.17          # kg, 6S 8000 mAh Li-ion
MASS_PAYLOAD = 0.16          # kg, camera + 2D lidar
MASS_TAIL_ASSY = 0.22        # kg, tail surfaces + boom
MASS_NACELLE_WING = 0.35     # kg, each wing nacelle (motor + tilt servo + mount)
# ⚠ 0.25 -> 0.10 kg. This was sized when the tail nacelle TILTED: it carried a
# yoke, a cradle, two 686ZZ bearings, a 6 mm shaft and a 20 kg.cm servo. All of
# that is gone -- the rotor is fixed and the mount is now a 5.1 cm^3 plate.
# What is left is the motor (~55 g), the printed mount (~6 g) and prop plus
# wiring (~35 g).
#
# This matters far beyond 150 g of MTOW: it sits 620 mm behind the CG, so the
# stale figure was contributing 93 g.m of phantom aft moment and dragging the
# whole CG solution aft with it.
MASS_NACELLE_TAIL = 0.10     # kg, fixed tail motor + mount + prop

# Mass carried at or very near the CG. Used for the pitch-inertia estimate.
MASS_CENTRAL = MASS_FUSELAGE + MASS_AVIONICS + MASS_BATTERY + MASS_PAYLOAD

# --- Wing -------------------------------------------------------------------
WING_SPAN = 2.00             # m, tip to tip
WING_CHORD = 0.26            # m, MEAN aerodynamic chord
WING_TAPER = 0.65            # tip chord / root chord
WING_DIHEDRAL = math.radians(4.0)
WING_TWIST_TIP = math.radians(-2.0)   # washout: tip stalls after the root
WING_LE_SWEEP = math.radians(3.0)

# --- Winglets ---------------------------------------------------------------
# A winglet recovers some induced drag by making the tip vortex do less work,
# which is worth more on a high-aspect-ratio wing like this one than on a
# stubby one. Height is quoted as a fraction of SEMI-span.
#
# The drag benefit below is an ESTIMATE from the standard span-efficiency
# correction, not a computed result. It is the one aerodynamic number in this
# file that is not derived, and it is flagged as such in the report.
WINGLET_HEIGHT_FRAC = 0.085      # of semi-span
WINGLET_CANT = math.radians(72.0)  # from horizontal; 90 = vertical
WINGLET_TAPER = 0.42             # tip chord / winglet root chord
WINGLET_SWEEP = math.radians(28.0)
WINGLET_TOE = math.radians(-1.5)   # slight toe-out, standard practice

# --- Tail rotor configuration -----------------------------------------------
# True  : the tail nacelle tilts 0-90 deg and becomes a pusher in cruise.
#         This is the configuration PX4 does not ship, and the reason this
#         project exists.
# False : the tail rotor is fixed vertical, stops in cruise, and is dead
#         weight plus drag -- the conventional arrangement.
#
# Priced: a stopped, unfeathered 10 in prop presents roughly 0.004 m^2 of flat
# plate. At cruise that is about 1.0 N against a 3.5 N cruise requirement, so
# roughly 30% more cruise drag. Folding props recover most of it, at the cost
# of another mechanism.
TAIL_TILTS = False

# Airfoil, NACA 4-digit. 2412 is a conventional cambered section: 2% camber at
# 40% chord, 12% thick. Chosen because its 2-D characteristics are published
# and standard, so the coefficients below are traceable rather than invented.
# 2410, not 2412: 10% thick rather than 12%. At this Reynolds number and wing
# loading the extra 2% buys structural depth we do not need and costs profile
# drag and frontal area. It also simply looks like an aircraft rather than a
# glider trainer.
WING_NACA = "2410"
TAIL_NACA = "0009"           # symmetric, as tail sections must be, and thin

# Section 2-D properties for NACA 2410 at low Reynolds number.
# These are the SOURCE numbers; the finite-wing values used in flight are
# derived from them in solve(), not typed in separately. Going from 12% to 10%
# thickness costs a little CL_max and returns a little drag -- both reflected
# here rather than left at the 2412 values.
WING_CL_ALPHA_2D = 2.0 * math.pi   # 1/rad, thin-airfoil theory
WING_CL_MAX_2D = 1.40              # 2-D section maximum (2412 was 1.45)
WING_CD_MIN = 0.0069               # section minimum drag (2412 was 0.0075)
WING_ALPHA_ZERO_LIFT = math.radians(-2.0)
WING_ALPHA_STALL_2D = math.radians(15.5)
OSWALD_E = 0.80

# Non-lifting parasitic drag: fuselage, nacelles, booms, gear. Expressed as an
# equivalent flat-plate area so it does not silently scale with wing area.
PARASITE_AREA = 0.0075       # m^2

# --- Fuselage shape ---------------------------------------------------------
# Stations as (x from nose / total length, half-height m, half-width m).
# Lofted through these, so it is a shaped body rather than a box.
# ⚠ 1.35 -> 1.55 m. At 1.35 the body ENDED at x = -0.872 m while the V-tail's
# root chord runs -0.825 .. -1.005 m: only the forward 47 mm of a 180 mm root
# had any fuselage under it and the remaining 133 mm cantilevered off the back
# attached to nothing. Widening the last station (the previous fix) gave the
# root something to bolt to but did not make the body reach far enough aft.
#
# The station table below is re-fractioned for the new length so the waist
# still lands under the lift rotor, and now ends as a slender constant-section
# tail boom that runs past the V-tail trailing edge instead of tapering to a
# point under its leading edge.
FUSELAGE_LENGTH = 1.55       # m, runs aft of the V-tail trailing edge
# Slimmer and more streamlined than the first pass: maximum section reduced
# from 0.072 to 0.061 m half-height, and more stations so the loft is smooth
# rather than visibly segmented. Fineness ratio (length / max diameter) goes
# from 8.2 to 10.2, which is squarely in the low-drag range for a body of
# revolution; below about 6 the pressure drag climbs sharply.
FUSELAGE_STATIONS = (
    (0.000, 0.008, 0.008),
    (0.030, 0.030, 0.027),
    (0.080, 0.044, 0.039),
    (0.160, 0.055, 0.047),
    (0.280, 0.060, 0.051),
    (0.360, 0.061, 0.052),
    (0.440, 0.059, 0.050),
    (0.520, 0.054, 0.045),
    (0.610, 0.044, 0.036),
    (0.690, 0.033, 0.026),
    # --- cut-down rear deck ---
    # The upper rear fuselage is cut away hard from here back. Two reasons:
    #   1. wetted area. This section carries no payload, no structure worth
    #      the name and no fuel -- it is pure skin friction.
    #   2. it lets the lift rotor sit LOW without burying it. The pylon drops
    #      from 75 mm to 34 mm because the deck it stands on came down to meet
    #      it, rather than the rotor coming down into the wash.
    # The result is a tail boom rather than a tapering body.
    (0.760, 0.022, 0.018),
    # From here aft it is a TAIL BOOM of near-constant section, not a taper to
    # a point. The V-tail root chord sits on 0.86 .. 1.00, so the body has to
    # still be there over all of it.
    (0.800, 0.018, 0.015),
    (0.860, 0.016, 0.013),
    (0.910, 0.015, 0.013),
    (0.960, 0.015, 0.012),
    # ⚠ CORRECTED. This station used to be (1.000, 0.005, 0.004): the body
    # tapered to a 10 x 8 mm needle exactly where the V-tail bolts on. The tail
    # root chord is 180 mm and its NACA 0009 section is 16.2 mm thick, so a
    # 0.128 m^2 tail was attached to something half as thick as itself. It
    # would have failed on the first hard landing.
    #
    # check() passed it because the only relevant invariant was "fuselage is
    # long enough to carry the tail", which compares LENGTH (1.35 m vs 0.956 m)
    # and never looks at whether there is any SECTION at the root.
    #
    # Ending the body as a small constant boom instead of a point gives the
    # root 28 x 24 mm to land on. Deliberately chosen not to disturb the waist
    # invariants: waist_half is max(h) over frac >= 0.80, which is 0.016 at the
    # 0.845 station either way, so the cut-down-deck and waisting checks are
    # unaffected.
    (1.000, 0.014, 0.012),
)

# --- Longitudinal layout ----------------------------------------------------
# CG position as a fraction of MAC, measured aft of the wing leading edge.
CG_MAC_FRACTION = 0.28       # 28% MAC
# Wing rotor plane, measured FORWARD of the wing leading edge (on booms).
WING_ROTOR_AHEAD_OF_LE = 0.100   # m
# Tail rotor plane, measured AFT of the CG.
# Moved FORWARD of the V-tail (was 0.780, behind it) to sit on a pylon at the
# fuselage waist. This is the industry-standard arrangement, and it is only
# viable because the tail rotor is hover-only: in cruise it is stopped, so it
# lays no wake over the tail surfaces -- the one condition in which tail
# effectiveness actually matters for stability.
# ⚠ 0.700 -> 0.620. At 0.700 the disc swept back to 0.827 m and the V-tail sat
# at 0.870 m -- a 43 mm gap from a 254 mm rotor. The old invariant passed it
# because it only asked "is the tail INSIDE the disc band, yes/no", and 43 mm
# is technically outside. A hover wake contracts AND spreads; 43 mm is nothing.
# The V-tail was flying in the lift rotor's downwash in hover, which is a
# nose-down disturbance exactly when attitude hold matters most.
#
# Moving the rotor forward costs a little: a shorter arm means the tail rotor
# carries MORE of the hover lift (9.32 -> 10.3 N), which the trim solve and the
# motor-headroom check both re-verify rather than my asserting it is fine.
TAIL_ROTOR_ARM = 0.620       # m
# Pylon height above the fuselage upper surface.
#
# Cut down from 75 mm. The original justification -- lift the rotor out of the
# fuselage boundary layer -- is weaker than it sounds: in HOVER there is no
# freestream, so there is no boundary layer, and in CRUISE this rotor is
# stopped. It only genuinely matters through the brief transition.
#
# The reason not to delete the pylon entirely is download, not inflow: with the
# hub sitting on the skin, the fuselage sits inside the rotor's own downwash
# and steals thrust. Measured trade:
#   pylon drag in cruise : ~0.03-0.05 N of a 3.35 N requirement  (~1.5%)
#   download if recessed : ~0.5-0.9 N of a 9.32 N lift rotor     (~5-10%)
# So a short pylon wins, but only just, and mostly in hover.
TAIL_PYLON_HEIGHT = 0.034    # m, above the fuselage upper surface
# Tail aerodynamic surfaces, measured AFT of the CG.
# Deliberately FORWARD of the tail rotor disc band. At 0.860 m the horizontal
# tail sat inside the 0.653-0.907 m disc sweep and would have flown in its own
# rotor wash in hover -- caught by check(), not by eye.
# V-tail moved right to the back, behind the rotor. The longer arm buys the
# same effectiveness from a SMALLER surface -- tail volume scales with
# area x arm, so 0.620 -> 0.870 m lets the areas drop by the same ratio.
TAIL_SURFACE_ARM = 0.870     # m

# --- V-tail -----------------------------------------------------------------
# Two surfaces instead of three: less wetted area, one fewer junction making
# interference drag, lighter, fewer parts to print. It is what essentially
# every modern fixed-wing VTOL uses, and PX4 allocates it natively
# (CA_SV_CS*_TYPE 7 = Left V-Tail, 8 = Right V-Tail).
#
# A V-tail at dihedral angle GAMMA from horizontal resolves into:
#     effective pitch area = S * cos^2(GAMMA)
#     effective yaw   area = S * sin^2(GAMMA)
# so GAMMA is chosen to reproduce the pitch/yaw split a conventional tail
# would have given, and the required total area follows. Both are DERIVED in
# solve(), not typed in, so they cannot disagree with the geometry.
# Scaled down with the longer arm (0.620 -> 0.870 m) to hold the SAME tail
# volume coefficients from ~29% less area. Less wetted surface, less weight,
# less drag -- bought purely by moving the surfaces aft.
TAIL_PITCH_AREA_REQ = 0.0784  # m^2, effective horizontal area needed
TAIL_YAW_AREA_REQ = 0.0428    # m^2, effective vertical area needed
# 0.150 m forced 0.567 m panels -- a 0.91 m projected tail span on a 2.0 m
# wing, which check() rejected. 0.18 m gives 0.47 m panels and a 0.76 m
# projected span, 38% of wing span, which is where real VTOL V-tails sit.
TAIL_CHORD = 0.180           # m

# --- Control surfaces -------------------------------------------------------
# These lived as bare literals inside gen_sdf.py, which meant the simulated
# hinge and any drawn geometry could disagree without anything noticing. They
# belong here with every other dimension.
#
# The surfaces used to be emitted as plain <box> visuals: grey slabs pasted
# near the trailing edge, floating clear of a wing that is tapered, swept and
# dihedralled. They are now lofted from the actual aerofoil section, hinged on
# the real hinge line. Note this is COSMETIC ONLY -- the Gazebo LiftDrag
# plugins read the joint ANGLE, never the mesh, so the aerodynamics are
# identical either way.
AILERON_SPAN_FRAC = 0.30      # of wing span
AILERON_Y_FRAC = 0.32         # spanwise centre, of wing span
AILERON_CHORD_FRAC = 0.25     # of local chord, hinged at 75% chord
RUDDERVATOR_SPAN_FRAC = 0.55  # of V-tail panel span
RUDDERVATOR_CHORD_FRAC = 0.30 # of tail chord
CONTROL_DEFLECT_MAX = math.radians(30.0)

# --- Servos and electrical load ---------------------------------------------
# SIX servos, not three. Four control surfaces (2 ailerons, 2 ruddervators) plus
# two wing tilts. The tail nacelle is fixed, so it has no tilt servo.
#
# The BOM listed three TILT servos and NO control-surface servos at all, and
# the flight controller line asked for "7 servo outputs" -- both left over from
# the tilting-tail layout. PX4 allocates SIM_GZ_SV_FUNC1..4 to the surfaces and
# 5..6 to the tilts, so the real requirement is 6 servo + 3 motor outputs.
N_SERVO_SURFACE = 4
N_SERVO_TILT = 2
# Control surfaces are far less loaded than the tilt axis: aerodynamic hinge
# moment only, no thrust vector and no nacelle weight hanging off the axis.
SURFACE_SERVO_TORQUE_KGCM = 8.0
# Peak current per servo at stall. Servos do not all stall at once in practice,
# but the 5 V rail has to survive it if they do.
SERVO_STALL_CURRENT_A = 2.5
# A Pixhawk's internal regulator cannot feed six servos. This is the single
# electrical fact that decides whether a separate BEC is needed.
FC_INTERNAL_BEC_MAX_A = 1.5

# --- Nose camera -------------------------------------------------------------
# 1080p forward-looking payload camera in the nose. Sits ahead of the avionics
# bay where it has a clear view below the wing and outside every propeller disc.
CAMERA_ENABLED = True
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30
CAMERA_HFOV = math.radians(78.0)   # typical 1080p module
CAMERA_MASS = 0.045                # kg, module + lens + mount
CAMERA_TILT_DOWN = math.radians(15.0)   # looks slightly down, as survey cams do

# --- Equipment bay -----------------------------------------------------------
# Everything that has to physically fit inside the fuselage, with real module
# dimensions. `x` is the station of the item's CENTRE, in the model frame,
# positive forward of the CG.
#
# This exists so "where do the electronics go" is answered by arithmetic against
# the actual fuselage section rather than by hoping. check() verifies every item
# fits inside the lofted envelope at its station -- a fuselage that is 122 mm
# across at its widest cannot swallow a 100 mm battery at a station where the
# body has narrowed to 60 mm, and until now nothing said so.
#
# (name, x_centre_m, length_mm, width_mm, height_mm, mass_kg)
# ⚠ REARRANGED after the CG was computed rather than assumed. With the battery
# at +20 mm the CG landed 48 mm AFT of the design point -- 46.6% MAC against a
# 20-35% window -- because 1.17 kg of battery was sitting essentially on the CG
# and doing nothing to balance the tail.
#
# solve_cg() puts the battery at +138 mm, which is where it has to be for the
# CG to land where every moment arm in the trim solve assumes it does. Avionics
# moved forward of it; nothing sits between +53 and +223 mm but the pack.
EQUIPMENT = (
    ("Nose camera",      0.418,  30.0, 30.0, 30.0, CAMERA_MASS),
    ("GPS / compass",    0.360,  50.0, 50.0, 16.0, 0.032),
    ("Flight controller", 0.285,  85.0, 45.0, 20.0, 0.100),
    ("Battery 6S",       0.138, 170.0, 70.0, 55.0, MASS_BATTERY),
    ("Airspeed sensor",  0.020,  25.0, 20.0, 15.0, 0.015),
    ("ESC x3",          -0.130,  70.0, 60.0, 20.0, 0.090),
    ("BEC",             -0.210,  40.0, 25.0, 12.0, 0.025),
)

# --- Lateral layout ---------------------------------------------------------
# Wing rotor lateral station, measured from centreline.  This single number
# sets roll authority in hover -- see check_roll_authority().
WING_ROTOR_Y = 0.400         # m
FUSELAGE_HALF_WIDTH = 0.060  # m

# --- Rotors -----------------------------------------------------------------
WING_PROP_DIAMETER = 0.3302  # m, 13 in
# FOLDING prop. Not a detail: a stopped, unfeathered 10 in disc presents about
# 0.004 m^2 of flat plate, worth ~1.0 N against a 3.3 N cruise requirement --
# roughly 30% more cruise drag, straight off the endurance. A folding prop
# streamlines against the pylon and recovers nearly all of it. This is why
# every production aircraft in this layout uses one.
TAIL_PROP_DIAMETER = 0.2540  # m, 10 in, FOLDING
TAIL_PROP_FOLDING = True
# Peak static thrust each motor can produce, from the motor/prop datasheet.
WING_MOTOR_THRUST_MAX = 38.0  # N
TAIL_MOTOR_THRUST_MAX = 25.0  # N

# --- Tilt mechanism ---------------------------------------------------------
# Sign convention is PX4's, taken from src/modules/control_allocator/module.yaml
# in the pinned v1.17.0 tree, NOT invented here:
#
#   "Defines the tilt angle when the servo is at the minimum.
#    An angle of zero means upwards."     -- CA_SV_TL{i}_MINA
#
# So 0 deg = thrust UP (hover) and +90 deg = tilted towards the CA_SV_TL{i}_TD
# azimuth, which we set to 0 ('Towards Front') = cruise. Valid range -90..+90.
#
# This is the opposite of the convention I first assumed. Vectored yaw
# therefore needs travel past vertical in the NEGATIVE direction (nacelle
# leaning aft), not past +90.
TILT_ANGLE_HOVER = math.radians(0.0)
TILT_ANGLE_CRUISE = math.radians(90.0)
# Past-vertical travel for vectored yaw, toward the rear. ArduPilot's
# equivalent knob is Q_TILT_YAW_ANGLE.
TILT_YAW_TRAVEL = math.radians(15.0)
TILT_RATE = math.radians(45.0)  # rad/s, nacelle slew rate

# --- Control allocation targets --------------------------------------------
# Fraction of nominal hover thrust reserved for differential (roll) control.
ROLL_THRUST_MARGIN = 0.30
# Minimum angular accelerations we require in hover. Below these the aircraft
# is not controllable in gusts. Values are conventional VTOL minimums.
MIN_ALPHA_ROLL = 4.0    # rad/s^2
MIN_ALPHA_PITCH = 4.0   # rad/s^2
MIN_ALPHA_YAW = 1.5     # rad/s^2  (yaw is always the weakest axis)

# --- Flight envelope --------------------------------------------------------
V_CRUISE = 19.0              # m/s
TRANSITION_STALL_MARGIN = 1.30   # transition completes at 1.3 x V_stall

# --- Ground clearance -------------------------------------------------------
LANDING_GEAR_HEIGHT = 0.180  # m, from ground to fuselage reference plane
NACELLE_Z_OFFSET = 0.045     # m, rotor plane above fuselage reference plane


# ---------------------------------------------------------------------------
# Tilt nacelle mechanism -- the PRINTED part.
#
# Dimensions here are in MILLIMETRES, unlike everything above, because this is
# the CAD half and mm is what the slicer, the drawing and the fastener specs
# all speak. Anything crossing between the two is converted explicitly, never
# implicitly.
#
# Same no-hardware constraint that governed 02: this cannot be test-fitted, so
# clearance fits only, no press fits, and a printable test coupon ships with it.
# ---------------------------------------------------------------------------

NOZZLE_DIA_MM = 0.4
LAYER_HEIGHT_MM = 0.2

# --- Motor interface --------------------------------------------------------
# 28xx-class outrunner, the usual choice at ~19 N hover thrust.
MOTOR_BOLT_PITCH_MM = 19.0     # square pattern, M3
MOTOR_BOLT_DIA_MM = 3.0
MOTOR_BOSS_DIA_MM = 28.0       # motor base diameter
MOTOR_SHAFT_CLEAR_MM = 8.0     # central bore for shaft + wiring
CRADLE_PLATE_DIA_MM = 38.0     # motor mount plate outside diameter

# --- Tilt axis --------------------------------------------------------------
TILT_SHAFT_DIA_MM = 6.0
BEARING_OD_MM = 13.0           # 686ZZ: 6 x 13 x 5
BEARING_WIDTH_MM = 5.0
BEARING_SEAT_CLEARANCE_MM = 0.15   # radial, on the OD. Clearance, not press.
SHAFT_BORE_CLEARANCE_MM = 0.30     # generous: printed holes shrink

# --- Structure --------------------------------------------------------------
WALL_MM = 2.4                  # 6 perimeters at 0.4 mm
CRADLE_PLATE_MM = 4.0          # motor mount plate thickness
YOKE_ARM_MM = 6.0              # yoke arm thickness
NACELLE_WIDTH_MM = 42.0        # across the yoke arms, inside face to inside face

# --- Boom clamp -------------------------------------------------------------
BOOM_DIA_MM = 16.0             # carbon tube
BOOM_CLAMP_CLEARANCE_MM = 0.20
BOOM_CLAMP_BOLT_DIA_MM = 3.0

# --- Servo ------------------------------------------------------------------
SERVO_STALL_TORQUE_KGCM = 20.0     # datasheet, at operating voltage
SERVO_HORN_RADIUS_MM = 12.0
# Perpendicular offset from the tilt axis to the thrust line. Ideally zero;
# a real build never achieves that, so it is budgeted and checked.
THRUST_AXIS_OFFSET_MM = 5.0
# Offset of the nacelle assembly CG from the tilt axis.
NACELLE_CG_OFFSET_MM = 8.0
SERVO_SAFETY_FACTOR = 2.0

# --- Servo bodies and linkage ------------------------------------------------
# ⚠ Until now the tilt mechanism was a shaft turning in bearings with NOTHING
# driving it: no horn on the cradle, no pushrod, no servo pocket in the yoke.
# The servo was costed in the BOM and sized by torque, but had no geometry.
#
# Standard 20 kg.cm digital metal-gear case, and a 9 g-class case for the
# control surfaces. Both mounted LYING FLAT so the thickest dimension is across
# the smallest one -- a 30 mm tall servo will not stand up inside a 24 mm wing.
TILT_SERVO_L_MM = 40.0
TILT_SERVO_W_MM = 20.0
TILT_SERVO_H_MM = 38.0
SURFACE_SERVO_L_MM = 29.0
SURFACE_SERVO_W_MM = 13.0
SURFACE_SERVO_H_MM = 30.0
# The ruddervators need FAR less than the ailerons: smaller surface, smaller
# chord, and check() puts the hinge load at 1.96 N against the aileron's 9.18 N.
# A 12 g case at 3 kg.cm still carries it with margin, and being smaller is what
# lets the servo sit right beside the ruddervator instead of 170 mm forward in
# the fuselage -- which is the whole point, because pushrod length is where the
# slop comes from.
# Sub-micro case. The servo has to sit at the RUDDERVATOR'S OWN span station
# (see below), and the panel is only 13.5 mm thick out there -- an 11 mm case
# leaves 0.9 mm of skin, a 9 mm case leaves 2.0 mm. Torque is not the binding
# constraint anywhere near here: the hinge moment is 0.0176 N.m and even 1.8
# kg.cm gives 10x.
TAIL_SERVO_L_MM = 20.0
TAIL_SERVO_W_MM = 9.0
TAIL_SERVO_H_MM = 20.0
TAIL_SERVO_TORQUE_KGCM = 1.8
SERVO_MOUNT_CLEARANCE_MM = 0.6      # pocket is this much bigger than the case

# Linkage. The horn on the CRADLE is driven by a pushrod from the servo horn on
# the yoke; both arms are SERVO_HORN_RADIUS_MM so the ratio is 1:1 and the
# servo's own range maps directly onto the nacelle's.
HORN_ARM_THICK_MM = 4.0
HORN_BOSS_DIA_MM = 9.0
PUSHROD_DIA_MM = 2.0                # M2 threaded rod with ball links
PUSHROD_HOLE_DIA_MM = 2.3           # clearance for the ball-link stud

# Chordwise station of the aileron servo bay, as a fraction of local chord aft
# of the leading edge. Chosen forward of the hinge where the section is still
# deep: at 0.55c the wing is 21 mm thick, at the 0.75c hinge only 18 mm, and
# the servo needs 13.6 mm plus skin both sides.
# ⚠ 0.55 -> 0.68. At 0.55 the servo output sat 48 mm ahead of the 0.75 hinge,
# so the pushrod ran 48 mm through open air between two 14 mm horns -- the
# crudest possible linkage, and the most exposed thing on the aircraft after
# the gear. Moving the servo aft shortens the exposed run to ~17 mm.
#
# It cannot go further: the section thins toward the trailing edge and the
# servo needs its 13.6 mm plus skin both sides. 0.68 was tried and REJECTED by
# the "aileron servo fits inside the wing section" invariant -- 15.4 mm of
# section leaves 0.9 mm of skin, which is not a wing, it is a sticker. 0.63
# gives 17.0 mm and 1.7 mm of skin.
SERVO_BAY_CHORD_FRAC = 0.63
# The two ruddervator servos do NOT go in the V-tail. A NACA 0009 panel at
# 180 mm chord is 16.2 mm thick at its thickest, which leaves 1.3 mm of skin
# around a 13.6 mm servo pocket -- not buildable. They live in the aft fuselage
# and drive the ruddervators through pushrods, which is what V-tails normally
# do anyway.
# ⚠ THE SERVO GOES IN THE V-TAIL PANEL, not in the fuselage.
#
# I rejected this twice on the grounds that a NACA 0009 panel is 16.2 mm thick
# and the servo needed 13.6 mm, leaving 1.3 mm of skin. That was true of the
# 29 x 13 x 30 mm case -- and I never re-checked it after the torque analysis
# showed the ruddervator needs 0.18 kg.cm and a 12 g case would do. An 11 mm
# case leaves 1.7 mm of skin, which works.
#
# Mounting it in the panel beside the surface it drives is strictly better:
# the pushrod is ~54 mm and runs INSIDE the fixed panel, exiting only at the
# hinge line. No sleeve, no long external run, no compliance.
# ⚠ 0.34 -> 0.50. At 0.34 the rod spanned 36% of a 152 mm chord = 55 mm, nearly
# twice the aileron's 29 mm, and crossed a wide stretch of fixed panel. Pushing
# the servo aft toward the hinge is what shortens it; the limit is the section
# thinning toward the trailing edge, which the fit check enforces.
# ⚠ SUPERSEDED by TAIL_SERVO_EXTERNAL below. Kept because the fit arithmetic
# still uses it to prove the in-panel option WAS viable -- it was, at 13.0 mm
# of section against a 9.6 mm servo. It is simply not how these are built.
TAIL_SERVO_PANEL_CHORD_FRAC = 0.50   # forward of the 0.70 hinge
# ⚠ 0.16 -> 0.55. THE SERVO MUST SIT AT THE RUDDERVATOR'S OWN SPAN STATION.
# At 0.16 the servo was near the panel root -- but the ruddervator only spans
# 0.275..0.825 of the panel, so the "control horn" landed on the FIXED panel
# inboard of the moving surface and the linkage drove nothing at all.
#
# The aileron got this right by construction: its servo sits at the aileron's
# own mid-span. This is the same rule, applied where I failed to apply it. It
# equals the ruddervator's s_mid (0.55 * panel span) and is now enforced by the
# "tail servo is at the ruddervator's own span station" invariant.
# 0.55 -> 0.32: still comfortably inside the ruddervator's 27.5..82.5% span,
# but further inboard where the panel chord is larger, which buys back the
# section thickness the aft move costs.
TAIL_SERVO_PANEL_SPAN_FRAC = 0.32

# --- How the ruddervator servos are ACTUALLY mounted -------------------------
# ⚠ Not buried inside the panel. On real V-tails of this size the two servos
# are SURFACE-MOUNTED on the fuselage at the tail root -- screwed to the skin,
# body proud, output arm facing aft -- with short pushrods running out to horns
# near the inboard end of each ruddervator.
#
# That is worth following over the in-panel version even though the in-panel
# one fits, because it is serviceable: a servo screwed to the outside can be
# replaced at the field, and a servo laminated inside a 13 mm tail panel
# cannot. The cost is a longer pushrod, which the buckling check re-verifies
# rather than my assuming it is still fine.
TAIL_SERVO_EXTERNAL = True
TAIL_SERVO_MOUNT_X = -0.885     # m, station on the fuselage tail root
TAIL_HORN_SPAN_FRAC = 0.30      # where the horn lands, of panel span

# --- Tail motor mount --------------------------------------------------------
# ⚠ The tail station was being drawn with the full TILTING hardware -- cradle,
# yoke, bearings, shaft -- left over from when the tail rotor tilted. It does
# not tilt (TAIL_TILTS = False): it is bolted to the top of the pylon and stays
# vertical. All that mechanism is mass, cost, print time and drag for a degree
# of freedom the aircraft does not use.
#
# A fixed rotor needs a motor plate and a way to attach it to the pylon. That
# is all.
# 11 -> 8 mm. The plate has to stay 4 mm for M3 threads (checked), so the
# saddle is what shrinks: 7 -> 4 mm of engagement onto the pylon top. Bonded
# rather than clamped, so the socket only has to locate it, not grip it.
TAIL_MOUNT_HEIGHT_MM = 8.0      # plate + saddle, against 42 mm of tilt hardware
TAIL_MOUNT_SADDLE_MM = 4.0      # depth of the socket that grips the pylon top
TAIL_PYLON_TOP_CHORD_MM = 82.0  # matches the pylon loft's upper section
TAIL_PYLON_TOP_THICK_MM = 12.0

# --- Control linkage visibility ----------------------------------------------
# A servo buried in a closed pocket is invisible and, more to the point, cannot
# be installed or reached. Real bays break through the lower skin and are
# closed with a cover; the horn projects through a slot and a pushrod runs aft
# to a horn on the control surface.
SERVO_BAY_OPENS_THROUGH = True
# 14 -> 9 mm. Horn height sets both the exposed frontal area and the linkage
# ratio. Shorter horn = less drag and less compliance, at the cost of higher
# rod force for the same hinge moment -- which the buckling check re-verifies
# rather than my asserting it is still fine.
CONTROL_HORN_H_MM = 9.0         # how far the horn stands off the surface
CONTROL_HORN_T_MM = 2.4
# A real control horn is a flat plate that BOLTS THROUGH the surface, with a
# blade carrying two or three holes so the linkage ratio can be trimmed at
# assembly. Modelling it as a bare tube hid the two things that matter on a
# build: the bolt flange has to land on solid material, and the hole you pick
# sets the deflection per unit of servo travel.
HORN_BASE_L_MM = 18.0      # flange, along the chord
HORN_BASE_W_MM = 9.0       # flange, across
HORN_BASE_T_MM = 2.0
HORN_BLADE_L_MM = 11.0     # blade footprint along the chord
HORN_HOLE_DIA_MM = 2.2     # clearance for an M2 clevis pin
HORN_HOLES = (0.55, 0.78, 0.96)   # hole heights as a fraction of blade height

# --- Pushrods ----------------------------------------------------------------
# A pushrod works in PUSH and PULL. A wire only pulls -- it buckles instantly in
# compression -- so a bare wire needs either a closed loop with two wires per
# surface, or an outer sleeve to keep it from bowing. The sleeved version
# (Bowden) is what gliders use for exactly the reason of alignment: the inner
# rod is free to follow a curved route and the ends do not have to line up.
#
# Straight, short run  -> solid rod, stiffer and simpler   (ailerons)
# Long or curved run   -> rod inside a sleeve              (ruddervators)
PUSHROD_E_PA = 70.0e9           # pultruded carbon, conservative
AILERON_ROD_DIA_MM = 2.0
RUDDERVATOR_ROD_DIA_MM = 2.0
RUDDERVATOR_ROD_SLEEVED = True  # unsupported length is the sleeve pitch, not the run
SLEEVE_SUPPORT_PITCH_MM = 120.0  # sleeve is bonded to structure this often
HINGE_MOMENT_COEFF = 0.15       # Ch at full deflection, thin symmetric section

# --- Primary structure: carbon rods ------------------------------------------
# Two longerons run nose to tail and carry fuselage bending; two spars run
# inside each wing and carry the wing bending plus the nacelle loads. This is
# what makes a printed airframe survivable -- the printed shells become
# fairings and load paths run in carbon.
# ⚠ A PAIR of longerons cannot run the whole length. check() caught it on the
# first run: two 12 mm rods at 46 mm spacing need 58 mm of width, and the tail
# boom is 33 mm across. They run through the wide forward body where the
# equipment bay needs the bending stiffness, then converge into a SINGLE tail
# boom tube aft of the wing -- which is what full-size aircraft do for the same
# reason.
LONGERON_DIA_MM = 12.0
LONGERON_SPACING_MM = 46.0      # lateral separation, port and starboard
LONGERON_Z_MM = -6.0            # below the fuselage centreline, under the bay
LONGERON_AFT_X = -0.250         # m, where the pair ends and the boom takes over
TAILBOOM_DIA_MM = 16.0          # single central tube, carries the V-tail loads
WING_SPAR_DIA_MM = 12.0
WING_SPAR_CHORD_FRAC = 0.30     # at max section thickness
WING_SPAR_REAR_CHORD_FRAC = 0.62  # rear spar, carries the aileron hinge line
WING_SPAR_SPAN_FRAC = 0.92      # how far out the spar runs, of semi-span

# --- Formers and joints ------------------------------------------------------
# Printed formers threaded onto the longerons. These ARE the internal mounts:
# the equipment straps to them, and they carry the skin's shape. Bonded to the
# rods, they turn two tubes and a shell into a semi-monocoque.
FORMER_THICK_MM = 3.0
FORMER_RIM_MM = 6.0             # material left around the rim after lightening
# (name, x station in m) -- chosen at the things that need carrying.
FORMERS = (
    ("F1 nose / camera",     0.418),
    ("F2 avionics front",    0.300),
    ("F3 avionics rear",     0.190),
    ("F4 battery front",     0.105),
    ("F5 spar box",          0.008),
    ("F6 battery rear",     -0.070),
    ("F7 ESC bay",          -0.150),
    ("F8 boom junction",    -0.260),
)

# Joints. NOT snap fits: a printed cantilever snap in PETG or ASA creeps and
# fatigues, and on a 4.8 kg aircraft it fails without warning. The spar carries
# bending and shear across the joint; the pin only ever sees shear, and it is
# in double shear at that. Tool-free, and nothing structural depends on plastic
# in tension.
JOINT_PIN_DIA_MM = 4.0
JOINT_PIN_SHEAR_MPA = 210.0     # stainless dowel, conservative
JOINT_SPAR_ENGAGE_MM = 90.0     # how far the panel sleeves onto the spar


# ---------------------------------------------------------------------------
# Derived geometry
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Derived:
    """Everything computed from the inputs above. Never edit by hand."""

    # Mass / areas
    weight: float
    wing_area: float
    wing_aspect_ratio: float
    wing_loading: float
    mac: float

    # Longitudinal stations, positive forward of CG
    wing_rotor_arm: float
    tail_rotor_arm: float

    # Hover trim
    thrust_wing_each: float
    thrust_tail: float
    tail_lift_fraction: float

    # Disc loading
    disc_loading_wing: float
    disc_loading_tail: float

    # Inertia estimates
    ixx: float
    iyy: float
    izz: float

    # Authority
    alpha_roll: float
    alpha_pitch: float
    alpha_yaw: float

    # Envelope
    v_stall: float
    v_transition: float

    # Unpowered glide, derived from the drag polar rather than asserted.
    l_over_d_max: float      # -, best achievable, at CL = sqrt(CD0 pi e AR)
    v_best_glide: float      # m/s, the speed that achieves it
    glide_angle: float       # rad, below horizontal
    sink_rate: float         # m/s, descent rate in that glide

    # Cruise
    thrust_required_cruise: float

    # Aerodynamic coefficients, DERIVED from the airfoil section rather than
    # asserted. These are what the Gazebo LiftDrag plugins are fed.
    cl_alpha: float          # 1/rad, finite wing
    cl_max: float            # -, finite wing
    alpha_stall: float       # rad, finite wing
    cd0: float               # -, total zero-lift drag incl. parasites
    l_over_d_cruise: float

    # V-tail, derived from the required pitch/yaw effectiveness
    tail_dihedral: float     # rad, from horizontal
    tail_area_total: float   # m^2, both panels
    tail_panel_span: float   # m, one panel root to tip
    tail_semi_span_h: float  # m, horizontal projection of one panel

    # Winglets
    ar_effective: float      # -, AR after the winglet span-efficiency credit
    winglet_height: float    # m
    fineness_ratio: float    # -, fuselage length / max diameter


def _wing_area() -> float:
    return WING_SPAN * WING_CHORD


def _inertia_estimates() -> tuple[float, float, float]:
    """Crude but honest rigid-body inertia estimates about the CG.

    Wing panels are treated as rods about the roll axis, point masses are
    treated as point masses. This is good to maybe +/-25%, which is enough to
    decide whether a control axis has authority or not -- the check thresholds
    below carry margin well beyond that.
    """
    # Roll (x): dominated by the wing span and the wing nacelles.
    wing_mass = 2.0 * MASS_WING_PANEL
    ixx = wing_mass * WING_SPAN ** 2 / 12.0
    ixx += 2.0 * MASS_NACELLE_WING * WING_ROTOR_Y ** 2

    # Pitch (y): dominated by the fuselage length and the tail nacelle arm.
    # Battery, avionics and payload sit at the CG by design, so they carry
    # essentially no pitch inertia and are deliberately excluded here.
    fuse_len = WING_ROTOR_AHEAD_OF_LE + WING_CHORD + TAIL_SURFACE_ARM
    iyy = MASS_FUSELAGE * fuse_len ** 2 / 12.0
    iyy += MASS_NACELLE_TAIL * TAIL_ROTOR_ARM ** 2
    iyy += MASS_TAIL_ASSY * TAIL_SURFACE_ARM ** 2
    iyy += 2.0 * MASS_NACELLE_WING * _wing_rotor_arm() ** 2

    # Yaw (z): for a roughly planar aircraft, Izz ~ Ixx + Iyy.
    izz = ixx + iyy
    return ixx, iyy, izz


def fuselage_nose_x() -> float:
    """Model-frame x of the fuselage nose."""
    return CG_MAC_FRACTION * WING_CHORD + 0.30 * FUSELAGE_LENGTH


def fuselage_half_height_at(x: float) -> float:
    """Fuselage half-height at a model-frame station, by interpolation.

    Used to sit the tail pylon on the actual fuselage surface instead of at a
    guessed height. Guessing is how the nacelles ended up floating 28 mm clear
    of the wing earlier.
    """
    frac = (fuselage_nose_x() - x) / FUSELAGE_LENGTH
    frac = min(max(frac, 0.0), 1.0)
    pts = FUSELAGE_STATIONS
    for i in range(len(pts) - 1):
        f0, h0, _ = pts[i]
        f1, h1, _ = pts[i + 1]
        if f0 <= frac <= f1:
            t = (frac - f0) / (f1 - f0) if f1 > f0 else 0.0
            return h0 + t * (h1 - h0)
    return pts[-1][1]


# ---------------------------------------------------------------------------
# Mass ledger and CG
#
# ⚠ CG_MAC_FRACTION is an INPUT: the design ASSERTS the CG sits at 28% MAC and
# every moment arm in the trim solve is measured from there. Nothing ever
# computed where the CG actually lands given where the mass is. That is the
# same class of gap as "fuselage is long enough to carry the tail" -- a number
# everything depends on, that nothing checks.
#
# This ledger fixes it. Every item is listed with its station, the CG follows
# by arithmetic, and the BATTERY POSITION is SOLVED so the CG lands on the
# design point rather than being placed by eye and hoped over.
# ---------------------------------------------------------------------------

# Densities, kg/m^3. Printed parts are quoted at realistic infill, not solid.
RHO_PETG_PRINTED = 800.0     # ~1270 solid at ~55% effective infill + walls
RHO_CARBON_TUBE = 1550.0


def mass_ledger() -> list[tuple[str, float, float]]:
    """(name, mass kg, x station m) for everything except the battery.

    x is positive FORWARD of the design CG, matching the rest of the module.
    """
    d = solve()
    a = _wing_rotor_arm()
    qx = CG_MAC_FRACTION * WING_CHORD - 0.25 * WING_CHORD

    items: list[tuple[str, float, float]] = [
        # Structure. The wing's mass centroid sits a little aft of the quarter
        # chord; the fuselage shell's is forward of mid-body because the nose
        # section is much fuller than the tail boom.
        ("wing panels (2)", 2 * MASS_WING_PANEL, qx - 0.02),
        ("fuselage shell", MASS_FUSELAGE, -0.06),
        ("tail surfaces", MASS_TAIL_ASSY, -TAIL_SURFACE_ARM),
        # Propulsion
        ("wing nacelles (2)", 2 * MASS_NACELLE_WING, a),
        ("tail nacelle", MASS_NACELLE_TAIL, -TAIL_ROTOR_ARM),
        # Servos, at the stations the CAD actually puts them
        ("aileron servos (2)", 2 * 0.022, qx - SERVO_BAY_CHORD_FRAC * WING_CHORD),
        ("ruddervator servos (2)", 2 * 0.009, -TAIL_SURFACE_ARM - 0.03),
        ("tilt servos (2)", 2 * 0.060, a - 0.02),
    ]
    # Avionics and payload, from the equipment bay table so the two cannot
    # disagree about where anything is.
    for name, x_c, _ln, _wd, _ht, m in EQUIPMENT:
        if name.startswith("Battery"):
            continue
        items.append((name, m, x_c))
    return items


def solve_cg() -> dict:
    """Total mass, CG, and the battery station that puts the CG on target.

    Sum(m_i x_i) + m_batt x_batt = 0   ->   x_batt = -Sum(m_i x_i) / m_batt
    since the design CG is the origin by construction.
    """
    items = mass_ledger()
    dry_mass = sum(m for _n, m, _x in items)
    dry_moment = sum(m * x for _n, m, x in items)
    x_batt = -dry_moment / MASS_BATTERY
    total = dry_mass + MASS_BATTERY
    # Where the CG actually lands if the battery is placed at its EQUIPMENT
    # station instead of the solved one.
    x_nominal = next(x for n, x, *_ in
                     ((e[0], e[1]) + tuple(e[2:]) for e in EQUIPMENT)
                     if n.startswith("Battery"))
    cg_nominal = (dry_moment + MASS_BATTERY * x_nominal) / total
    return dict(dry_mass=dry_mass, total_mass=total, x_batt=x_batt,
                x_batt_nominal=x_nominal, cg_nominal=cg_nominal,
                dry_moment=dry_moment)


HOVER_HEADROOM = 1.8       # thrust available / thrust at hover, the design rule


def payload_capacity() -> dict:
    """How much more can it lift, and what stops it lifting more?

    Every limit is solved for the WEIGHT at which it binds, then the smallest
    wins. Reporting only the number would hide the useful part: which
    constraint is actually the wall, because that is what you would change.

    The hover trim ratio is fixed by geometry, not by loading:
        T_wing_total = W c / (a + c)     T_tail = W a / (a + c)
    so each motor's share of W is a constant and the thrust limits become
    straight limits on W.
    """
    a = _wing_rotor_arm()
    c = TAIL_ROTOR_ARM
    area = _wing_area()

    share_wing_each = (c / (a + c)) / 2.0     # fraction of W per wing motor
    share_tail = a / (a + c)

    limits: list[tuple[str, float]] = []

    # Motor thrust, at the design's own headroom rule.
    limits.append(("wing motor headroom",
                   WING_MOTOR_THRUST_MAX / HOVER_HEADROOM / share_wing_each))
    limits.append(("tail motor headroom",
                   TAIL_MOTOR_THRUST_MAX / HOVER_HEADROOM / share_tail))

    # Transition must stay below cruise with the 10% margin check() enforces.
    # v_trans = 1.3 sqrt(2W / (rho S CL_max)) <= V_CRUISE / 1.10
    v_trans_max = V_CRUISE / 1.10
    cl_max = 0.90 * WING_CL_MAX_2D
    limits.append(("transition speed vs cruise",
                   (v_trans_max / TRANSITION_STALL_MARGIN) ** 2
                   * RHO * area * cl_max / 2.0))

    # Wing loading ceiling from the sanity check.
    limits.append(("wing loading ceiling", 18.0 * area * G))

    # Disc loading ceiling on the wing rotors.
    disc_wing = math.pi * (WING_PROP_DIAMETER / 2.0) ** 2
    limits.append(("wing disc loading", 400.0 * disc_wing / share_wing_each))

    binding, w_max = min(limits, key=lambda t: t[1])
    mtow_max = w_max / G
    return dict(limits=sorted(limits, key=lambda t: t[1]),
                binding=binding, mtow_max=mtow_max,
                payload_kg=mtow_max - MASS_TOTAL,
                current_mtow=MASS_TOTAL)


def camera_x() -> float:
    """Model-frame x of the nose camera.

    Sits 60 mm aft of the nose tip so there is fuselage section around it to
    mount to, rather than being cantilevered off the very point.
    """
    return fuselage_nose_x() - 0.060


def naca_yt_yc(code: str, x: float) -> tuple[float, float]:
    """Half-thickness and camber of a 4-digit section at x/c, in chord units.

    Lives here, not in gen_geometry, because BOTH the lofted mesh and the SDF
    joint placement need it. Duplicating it was how the box control surfaces
    ended up floating clear of a wing they were supposed to be hinged to.
    """
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0
    yt = 5.0 * t * (
        0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
        + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    if m > 0.0 and p > 0.0:
        if x < p:
            yc = m / p ** 2 * (2.0 * p * x - x ** 2)
        else:
            yc = m / (1.0 - p) ** 2 * ((1.0 - 2.0 * p) + 2.0 * p * x - x ** 2)
    else:
        yc = 0.0
    return yt, yc


def wing_chords() -> tuple[float, float]:
    """Root and tip chord from the MAC and taper."""
    lam = WING_TAPER
    c_root = WING_CHORD * 3.0 * (1.0 + lam) / (2.0 * (1.0 + lam + lam ** 2))
    return c_root, c_root * lam


def wing_station(y: float) -> dict:
    """Chord, quarter-chord x, z and twist at a spanwise station.

    Frame is the WING's own: root quarter-chord at the origin, +x forward.
    Downstream consumers add quarter_x to reach the model frame.
    """
    c_root, c_tip = wing_chords()
    f = y / (WING_SPAN / 2.0)
    return dict(
        chord=c_root + (c_tip - c_root) * f,
        x_qc=-y * math.tan(WING_LE_SWEEP),
        z=y * math.tan(WING_DIHEDRAL),
        twist=WING_TWIST_TIP * f,
    )


def aileron_geometry() -> dict:
    """Span limits and mid-span hinge point of one aileron, in the wing frame."""
    y_mid = AILERON_Y_FRAC * WING_SPAN
    span = AILERON_SPAN_FRAC * WING_SPAN
    hinge = 1.0 - AILERON_CHORD_FRAC
    st = wing_station(y_mid)
    _, yc_h = naca_yt_yc(WING_NACA, hinge)
    return dict(
        y0=y_mid - span / 2.0, y1=y_mid + span / 2.0, y_mid=y_mid,
        span=span, hinge=hinge,
        x=st["x_qc"] + (0.25 - hinge) * st["chord"],
        z=st["z"] + yc_h * st["chord"],
    )


def ruddervator_geometry(sign: float) -> dict:
    """Mid-span hinge point and hinge axis of one ruddervator, model frame.

    build_tail() roots the panel at x = -TAIL_SURFACE_ARM, z = 0.010, running
    outboard along (0, cos gamma, sin gamma). The moving surface hinges about
    that same panel axis -- deflected together they are an elevator,
    differentially a rudder.
    """
    d = solve()
    gam = d.tail_dihedral
    hinge = 1.0 - RUDDERVATOR_CHORD_FRAC
    uy, uz = sign * math.cos(gam), math.sin(gam)
    s_mid = 0.55 * d.tail_panel_span
    return dict(
        s_mid=s_mid, span=RUDDERVATOR_SPAN_FRAC * d.tail_panel_span,
        hinge=hinge, axis=(0.0, uy, uz),
        x=-TAIL_SURFACE_ARM + (0.25 - hinge) * TAIL_CHORD,
        y=s_mid * uy, z=0.010 + s_mid * uz,
    )


def fuselage_half_width_at(x: float) -> float:
    """Fuselage half-WIDTH at a model-frame station, by interpolation.

    The half-height twin of this existed; width did not, which is part of why
    the V-tail could be rooted on an 8.5 mm wide stub without anything
    complaining. A tail bolts to the sides of the body, so width is the
    dimension that matters for that joint.
    """
    frac = (fuselage_nose_x() - x) / FUSELAGE_LENGTH
    frac = min(max(frac, 0.0), 1.0)
    pts = FUSELAGE_STATIONS
    for i in range(len(pts) - 1):
        f0, _, w0 = pts[i]
        f1, _, w1 = pts[i + 1]
        if f0 <= frac <= f1:
            t = (frac - f0) / (f1 - f0) if f1 > f0 else 0.0
            return w0 + t * (w1 - w0)
    return pts[-1][2]


def tail_rotor_z() -> float:
    """Height of the tail rotor hub: fuselage surface plus pylon."""
    return fuselage_half_height_at(-TAIL_ROTOR_ARM) + TAIL_PYLON_HEIGHT


def base_link_inertia() -> tuple[float, float, float, float]:
    """Inertia for base_link ALONE, plus its mass.

    The three nacelles are separate SDF links placed at their own offsets, so
    Gazebo computes and adds their parallel-axis contribution itself. If
    base_link carried the whole-aircraft inertia the nacelle terms would be
    counted twice and the model would be sluggish in a way that looks like a
    controller tuning problem. Subtract them here.
    """
    ixx, iyy, izz = _inertia_estimates()

    nacelle_ixx = 2.0 * MASS_NACELLE_WING * WING_ROTOR_Y ** 2
    nacelle_iyy = (
        MASS_NACELLE_TAIL * TAIL_ROTOR_ARM ** 2
        + 2.0 * MASS_NACELLE_WING * _wing_rotor_arm() ** 2
    )
    nacelle_izz = nacelle_ixx + nacelle_iyy

    mass = MASS_TOTAL - 2 * MASS_NACELLE_WING - MASS_NACELLE_TAIL
    return mass, ixx - nacelle_ixx, iyy - nacelle_iyy, izz - nacelle_izz


def _wing_rotor_arm() -> float:
    """Distance from CG forward to the wing rotor plane."""
    cg_aft_of_le = CG_MAC_FRACTION * WING_CHORD
    return cg_aft_of_le + WING_ROTOR_AHEAD_OF_LE


def solve() -> Derived:
    """Solve hover trim and derive every dependent quantity."""
    weight = MASS_TOTAL * G
    area = _wing_area()
    mac = WING_CHORD                       # rectangular planform
    a = _wing_rotor_arm()                  # forward of CG
    c = TAIL_ROTOR_ARM                     # aft of CG

    # --- Hover trim ---------------------------------------------------------
    # Two equations, two unknowns:
    #   vertical:  T_wing_total + T_tail = W
    #   pitch:     T_wing_total * a      = T_tail * c
    # The wing rotors sit AHEAD of the CG and the tail rotor BEHIND it, so
    # their moments oppose and the aircraft can be trimmed. This is the single
    # thing the two-rear-motor layout could not do.
    thrust_wing_total = weight * c / (a + c)
    thrust_tail = weight * a / (a + c)

    # --- Rotor discs --------------------------------------------------------
    disc_wing = math.pi * (WING_PROP_DIAMETER / 2.0) ** 2
    disc_tail = math.pi * (TAIL_PROP_DIAMETER / 2.0) ** 2

    ixx, iyy, izz = _inertia_estimates()

    # --- Control authority in hover ----------------------------------------
    # Roll: differential thrust across the wing pair.
    t_wing_each = thrust_wing_total / 2.0
    d_thrust = ROLL_THRUST_MARGIN * t_wing_each
    alpha_roll = (2.0 * d_thrust * WING_ROTOR_Y) / ixx

    # Pitch: modulate the tail rotor against the wing pair. The binding limit
    # is thrust-DOWN, because the tail rotor can only go to zero.
    pitch_moment = thrust_tail * c
    alpha_pitch = pitch_moment / iyy

    # Yaw: differential TILT of the wing pair only.
    #
    # NOTE: the tail nacelle tilts fore/aft (about y) so that it becomes a
    # pusher in cruise. A force in the x-z plane on the centreline produces NO
    # moment about z, so the tail contributes nothing to yaw. Yaw authority is
    # single-path, from wing vectoring alone. This is ArduPilot's
    # Q_TILT_TYPE=2 "vectored yaw".
    yaw_force_each = t_wing_each * math.sin(TILT_YAW_TRAVEL)
    alpha_yaw = (2.0 * yaw_force_each * WING_ROTOR_Y) / izz

    # --- Aerodynamics, derived from the NACA section ------------------------
    ar = WING_SPAN ** 2 / area

    # Finite-wing lift-curve slope from the 2-D section (Prandtl):
    #     CL_alpha = a0 / (1 + a0 / (pi e AR))
    # The old hand-entered 5.20 /rad was optimistic; this gives ~4.74 for our
    # AR, which is the number the simulator should actually see.
    cl_alpha = WING_CL_ALPHA_2D / (
        1.0 + WING_CL_ALPHA_2D / (math.pi * OSWALD_E * ar))

    # A finite wing reaches less than its section maximum.
    cl_max = 0.90 * WING_CL_MAX_2D

    # Stall angle follows from the slope and the zero-lift angle, so it cannot
    # disagree with them.
    alpha_stall = WING_ALPHA_ZERO_LIFT + cl_max / cl_alpha

    # Zero-lift drag = section minimum + everything that is not the wing,
    # referred to wing area.
    cd0 = WING_CD_MIN + PARASITE_AREA / area

    # --- V-tail geometry ----------------------------------------------------
    # tan^2(GAMMA) = yaw_area / pitch_area, then S = pitch_area / cos^2(GAMMA).
    tail_dihedral = math.atan(math.sqrt(TAIL_YAW_AREA_REQ / TAIL_PITCH_AREA_REQ))
    tail_area_total = TAIL_PITCH_AREA_REQ / math.cos(tail_dihedral) ** 2
    tail_panel_span = (tail_area_total / 2.0) / TAIL_CHORD
    tail_semi_span_h = tail_panel_span * math.cos(tail_dihedral)

    # --- Envelope -----------------------------------------------------------
    v_stall = math.sqrt(2.0 * weight / (RHO * area * cl_max))
    v_transition = TRANSITION_STALL_MARGIN * v_stall

    # --- Winglet credit -----------------------------------------------------
    # Standard span-efficiency correction: AR_eff = AR * (1 + 1.9 * h/b).
    # ESTIMATE, not a computed result -- it is a published rule of thumb, and
    # the only aerodynamic number here that is not derived from the section.
    winglet_h = WINGLET_HEIGHT_FRAC * (WING_SPAN / 2.0)
    ar_eff = ar * (1.0 + 1.9 * winglet_h / WING_SPAN)

    # Fuselage fineness, from the station table rather than a separate number.
    max_half = max(max(h, w) for _, h, w in FUSELAGE_STATIONS)
    fineness = FUSELAGE_LENGTH / (2.0 * max_half)

    # Cruise thrust: L = W fixes CL; drag from the polar, using the winglet-
    # corrected aspect ratio for the induced term only.
    cl_cruise = 2.0 * weight / (RHO * V_CRUISE ** 2 * area)
    cd = cd0 + cl_cruise ** 2 / (math.pi * OSWALD_E * ar_eff)
    thrust_cruise = 0.5 * RHO * V_CRUISE ** 2 * area * cd
    l_over_d = cl_cruise / cd

    # --- Unpowered glide ----------------------------------------------------
    # For a parabolic polar CD = CD0 + CL^2/(pi e AR), L/D peaks where the
    # induced and parasite terms are EQUAL:
    #     CL_bestLD = sqrt(CD0 pi e AR)
    #     (L/D)_max = 0.5 sqrt(pi e AR / CD0)
    # Everything below follows from that, so the glide the aircraft is asked to
    # fly is the glide its own polar predicts -- which is what makes
    # verify_glide.sh a TEST rather than a demonstration. Note this is the
    # best-case L/D, reached at v_best_glide, and is necessarily higher than the
    # cruise L/D at V_CRUISE, which is off the best-glide point.
    k = math.pi * OSWALD_E * ar_eff
    cl_best = math.sqrt(cd0 * k)
    ld_max = 0.5 * math.sqrt(k / cd0)
    v_best = math.sqrt(2.0 * weight / (RHO * area * cl_best))
    gamma_glide = math.atan(1.0 / ld_max)
    sink = v_best * math.sin(gamma_glide)

    return Derived(
        weight=weight,
        wing_area=area,
        wing_aspect_ratio=ar,
        wing_loading=MASS_TOTAL / area,
        mac=mac,
        wing_rotor_arm=a,
        tail_rotor_arm=c,
        thrust_wing_each=t_wing_each,
        thrust_tail=thrust_tail,
        tail_lift_fraction=thrust_tail / weight,
        disc_loading_wing=t_wing_each / disc_wing,
        disc_loading_tail=thrust_tail / disc_tail,
        ixx=ixx,
        iyy=iyy,
        izz=izz,
        alpha_roll=alpha_roll,
        alpha_pitch=alpha_pitch,
        alpha_yaw=alpha_yaw,
        v_stall=v_stall,
        v_transition=v_transition,
        l_over_d_max=ld_max,
        v_best_glide=v_best,
        glide_angle=gamma_glide,
        sink_rate=sink,
        thrust_required_cruise=thrust_cruise,
        cl_alpha=cl_alpha,
        cl_max=cl_max,
        alpha_stall=alpha_stall,
        cd0=cd0,
        l_over_d_cruise=l_over_d,
        tail_dihedral=tail_dihedral,
        tail_area_total=tail_area_total,
        tail_panel_span=tail_panel_span,
        tail_semi_span_h=tail_semi_span_h,
        ar_effective=ar_eff,
        winglet_height=winglet_h,
        fineness_ratio=fineness,
    )


# ---------------------------------------------------------------------------
# Design invariants
# ---------------------------------------------------------------------------

class DesignError(ValueError):
    """Raised when the parameter set describes an aircraft that cannot fly."""


def check(verbose: bool = False) -> list[str]:
    """Run every design invariant. Raises DesignError on the first failure.

    Returns the list of checks that passed, so callers can print a count the
    way 01-nav2-deck's verify scripts do.
    """
    d = solve()
    passed: list[str] = []

    def ok(name: str, condition: bool, detail: str) -> None:
        if not condition:
            raise DesignError(f"{name}: {detail}")
        passed.append(f"{name}: {detail}")

    # --- Mass budget --------------------------------------------------------
    accounted = (
        2 * MASS_WING_PANEL
        + MASS_CENTRAL
        + MASS_TAIL_ASSY
        + 2 * MASS_NACELLE_WING
        + MASS_NACELLE_TAIL
    )
    ok(
        "mass budget closes",
        abs(accounted - MASS_TOTAL) < 0.05,
        f"components {accounted:.3f} kg vs MTOW {MASS_TOTAL:.3f} kg",
    )

    # --- Mass ledger and CG -------------------------------------------------
    # THE GAP THIS CLOSES: CG_MAC_FRACTION is an input, and every moment arm in
    # the trim solve is measured from a CG that nothing ever computed. With the
    # battery on the CG the real CG sat 48 mm aft -- 46.6% MAC against a 20-35%
    # window -- and the whole hover trim was solved about a point the aircraft
    # does not balance on.
    cg = solve_cg()
    ok(
        "declared MTOW matches the mass ledger",
        abs(cg["total_mass"] - MASS_TOTAL) <= 0.06,
        f"ledger {cg['total_mass']:.3f} kg vs declared {MASS_TOTAL:.3f} kg "
        f"({len(mass_ledger())} items + battery)",
    )
    cg_err = cg["cg_nominal"]
    ok(
        "CG lands on the design point where the battery is actually placed",
        abs(cg_err) <= 0.010,
        f"CG at {cg_err * 1000:+.1f} mm "
        f"({cg_err / WING_CHORD * 100:+.1f}% MAC from design), battery at "
        f"{cg['x_batt_nominal'] * 1000:+.0f} mm "
        f"(solve wants {cg['x_batt'] * 1000:+.0f} mm)",
    )
    # The battery is the only item heavy enough to trim the CG, so its travel
    # is the design's whole CG margin. Worth knowing how much there is.
    cg_per_10mm = MASS_BATTERY * 0.010 / cg["total_mass"]
    ok(
        "battery has authority over the CG",
        cg_per_10mm >= 0.0015,
        f"10 mm of battery travel moves the CG "
        f"{cg_per_10mm * 1000:.1f} mm ({cg_per_10mm / WING_CHORD * 100:.1f}% MAC)",
    )

    # --- Longitudinal layout ------------------------------------------------
    # THE invariant. Rotors must straddle the CG or pitch is untrimmable.
    ok(
        "rotors straddle the CG",
        d.wing_rotor_arm > 0 and d.tail_rotor_arm > 0,
        f"wing +{d.wing_rotor_arm:.3f} m fwd, tail -{d.tail_rotor_arm:.3f} m aft",
    )
    ok(
        "CG inside the fixed-wing window",
        0.20 <= CG_MAC_FRACTION <= 0.35,
        f"CG at {CG_MAC_FRACTION * 100:.1f}% MAC (need 20-35%)",
    )
    ok(
        "tail rotor is aft of the wing rotors",
        TAIL_ROTOR_ARM > d.wing_rotor_arm,
        f"{TAIL_ROTOR_ARM:.3f} m aft vs {d.wing_rotor_arm:.3f} m fwd",
    )

    # --- Hover trim ---------------------------------------------------------
    ok(
        "hover trim has a positive solution",
        d.thrust_wing_each > 0 and d.thrust_tail > 0,
        f"wing {d.thrust_wing_each:.2f} N each, tail {d.thrust_tail:.2f} N",
    )
    ok(
        "trim residual is zero",
        abs(2 * d.thrust_wing_each * d.wing_rotor_arm
            - d.thrust_tail * d.tail_rotor_arm) < 1e-9,
        "pitch moments balance to < 1e-9 N.m",
    )
    ok(
        "vertical equilibrium",
        abs(2 * d.thrust_wing_each + d.thrust_tail - d.weight) < 1e-9,
        f"sum of thrust = weight = {d.weight:.3f} N",
    )
    ok(
        "tail carries a sane share of hover lift",
        0.08 <= d.tail_lift_fraction <= 0.35,
        f"tail carries {d.tail_lift_fraction * 100:.1f}% of lift (need 8-35%)",
    )

    # --- Motor sizing -------------------------------------------------------
    ok(
        "wing motors have hover headroom",
        d.thrust_wing_each * 1.8 <= WING_MOTOR_THRUST_MAX,
        f"need {d.thrust_wing_each * 1.8:.1f} N at 1.8x hover, "
        f"have {WING_MOTOR_THRUST_MAX:.1f} N",
    )
    ok(
        "tail motor has hover headroom",
        d.thrust_tail * 1.8 <= TAIL_MOTOR_THRUST_MAX,
        f"need {d.thrust_tail * 1.8:.1f} N at 1.8x hover, "
        f"have {TAIL_MOTOR_THRUST_MAX:.1f} N",
    )
    ok(
        "cruise thrust is available with the tail as pusher",
        d.thrust_required_cruise
        <= 2 * WING_MOTOR_THRUST_MAX + TAIL_MOTOR_THRUST_MAX,
        f"cruise needs {d.thrust_required_cruise:.1f} N, "
        f"three rotors give {2 * WING_MOTOR_THRUST_MAX + TAIL_MOTOR_THRUST_MAX:.1f} N",
    )

    # --- Prop clearance -----------------------------------------------------
    r_wing = WING_PROP_DIAMETER / 2.0
    r_tail = TAIL_PROP_DIAMETER / 2.0
    ok(
        "wing props clear the fuselage",
        WING_ROTOR_Y - r_wing > FUSELAGE_HALF_WIDTH,
        f"inboard tip at {WING_ROTOR_Y - r_wing:.3f} m vs "
        f"fuselage half-width {FUSELAGE_HALF_WIDTH:.3f} m",
    )
    ok(
        "wing props stay inboard of the tip",
        WING_ROTOR_Y + r_wing < WING_SPAN / 2.0,
        f"outboard tip at {WING_ROTOR_Y + r_wing:.3f} m vs "
        f"semi-span {WING_SPAN / 2.0:.3f} m",
    )
    ok(
        "wing and tail discs do not intersect",
        (TAIL_ROTOR_ARM + _wing_rotor_arm()) > (r_wing + r_tail),
        f"longitudinal separation {TAIL_ROTOR_ARM + _wing_rotor_arm():.3f} m vs "
        f"summed radii {r_wing + r_tail:.3f} m",
    )
    # In hover the tail disc lies horizontally, sweeping +/- r_tail in x about
    # its own station. If the horizontal tail sits inside that band it lives in
    # the rotor's own wash: lost thrust plus an uncommanded nose-down moment.
    disc_fwd_edge = TAIL_ROTOR_ARM - r_tail
    disc_aft_edge = TAIL_ROTOR_ARM + r_tail
    ok(
        "tail surfaces are clear of the tail rotor disc",
        not (disc_fwd_edge < TAIL_SURFACE_ARM < disc_aft_edge),
        f"tail surfaces at {TAIL_SURFACE_ARM:.3f} m aft vs disc band "
        f"{disc_fwd_edge:.3f}-{disc_aft_edge:.3f} m aft "
        f"({'behind' if TAIL_SURFACE_ARM > disc_aft_edge else 'ahead of'} the disc)",
    )
    # ⚠ The check above is BINARY -- inside the band or not -- and passed a
    # 43 mm gap from a 254 mm rotor as "clear". A hover wake contracts and
    # spreads; being 43 mm outside the tip path is not being out of the wake.
    # Require real separation, as a fraction of rotor radius.
    wake_gap = TAIL_SURFACE_ARM - disc_aft_edge
    ok(
        "tail surfaces have real separation from the rotor wake",
        wake_gap >= 0.60 * r_tail,
        f"{wake_gap * 1000:.0f} mm behind the disc edge = "
        f"{wake_gap / r_tail:.2f} rotor radii (need >= 0.60)",
    )
    ok(
        "tail rotor is forward of the tail surfaces",
        TAIL_ROTOR_ARM < TAIL_SURFACE_ARM,
        f"rotor {TAIL_ROTOR_ARM:.3f} m, surfaces {TAIL_SURFACE_ARM:.3f} m aft",
    )
    # A tail surface sitting behind a rotor is only acceptable when that rotor
    # is stopped in cruise -- otherwise it flies in the wake exactly when its
    # effectiveness matters most. Tie the two facts together so the layout
    # cannot be changed without the consequence surfacing.
    ok(
        "surfaces behind the rotor implies a hover-only rotor",
        (TAIL_SURFACE_ARM <= disc_aft_edge) or (not TAIL_TILTS),
        "tail surfaces sit aft of the disc, and the rotor is hover-only "
        "(stopped in cruise), so they never fly in its wake",
    )
    ok(
        "a fixed lift rotor uses a folding prop",
        TAIL_TILTS or TAIL_PROP_FOLDING,
        "fixed rotor + folding prop: a stopped flat disc would add ~30% to "
        "cruise drag",
    )
    waist_half = max(h for frac, h, _w in FUSELAGE_STATIONS if frac >= 0.80)
    ok(
        "pylon lifts the rotor clear of the fuselage",
        TAIL_PYLON_HEIGHT >= 1.5 * waist_half,
        f"pylon {TAIL_PYLON_HEIGHT * 1000:.0f} mm above a "
        f"{waist_half * 1000:.0f} mm waist half-height",
    )
    # The binding constraint on pylon height is DOWNLOAD, not inflow: a disc
    # sitting close to a surface blows onto it and loses thrust. The usual
    # guideline is a hub at least 0.10-0.20 rotor diameters clear.
    clear_frac = TAIL_PYLON_HEIGHT / TAIL_PROP_DIAMETER
    ok(
        "lift rotor is far enough above the deck to limit download",
        clear_frac >= 0.10,
        f"hub {TAIL_PYLON_HEIGHT * 1000:.0f} mm = {clear_frac:.2f} rotor "
        f"diameters above the deck (need >= 0.10)",
    )
    ok(
        "rear deck is cut down, not merely tapered",
        waist_half < 0.35 * max(h for frac, h, _w in FUSELAGE_STATIONS
                                if 0.30 <= frac <= 0.55),
        f"rear deck {waist_half * 1000:.0f} mm vs mid-body "
        f"{max(h for frac, h, _w in FUSELAGE_STATIONS if 0.30 <= frac <= 0.55) * 1000:.0f} mm",
    )
    # The waist is the whole point of this layout: it must actually be a waist.
    fwd_half = max(h for frac, h, _w in FUSELAGE_STATIONS if 0.30 <= frac <= 0.55)
    ok(
        "fuselage is genuinely waisted ahead of the tail rotor",
        waist_half < 0.55 * fwd_half,
        f"waist {waist_half * 1000:.0f} mm vs mid-body "
        f"{fwd_half * 1000:.0f} mm half-height "
        f"({100 * (1 - waist_half / fwd_half):.0f}% reduction)",
    )
    ok(
        "props clear the ground when tilted forward",
        LANDING_GEAR_HEIGHT + NACELLE_Z_OFFSET > r_wing,
        f"hub {LANDING_GEAR_HEIGHT + NACELLE_Z_OFFSET:.3f} m up vs "
        f"prop radius {r_wing:.3f} m",
    )

    # --- Control authority in hover ----------------------------------------
    ok(
        "roll authority",
        d.alpha_roll >= MIN_ALPHA_ROLL,
        f"{d.alpha_roll:.2f} rad/s^2 ({math.degrees(d.alpha_roll):.0f} deg/s^2), "
        f"need {MIN_ALPHA_ROLL:.1f}",
    )
    ok(
        "pitch authority",
        d.alpha_pitch >= MIN_ALPHA_PITCH,
        f"{d.alpha_pitch:.2f} rad/s^2 ({math.degrees(d.alpha_pitch):.0f} deg/s^2), "
        f"need {MIN_ALPHA_PITCH:.1f}",
    )
    ok(
        "yaw authority from wing vectoring alone",
        d.alpha_yaw >= MIN_ALPHA_YAW,
        f"{d.alpha_yaw:.2f} rad/s^2 ({math.degrees(d.alpha_yaw):.0f} deg/s^2), "
        f"need {MIN_ALPHA_YAW:.1f}",
    )

    # --- Flight envelope ----------------------------------------------------
    ok(
        "transition speed is above stall with margin",
        d.v_transition > d.v_stall,
        f"transition at {d.v_transition:.1f} m/s vs stall {d.v_stall:.1f} m/s",
    )
    ok(
        "cruise is comfortably above transition",
        V_CRUISE > d.v_transition * 1.10,
        f"cruise {V_CRUISE:.1f} m/s vs transition {d.v_transition:.1f} m/s",
    )
    ok(
        "wing loading is sane for the class",
        4.0 <= d.wing_loading <= 18.0,
        f"{d.wing_loading:.2f} kg/m^2 (need 4-18)",
    )
    ok(
        "aspect ratio is sane",
        4.0 <= d.wing_aspect_ratio <= 14.0,
        f"AR {d.wing_aspect_ratio:.2f}",
    )
    # Moving the tail forward to escape the rotor wash shortens the tail arm,
    # which costs pitch stability. These two coefficients are what decide
    # whether that trade was acceptable.
    # V-tail: judge on the EFFECTIVE projected areas, not the panel area.
    # A V-tail sized by its raw area is undersized in both axes.
    eff_pitch = d.tail_area_total * math.cos(d.tail_dihedral) ** 2
    eff_yaw = d.tail_area_total * math.sin(d.tail_dihedral) ** 2
    v_h = (eff_pitch * TAIL_SURFACE_ARM) / (d.wing_area * d.mac)
    ok(
        "tail volume coefficient in pitch",
        0.35 <= v_h <= 0.80,
        f"V_h = {v_h:.3f} from {eff_pitch:.4f} m^2 effective (need 0.35-0.80)",
    )
    v_v = (eff_yaw * TAIL_SURFACE_ARM) / (d.wing_area * WING_SPAN)
    ok(
        "tail volume coefficient in yaw",
        0.020 <= v_v <= 0.060,
        f"V_v = {v_v:.4f} from {eff_yaw:.4f} m^2 effective (need 0.020-0.060)",
    )
    ok(
        "V-tail resolves to the requested effectiveness",
        abs(eff_pitch - TAIL_PITCH_AREA_REQ) < 1e-6
        and abs(eff_yaw - TAIL_YAW_AREA_REQ) < 1e-6,
        f"{math.degrees(d.tail_dihedral):.1f}° dihedral, "
        f"{d.tail_area_total:.4f} m^2 total -> "
        f"{eff_pitch:.4f} pitch / {eff_yaw:.4f} yaw",
    )
    ok(
        "winglet is a sensible height",
        0.03 <= WINGLET_HEIGHT_FRAC <= 0.20,
        f"{d.winglet_height * 1000:.0f} mm = "
        f"{WINGLET_HEIGHT_FRAC * 100:.1f}% of semi-span",
    )
    ok(
        "winglet credit is modest and not load-bearing",
        1.0 < d.ar_effective / d.wing_aspect_ratio < 1.20,
        f"AR {d.wing_aspect_ratio:.2f} -> {d.ar_effective:.2f} effective "
        f"(+{100 * (d.ar_effective / d.wing_aspect_ratio - 1):.1f}%, ESTIMATE)",
    )
    ok(
        "winglet is canted, not a plain tip extension",
        math.radians(45.0) <= WINGLET_CANT <= math.radians(90.0),
        f"{math.degrees(WINGLET_CANT):.0f}° cant",
    )
    ok(
        "fuselage fineness ratio is in the low-drag range",
        6.0 <= d.fineness_ratio <= 14.0,
        f"{d.fineness_ratio:.1f} (below ~6 pressure drag climbs sharply)",
    )
    ok(
        "V-tail panels are not absurdly long",
        d.tail_panel_span < WING_SPAN / 4.0,
        f"panel span {d.tail_panel_span:.3f} m at chord {TAIL_CHORD:.3f} m",
    )
    ok(
        "disc loading is sane",
        d.disc_loading_wing <= 400.0,
        f"wing rotors {d.disc_loading_wing:.0f} N/m^2 (limit 400)",
    )

    # --- Tilt mechanism -----------------------------------------------------
    ok(
        "tilt range spans hover to cruise",
        abs(TILT_ANGLE_HOVER) < 1e-9
        and TILT_ANGLE_CRUISE >= math.radians(89.999),
        f"hover {math.degrees(TILT_ANGLE_HOVER):.1f} deg (up) to "
        f"cruise {math.degrees(TILT_ANGLE_CRUISE):.1f} deg (fwd)",
    )
    ok(
        "vectored-yaw travel exists past vertical",
        TILT_YAW_TRAVEL > math.radians(5.0),
        f"{math.degrees(TILT_YAW_TRAVEL):.1f} deg aft of vertical",
    )
    # PX4 clamps CA_SV_TL{i}_MINA/MAXA to -90..+90. A design that needs travel
    # outside that cannot be expressed in stock control allocation at all.
    ok(
        "tilt range fits PX4's -90..+90 limit",
        -math.radians(90.0) <= -TILT_YAW_TRAVEL
        and TILT_ANGLE_CRUISE <= math.radians(90.0),
        f"wing servos span {-math.degrees(TILT_YAW_TRAVEL):.1f} to "
        f"{math.degrees(TILT_ANGLE_CRUISE):.1f} deg",
    )
    # PX4 v1.17.0 src/modules/control_allocator/module.yaml: __max_num_tilts: 4
    # Was hardcoded as `3 <= 4` with the message "3 tilt servos" -- still
    # claiming three after the tail rotor became fixed. A check with a constant
    # on both sides cannot go wrong, and cannot go right either.
    ok(
        "tilt servo count is within PX4's allocator limit",
        N_SERVO_TILT <= 4,
        f"{N_SERVO_TILT} tilt servos, PX4 max is 4",
    )

    # --- Derived aerodynamics ----------------------------------------------
    ok(
        "finite-wing slope is below the 2-D section slope",
        d.cl_alpha < WING_CL_ALPHA_2D,
        f"CL_alpha {d.cl_alpha:.3f} < 2-D {WING_CL_ALPHA_2D:.3f} /rad "
        f"(downwash penalty {100 * (1 - d.cl_alpha / WING_CL_ALPHA_2D):.0f}%)",
    )
    ok(
        "finite-wing CL_max is below the section CL_max",
        d.cl_max < WING_CL_MAX_2D,
        f"CL_max {d.cl_max:.3f} < 2-D {WING_CL_MAX_2D:.3f}",
    )
    ok(
        "stall angle is physically sensible",
        math.radians(8.0) <= d.alpha_stall <= math.radians(20.0),
        f"{math.degrees(d.alpha_stall):.1f} deg, derived from CL_max/CL_alpha",
    )
    ok(
        "derived stall angle agrees with the section",
        d.alpha_stall < WING_ALPHA_STALL_2D + math.radians(2.0),
        f"{math.degrees(d.alpha_stall):.1f} deg vs section "
        f"{math.degrees(WING_ALPHA_STALL_2D):.1f} deg",
    )
    ok(
        "parasite drag is accounted for separately from the section",
        d.cd0 > WING_CD_MIN,
        f"CD0 {d.cd0:.4f} = section {WING_CD_MIN:.4f} + "
        f"{PARASITE_AREA / d.wing_area:.4f} parasites",
    )
    ok(
        "cruise L/D is realistic for the class",
        6.0 <= d.l_over_d_cruise <= 20.0,
        f"L/D {d.l_over_d_cruise:.1f} at {V_CRUISE:.0f} m/s",
    )
    ok(
        "wing has washout so the root stalls first",
        WING_TWIST_TIP < 0,
        f"tip twist {math.degrees(WING_TWIST_TIP):+.1f} deg",
    )
    ok(
        "taper ratio is sane",
        0.35 <= WING_TAPER <= 1.0,
        f"taper {WING_TAPER:.2f}",
    )
    ok(
        "fuselage stations are ordered and closed at both ends",
        all(FUSELAGE_STATIONS[i][0] < FUSELAGE_STATIONS[i + 1][0]
            for i in range(len(FUSELAGE_STATIONS) - 1))
        and FUSELAGE_STATIONS[0][0] == 0.0
        and FUSELAGE_STATIONS[-1][0] == 1.0,
        f"{len(FUSELAGE_STATIONS)} stations, monotonic, 0.0 to 1.0",
    )
    ok(
        "fuselage is long enough to carry the tail",
        FUSELAGE_LENGTH >= TAIL_SURFACE_ARM + _wing_rotor_arm() * 0.5,
        f"{FUSELAGE_LENGTH:.2f} m vs tail arm {TAIL_SURFACE_ARM:.2f} m",
    )
    # LENGTH is not the same as STRUCTURE. The check above passed happily while
    # the V-tail was rooted on a 10 x 8 mm needle, because it only ever compared
    # two longitudinal distances. This one looks at the section that is actually
    # there to bolt to.
    root_half_w = fuselage_half_width_at(-TAIL_SURFACE_ARM)
    root_half_h = fuselage_half_height_at(-TAIL_SURFACE_ARM)
    tail_root_thick = 0.09 * TAIL_CHORD          # NACA 0009
    # Having SECTION at the root is not the same as having BODY under the whole
    # root chord. At FUSELAGE_LENGTH 1.35 the body ended at -0.872 m while the
    # tail root ran to -1.005 m, so 133 mm of a 180 mm root chord cantilevered
    # into thin air -- and every existing check passed.
    fuse_tail_x = fuselage_nose_x() - FUSELAGE_LENGTH
    tail_te_x = -TAIL_SURFACE_ARM - 0.75 * TAIL_CHORD
    ok(
        "fuselage runs aft of the V-tail trailing edge",
        fuse_tail_x <= tail_te_x,
        f"body ends at {fuse_tail_x:.4f} m, tail TE at {tail_te_x:.4f} m "
        f"({(tail_te_x - fuse_tail_x) * 1000:+.0f} mm of overhang)",
    )
    ok(
        "V-tail root has fuselage section to attach to",
        2.0 * root_half_w >= tail_root_thick,
        f"body {2 * root_half_w * 1000:.1f} x {2 * root_half_h * 1000:.1f} mm "
        f"at the root vs a {tail_root_thick * 1000:.1f} mm thick tail section",
    )

    # --- Buried boom --------------------------------------------------------
    # The boom used to hang 13.7 mm below the wing. Nothing checked it, because
    # the boom's z was a literal in gen_geometry.py and params.py never saw it.
    _c_root = WING_CHORD * 3.0 * (1.0 + WING_TAPER) / (
        2.0 * (1.0 + WING_TAPER + WING_TAPER ** 2))
    _c_tip = _c_root * WING_TAPER
    _f = WING_ROTOR_Y / (WING_SPAN / 2.0)
    _c_local = _c_root + (_c_tip - _c_root) * _f
    # Half-thickness of a 4-digit section at 30% chord, in chord units.
    _yt30 = 5.0 * (int(WING_NACA[2:]) / 100.0) * (
        0.2969 * math.sqrt(0.30) - 0.1260 * 0.30 - 0.3516 * 0.30 ** 2
        + 0.2843 * 0.30 ** 3 - 0.1036 * 0.30 ** 4)
    _skin = _yt30 * _c_local - BOOM_DIA_MM / 2000.0
    ok(
        "wing boom fits inside the wing section",
        _skin >= 0.0015,
        f"{_skin * 1000:.1f} mm of skin each side of a "
        f"{BOOM_DIA_MM:.0f} mm boom in a {2 * _yt30 * _c_local * 1000:.1f} mm "
        f"section (need >= 1.5)",
    )

    # --- Tilt nacelle mechanism (mm) ---------------------------------------
    # Note the unit change: everything above is SI, this block is millimetres.
    ok(
        "wall thickness is printable",
        WALL_MM >= 3 * NOZZLE_DIA_MM,
        f"{WALL_MM:.1f} mm = {WALL_MM / NOZZLE_DIA_MM:.1f} perimeters "
        f"at {NOZZLE_DIA_MM:.1f} mm",
    )
    ok(
        "bearing seat is larger than the shaft",
        BEARING_OD_MM > TILT_SHAFT_DIA_MM + 2 * WALL_MM * 0.5,
        f"bearing OD {BEARING_OD_MM:.1f} mm vs shaft {TILT_SHAFT_DIA_MM:.1f} mm",
    )
    ok(
        "bearing boss has material around it",
        (BEARING_OD_MM + 2 * WALL_MM) < NACELLE_WIDTH_MM,
        f"boss OD {BEARING_OD_MM + 2 * WALL_MM:.1f} mm inside "
        f"nacelle width {NACELLE_WIDTH_MM:.1f} mm",
    )
    ok(
        "fits are clearance, never press",
        BEARING_SEAT_CLEARANCE_MM > 0 and SHAFT_BORE_CLEARANCE_MM > 0,
        f"bearing +{BEARING_SEAT_CLEARANCE_MM:.2f} mm, "
        f"shaft +{SHAFT_BORE_CLEARANCE_MM:.2f} mm (no arm to test-fit against)",
    )
    # The plate must contain the bolt circle plus half a bolt of material plus
    # a wall on each side. Stated as the required plate diameter, not as an
    # inequality with terms on both sides -- the first version of this check
    # double-counted the bolt diameter and failed a design that was fine.
    plate_dia_needed = (
        MOTOR_BOLT_PITCH_MM * math.sqrt(2)   # bolt circle, across corners
        + MOTOR_BOLT_DIA_MM                  # half a bolt each side
        + 2 * WALL_MM                        # wall each side
    )
    ok(
        "motor bolt pattern fits the cradle plate",
        CRADLE_PLATE_DIA_MM >= plate_dia_needed,
        f"plate {CRADLE_PLATE_DIA_MM:.1f} mm vs {plate_dia_needed:.1f} mm needed "
        f"(bolt circle {MOTOR_BOLT_PITCH_MM * math.sqrt(2):.1f} mm)",
    )
    ok(
        "central bore clears the motor shaft without breaking the bolt circle",
        MOTOR_SHAFT_CLEAR_MM + 2 * WALL_MM
        < MOTOR_BOLT_PITCH_MM * math.sqrt(2) - MOTOR_BOLT_DIA_MM,
        f"bore {MOTOR_SHAFT_CLEAR_MM:.1f} mm + walls vs "
        f"{MOTOR_BOLT_PITCH_MM * math.sqrt(2) - MOTOR_BOLT_DIA_MM:.1f} mm available",
    )
    ok(
        "cradle plate is thick enough for M3 threads",
        CRADLE_PLATE_MM >= 1.2 * MOTOR_BOLT_DIA_MM,
        f"{CRADLE_PLATE_MM:.1f} mm vs M{MOTOR_BOLT_DIA_MM:.0f}",
    )
    ok(
        "boom clamp is a clearance fit",
        BOOM_CLAMP_CLEARANCE_MM > 0,
        f"boom {BOOM_DIA_MM:.1f} mm +{BOOM_CLAMP_CLEARANCE_MM:.2f} mm",
    )

    # The one that actually decides whether the mechanism works. The servo has
    # to hold the nacelle against thrust acting off the tilt axis plus the
    # nacelle's own weight acting off it. Both offsets are budgeted above
    # rather than assumed to be zero, because a real build never achieves zero.
    torque_thrust = d.thrust_wing_each * (THRUST_AXIS_OFFSET_MM / 1000.0)
    torque_weight = MASS_NACELLE_WING * G * (NACELLE_CG_OFFSET_MM / 1000.0)
    torque_req = (torque_thrust + torque_weight) * SERVO_SAFETY_FACTOR
    servo_nm = SERVO_STALL_TORQUE_KGCM * 0.0980665
    ok(
        "tilt servo has torque for the job",
        servo_nm >= torque_req,
        f"need {torque_req:.3f} N.m at SF {SERVO_SAFETY_FACTOR:.1f}, "
        f"servo gives {servo_nm:.3f} N.m ({servo_nm / torque_req:.1f}x margin)",
    )
    # --- Servos and electrical load -----------------------------------------
    n_servo = N_SERVO_SURFACE + N_SERVO_TILT
    ok(
        "servo count matches the control allocation",
        n_servo == 6,
        f"{N_SERVO_SURFACE} control surfaces + {N_SERVO_TILT} tilts = {n_servo} "
        f"servos (PX4 allocates SIM_GZ_SV_FUNC1..{n_servo})",
    )
    ok(
        "no tilt servo is specified for the fixed tail rotor",
        N_SERVO_TILT == (3 if TAIL_TILTS else 2),
        f"{N_SERVO_TILT} tilt servos for {'three' if TAIL_TILTS else 'two'} "
        f"tilting nacelles",
    )
    # The one that decides whether the aircraft needs a part nobody has costed.
    servo_peak_a = n_servo * SERVO_STALL_CURRENT_A
    ok(
        "servo rail needs a dedicated BEC, and one is budgeted",
        servo_peak_a > FC_INTERNAL_BEC_MAX_A,
        f"{n_servo} servos x {SERVO_STALL_CURRENT_A:.1f} A = {servo_peak_a:.1f} A "
        f"peak vs {FC_INTERNAL_BEC_MAX_A:.1f} A from the FC regulator "
        f"-> separate BEC required",
    )
    ok(
        "surface servos are sized below the tilt servos",
        SURFACE_SERVO_TORQUE_KGCM < SERVO_STALL_TORQUE_KGCM,
        f"surfaces {SURFACE_SERVO_TORQUE_KGCM:.0f} kg.cm vs tilt "
        f"{SERVO_STALL_TORQUE_KGCM:.0f} kg.cm (surfaces carry hinge moment "
        f"only, no thrust vector or nacelle weight)",
    )

    # --- Nose camera --------------------------------------------------------
    if CAMERA_ENABLED:
        r_wing = WING_PROP_DIAMETER / 2.0
        ok(
            "nose camera is ahead of the wing rotor discs",
            camera_x() > _wing_rotor_arm() + r_wing,
            f"camera at {camera_x():.3f} m fwd vs disc leading edge at "
            f"{_wing_rotor_arm() + r_wing:.3f} m (no blade in shot)",
        )
        ok(
            "camera mass is inside the payload budget",
            CAMERA_MASS <= MASS_PAYLOAD,
            f"camera {CAMERA_MASS * 1000:.0f} g of a "
            f"{MASS_PAYLOAD * 1000:.0f} g payload allowance",
        )

    # --- Pushrod buckling ---------------------------------------------------
    # The question a wire cannot answer: does the rod survive PUSH? Euler,
    # pinned-pinned:  Pcr = pi^2 E I / L^2,  I = pi d^4 / 64.
    # If Pcr is not comfortably above the hinge load, the rod bows instead of
    # moving the surface and the control goes soft at exactly the moment it is
    # needed most -- full deflection at speed.
    q = 0.5 * RHO * V_CRUISE ** 2
    for label, dia, area, chord, arm_len, sleeved in (
        ("aileron", AILERON_ROD_DIA_MM,
         AILERON_SPAN_FRAC * WING_SPAN * AILERON_CHORD_FRAC * WING_CHORD,
         AILERON_CHORD_FRAC * WING_CHORD, 0.10, False),
        ("ruddervator", RUDDERVATOR_ROD_DIA_MM,
         RUDDERVATOR_SPAN_FRAC * solve().tail_panel_span
         * RUDDERVATOR_CHORD_FRAC * TAIL_CHORD,
         RUDDERVATOR_CHORD_FRAC * TAIL_CHORD, 0.32, RUDDERVATOR_ROD_SLEEVED),
    ):
        hinge_moment = q * area * chord * HINGE_MOMENT_COEFF
        force = hinge_moment / (CONTROL_HORN_H_MM / 1000.0)
        # A sleeved rod's free length is the sleeve support pitch, not the run.
        free_len = (SLEEVE_SUPPORT_PITCH_MM / 1000.0) if sleeved else arm_len
        inertia = math.pi * (dia / 1000.0) ** 4 / 64.0
        p_cr = math.pi ** 2 * PUSHROD_E_PA * inertia / free_len ** 2
        ok(
            f"{label} pushrod does not buckle under push",
            p_cr >= 3.0 * force,
            f"{dia:.1f} mm rod over {free_len * 1000:.0f} mm free length: "
            f"Pcr {p_cr:.1f} N vs {force:.2f} N hinge load "
            f"({p_cr / max(force, 1e-9):.1f}x)"
            f"{' [sleeved]' if sleeved else ''}",
        )

    # --- Primary structure --------------------------------------------------
    ok(
        "longeron pair fits where it actually runs",
        LONGERON_SPACING_MM / 2000.0 + LONGERON_DIA_MM / 2000.0
        <= fuselage_half_width_at(LONGERON_AFT_X),
        f"pair spans {LONGERON_SPACING_MM + LONGERON_DIA_MM:.0f} mm vs "
        f"{2 * fuselage_half_width_at(LONGERON_AFT_X) * 1000:.0f} mm of body at "
        f"its aft end (x={LONGERON_AFT_X:+.3f} m)",
    )
    ok(
        "single tail boom fits the slender aft body",
        TAILBOOM_DIA_MM / 2000.0 <= fuselage_half_width_at(-TAIL_SURFACE_ARM),
        f"{TAILBOOM_DIA_MM:.0f} mm boom vs "
        f"{2 * fuselage_half_width_at(-TAIL_SURFACE_ARM) * 1000:.0f} mm of body "
        f"at the V-tail root -- a 46 mm longeron pair could not, which is why "
        f"the pair stops at x={LONGERON_AFT_X:+.3f} m",
    )
    ok(
        "tail boom overlaps the longeron pair for a splice",
        LONGERON_AFT_X > -TAIL_SURFACE_ARM,
        f"pair ends {LONGERON_AFT_X:+.3f} m, boom carries on to the tail at "
        f"{-TAIL_SURFACE_ARM:+.3f} m",
    )
    _sp = wing_station(WING_SPAR_SPAN_FRAC * WING_SPAN / 2.0)
    _yt_sp, _ = naca_yt_yc(WING_NACA, WING_SPAR_CHORD_FRAC)
    ok(
        "wing spar fits the section at its outboard end",
        2.0 * _yt_sp * _sp["chord"] * 1000.0 >= WING_SPAR_DIA_MM + 3.0,
        f"section {2 * _yt_sp * _sp['chord'] * 1000:.1f} mm at "
        f"{WING_SPAR_SPAN_FRAC:.0%} semi-span vs {WING_SPAR_DIA_MM:.0f} mm spar",
    )
    ok(
        "wing spar passes through the nacelle station",
        WING_SPAR_SPAN_FRAC * WING_SPAN / 2.0 > WING_ROTOR_Y,
        f"spar runs to {WING_SPAR_SPAN_FRAC * WING_SPAN / 2.0:.3f} m, nacelle "
        f"at {WING_ROTOR_Y:.3f} m -- the boom load lands on the spar, not the skin",
    )

    # --- Joints -------------------------------------------------------------
    # The wing joint has to carry the panel's whole lift plus the nacelle. A
    # 1.8 g load case is the usual sizing gust for this class.
    panel_lift = 1.8 * (MASS_TOTAL * G) / 2.0
    pin_area = 2.0 * math.pi * (JOINT_PIN_DIA_MM / 2000.0) ** 2   # double shear
    pin_capacity = pin_area * JOINT_PIN_SHEAR_MPA * 1e6
    ok(
        "wing retention pin carries the panel load in double shear",
        pin_capacity >= 3.0 * panel_lift,
        f"{JOINT_PIN_DIA_MM:.0f} mm pin: {pin_capacity:.0f} N capacity vs "
        f"{panel_lift:.0f} N panel load at 1.8 g ({pin_capacity / panel_lift:.1f}x)",
    )
    ok(
        "spar engagement is long enough to react the panel's bending",
        JOINT_SPAR_ENGAGE_MM / 1000.0 >= 6.0 * WING_SPAR_DIA_MM / 1000.0,
        f"{JOINT_SPAR_ENGAGE_MM:.0f} mm of sleeve over a "
        f"{WING_SPAR_DIA_MM:.0f} mm spar "
        f"({JOINT_SPAR_ENGAGE_MM / WING_SPAR_DIA_MM:.1f} diameters, want >= 6)",
    )
    ok(
        "formers are spaced closely enough to stabilise the shell",
        all(abs(FORMERS[i][1] - FORMERS[i + 1][1]) <= 0.16
            for i in range(len(FORMERS) - 1)),
        f"{len(FORMERS)} formers, max gap "
        f"{max(abs(FORMERS[i][1] - FORMERS[i + 1][1]) for i in range(len(FORMERS) - 1)) * 1000:.0f} mm",
    )
    for fname, fx in FORMERS:
        ok(
            f"{fname} former sits on real fuselage section",
            fuselage_half_width_at(fx) > LONGERON_SPACING_MM / 2000.0
            + LONGERON_DIA_MM / 2000.0,
            f"body {2 * fuselage_half_width_at(fx) * 1000:.0f} mm wide at "
            f"x={fx:+.3f} m vs {LONGERON_SPACING_MM + LONGERON_DIA_MM:.0f} mm "
            f"longeron pair",
        )

    # --- Servo bays ---------------------------------------------------------
    # A servo has to physically fit inside the surface it drives. The aileron
    # servo goes in the wing; the ruddervator servos deliberately do NOT go in
    # the V-tail, and this is where that decision is justified in arithmetic.
    a_geo = aileron_geometry()
    st_a = wing_station(a_geo["y_mid"])
    yt_bay, _ = naca_yt_yc(WING_NACA, SERVO_BAY_CHORD_FRAC)
    bay_thick = 2.0 * yt_bay * st_a["chord"] * 1000.0        # mm
    need = SURFACE_SERVO_W_MM + SERVO_MOUNT_CLEARANCE_MM
    ok(
        "aileron servo fits inside the wing section",
        bay_thick >= need + 2 * 1.5,
        f"wing {bay_thick:.1f} mm thick at {SERVO_BAY_CHORD_FRAC:.0%} chord vs "
        f"{need:.1f} mm servo ({(bay_thick - need) / 2:.1f} mm skin each side)",
    )
    # The ruddervator servo now lives IN the panel. This is the check that
    # decides whether that is honest -- and it is the check I should have
    # re-run when the servo case shrank, instead of carrying forward a "does
    # not fit" conclusion that was only true of the larger case.
    c_ts = TAIL_CHORD + (TAIL_CHORD * 0.72 - TAIL_CHORD) * TAIL_SERVO_PANEL_SPAN_FRAC
    yt_ts, _ = naca_yt_yc(TAIL_NACA, TAIL_SERVO_PANEL_CHORD_FRAC)
    panel_thick = 2.0 * yt_ts * c_ts * 1000.0
    need_ts = TAIL_SERVO_W_MM + SERVO_MOUNT_CLEARANCE_MM
    ok(
        "ruddervator servo fits inside the V-tail panel",
        panel_thick >= need_ts + 2 * 1.5,
        f"panel {panel_thick:.1f} mm thick at "
        f"{TAIL_SERVO_PANEL_CHORD_FRAC:.0%} chord / "
        f"{TAIL_SERVO_PANEL_SPAN_FRAC:.0%} span vs {need_ts:.1f} mm servo "
        f"({(panel_thick - need_ts) / 2:.1f} mm skin each side)",
    )
    # THE ONE THAT WAS MISSING. A control horn has to land ON the surface it
    # drives. The servo sat at 16% of panel span while the ruddervator spans
    # 27.5%..82.5%, so the horn was bolted to the FIXED panel and the linkage
    # moved nothing. Every other check passed -- the servo fitted, the rod did
    # not buckle, the torque was ample -- because none of them asked the only
    # question that mattered.
    rv_s_mid = 0.55
    rv_half = RUDDERVATOR_SPAN_FRAC / 2.0
    ok(
        "tail servo is at the ruddervator's own span station",
        rv_s_mid - rv_half <= TAIL_SERVO_PANEL_SPAN_FRAC <= rv_s_mid + rv_half,
        f"servo at {TAIL_SERVO_PANEL_SPAN_FRAC:.0%} of panel span, ruddervator "
        f"spans {rv_s_mid - rv_half:.1%}..{rv_s_mid + rv_half:.1%} -- the horn "
        f"lands on the MOVING surface",
    )
    ok(
        "aileron servo is at the aileron's own span station",
        abs(AILERON_Y_FRAC * WING_SPAN - aileron_geometry()["y_mid"]) < 1e-9,
        f"servo and aileron share mid-span at "
        f"{aileron_geometry()['y_mid']:.3f} m",
    )
    # Length, not just existence. A linkage that reaches is not the same as a
    # linkage that is any good: a long rod crossing open panel is compliance,
    # drag and something to catch on. Both runs are held to the same standard.
    _st_a = wing_station(aileron_geometry()["y_mid"])
    ail_rod_mm = (aileron_geometry()["hinge"] - SERVO_BAY_CHORD_FRAC) \
        * _st_a["chord"] * 1000.0
    _c_rv = TAIL_CHORD + (TAIL_CHORD * 0.72 - TAIL_CHORD) \
        * TAIL_SERVO_PANEL_SPAN_FRAC
    rv_rod_mm = ((1.0 - RUDDERVATOR_CHORD_FRAC)
                 - TAIL_SERVO_PANEL_CHORD_FRAC) * _c_rv * 1000.0
    ok(
        "control pushrods are short",
        max(ail_rod_mm, rv_rod_mm) <= 40.0,
        f"aileron {ail_rod_mm:.0f} mm, ruddervator {rv_rod_mm:.0f} mm "
        f"(limit 40 mm)",
    )
    ok(
        "the two control runs are comparable",
        rv_rod_mm <= 1.5 * ail_rod_mm,
        f"ruddervator {rv_rod_mm:.0f} mm vs aileron {ail_rod_mm:.0f} mm "
        f"({rv_rod_mm / ail_rod_mm:.2f}x, limit 1.50)",
    )
    ok(
        "ruddervator pushrod runs inside the fixed panel",
        RUDDERVATOR_CHORD_FRAC < 1.0 - TAIL_SERVO_PANEL_CHORD_FRAC,
        f"servo at {TAIL_SERVO_PANEL_CHORD_FRAC:.0%} chord, hinge at "
        f"{1 - RUDDERVATOR_CHORD_FRAC:.0%} -- "
        f"{(1 - RUDDERVATOR_CHORD_FRAC - TAIL_SERVO_PANEL_CHORD_FRAC) * c_ts * 1000:.0f} mm "
        f"of rod, all of it inside the panel",
    )
    # Both servos sized against the load they actually see, rather than against
    # a number picked once and copied.
    q_s = 0.5 * RHO * V_CRUISE ** 2
    for lbl, area, chord, torque_kgcm in (
        ("aileron",
         AILERON_SPAN_FRAC * WING_SPAN * AILERON_CHORD_FRAC * WING_CHORD,
         AILERON_CHORD_FRAC * WING_CHORD, SURFACE_SERVO_TORQUE_KGCM),
        ("ruddervator",
         RUDDERVATOR_SPAN_FRAC * d.tail_panel_span
         * RUDDERVATOR_CHORD_FRAC * TAIL_CHORD,
         RUDDERVATOR_CHORD_FRAC * TAIL_CHORD, TAIL_SERVO_TORQUE_KGCM),
    ):
        need_nm = q_s * area * chord * HINGE_MOMENT_COEFF
        have_nm = torque_kgcm * 0.0980665 * (SERVO_HORN_RADIUS_MM / 10.0)
        ok(
            f"{lbl} servo torque covers the hinge moment",
            have_nm >= 3.0 * need_nm,
            f"need {need_nm:.4f} N.m, {torque_kgcm:.0f} kg.cm servo gives "
            f"{have_nm:.3f} N.m ({have_nm / max(need_nm, 1e-9):.0f}x)",
        )

    # --- Equipment bay ------------------------------------------------------
    # Does every box actually fit inside the lofted body at its own station?
    # The fuselage is 122 mm across at its widest and narrows hard toward both
    # ends, so "it fits in the fuselage" is meaningless without a station.
    for name, x_c, ln, wd, ht, _m in EQUIPMENT:
        for edge in (x_c + ln / 2000.0, x_c - ln / 2000.0):
            half_w = fuselage_half_width_at(edge)
            half_h = fuselage_half_height_at(edge)
            ok(
                f"{name} fits the fuselage section",
                wd / 2000.0 <= half_w and ht / 2000.0 <= half_h,
                f"{wd:.0f} x {ht:.0f} mm at x={edge:+.3f} m vs bay "
                f"{2 * half_w * 1000:.0f} x {2 * half_h * 1000:.0f} mm",
            )
    equip_mass = sum(m for *_, m in EQUIPMENT)
    ok(
        "equipment mass is inside the budget it draws on",
        equip_mass <= MASS_BATTERY + MASS_AVIONICS + MASS_PAYLOAD + 1e-9,
        f"{equip_mass:.3f} kg listed vs "
        f"{MASS_BATTERY + MASS_AVIONICS + MASS_PAYLOAD:.3f} kg of battery + "
        f"avionics + payload allowance",
    )

    ok(
        "wing prop clears the yoke",
        WING_PROP_DIAMETER * 1000.0 / 2.0 > NACELLE_WIDTH_MM / 2.0,
        f"prop radius {WING_PROP_DIAMETER * 1000 / 2:.1f} mm vs "
        f"half-width {NACELLE_WIDTH_MM / 2:.1f} mm",
    )

    if verbose:
        for line in passed:
            print(f"  PASS  {line}")

    return passed


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def report() -> str:
    d = solve()
    lines = [
        "=" * 74,
        "TRI-TILTROTOR VTOL -- DESIGN REPORT",
        "=" * 74,
        "",
        "MASS AND WING",
        f"  MTOW                     {MASS_TOTAL:>10.3f} kg",
        f"  Weight                   {d.weight:>10.3f} N",
        f"  Wing area                {d.wing_area:>10.4f} m^2",
        f"  Wing span / chord        {WING_SPAN:>10.3f} / {WING_CHORD:.3f} m",
        f"  Aspect ratio             {d.wing_aspect_ratio:>10.2f}",
        f"  Wing loading             {d.wing_loading:>10.2f} kg/m^2",
        "",
        "LONGITUDINAL LAYOUT (from CG, + forward)",
        f"  CG                       {CG_MAC_FRACTION * 100:>10.1f} % MAC",
        f"  Wing rotor plane         {d.wing_rotor_arm:>+10.3f} m",
        f"  Tail rotor plane         {-d.tail_rotor_arm:>+10.3f} m",
        f"  Wing rotor lateral       {WING_ROTOR_Y:>10.3f} m",
        "",
        "HOVER TRIM (solved, not assumed)",
        f"  Wing rotor thrust, each  {d.thrust_wing_each:>10.2f} N",
        f"  Tail rotor thrust        {d.thrust_tail:>10.2f} N",
        f"  Tail share of lift       {d.tail_lift_fraction * 100:>10.1f} %",
        f"  Disc loading, wing       {d.disc_loading_wing:>10.0f} N/m^2",
        f"  Disc loading, tail       {d.disc_loading_tail:>10.0f} N/m^2",
        "",
        "INERTIA (estimated)",
        f"  Ixx / Iyy / Izz          {d.ixx:>10.3f} / {d.iyy:.3f} / {d.izz:.3f} kg.m^2",
        "",
        "HOVER CONTROL AUTHORITY",
        f"  Roll  (diff. thrust)     {d.alpha_roll:>10.2f} rad/s^2"
        f"   ({math.degrees(d.alpha_roll):.0f} deg/s^2)",
        f"  Pitch (tail modulation)  {d.alpha_pitch:>10.2f} rad/s^2"
        f"   ({math.degrees(d.alpha_pitch):.0f} deg/s^2)",
        f"  Yaw   (wing vectoring)   {d.alpha_yaw:>10.2f} rad/s^2"
        f"   ({math.degrees(d.alpha_yaw):.0f} deg/s^2)",
        "",
        "AERODYNAMICS (derived from NACA " + WING_NACA + ", not asserted)",
        f"  CL_alpha, 2-D -> finite  {WING_CL_ALPHA_2D:>10.3f} -> {d.cl_alpha:.3f} /rad",
        f"  CL_max,   2-D -> finite  {WING_CL_MAX_2D:>10.3f} -> {d.cl_max:.3f}",
        f"  Stall angle              {math.degrees(d.alpha_stall):>10.1f} deg",
        f"  CD0 (section+parasite)   {d.cd0:>10.4f}",
        f"  Cruise L/D               {d.l_over_d_cruise:>10.1f}",
        "",
        "ENVELOPE",
        f"  Stall speed              {d.v_stall:>10.2f} m/s",
        f"  Transition complete at   {d.v_transition:>10.2f} m/s",
        f"  Cruise                   {V_CRUISE:>10.2f} m/s",
        f"  Cruise thrust required   {d.thrust_required_cruise:>10.2f} N",
        "",
    ]
    return "\n".join(lines)


def roll_authority_sensitivity() -> str:
    """Show how roll authority collapses if the nacelles move inboard.

    This is the design point that is easiest to get wrong by eye: mounting the
    nacelles at the wing root looks tidy and is nearly unflyable.
    """
    ixx_base = _inertia_estimates()[0]
    d = solve()
    out = ["ROLL AUTHORITY vs NACELLE LATERAL STATION",
           "  y (m)    arm      alpha_roll        verdict"]
    for y in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50):
        ixx = (2 * MASS_WING_PANEL * WING_SPAN ** 2 / 12.0
               + 2 * MASS_NACELLE_WING * y ** 2)
        d_thrust = ROLL_THRUST_MARGIN * d.thrust_wing_each
        alpha = (2 * d_thrust * y) / ixx
        verdict = "OK" if alpha >= MIN_ALPHA_ROLL else "UNDER-ACTUATED"
        marker = "  <-- chosen" if abs(y - WING_ROTOR_Y) < 1e-9 else ""
        out.append(
            f"  {y:.2f}   {y:.3f}   {alpha:>6.2f} rad/s^2   {verdict}{marker}"
        )
    return "\n".join(out)


if __name__ == "__main__":
    print(report())
    print("RUNNING DESIGN INVARIANTS")
    print("-" * 74)
    try:
        results = check(verbose=True)
    except DesignError as exc:
        print(f"\n  FAIL  {exc}\n")
        raise SystemExit(1)
    print("-" * 74)
    print(f"  {len(results)}/{len(results)} invariants passed\n")
    print(roll_authority_sensitivity())
