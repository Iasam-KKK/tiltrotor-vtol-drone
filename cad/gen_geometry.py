r"""
Generate the real airframe geometry -- lofted NACA wing, streamlined fuselage,
tail surfaces -- from params.py, and export meshes for the Gazebo visuals.

Why this exists: the first version of the model used <box> primitives for every
visual. It flew correctly (the LiftDrag coefficients were doing the real work)
but it looked like it had been cut from planks, which is fatal for a project
whose output is renders, video and a marketplace listing.

The aerodynamic coefficients are NOT re-derived here. params.solve() computes
them from the NACA section; this module builds the shape that section implies.
One source, two consumers -- the picture and the physics cannot disagree.

Collision geometry deliberately stays as primitives in gen_sdf.py. Mesh
collision would cost a large amount of physics time for no fidelity gain on an
aircraft whose contacts are "sitting on the ground" and "crashing".

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_geometry.py
"""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

from build123d import (
    Align, BuildLine, BuildPart, BuildSketch, Location, Mode, Plane, Polyline,
    Vector, export_step, export_stl, extrude, loft, make_face, Rot,
)

import params as P

MESH_DIR = (Path(__file__).resolve().parent.parent
            / "sim" / "models" / "tri_tiltrotor" / "meshes")
STEP_DIR = Path(__file__).resolve().parent / "out"


# ---------------------------------------------------------------------------
# NACA 4-digit section
# ---------------------------------------------------------------------------

def naca4(code: str, n: int = 80) -> list[tuple[float, float]]:
    """Return closed airfoil coordinates for a NACA 4-digit section.

    Standard formulation. Cosine spacing concentrates points at the leading
    edge where curvature is highest -- uniform spacing produces a visibly
    faceted nose at this point count.
    """
    m = int(code[0]) / 100.0          # max camber
    p = int(code[1]) / 10.0           # position of max camber
    t = int(code[2:]) / 100.0         # thickness

    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []

    for i in range(n + 1):
        beta = math.pi * i / n
        x = 0.5 * (1.0 - math.cos(beta))          # cosine spacing

        yt = 5.0 * t * (
            0.2969 * math.sqrt(x)
            - 0.1260 * x
            - 0.3516 * x ** 2
            + 0.2843 * x ** 3
            - 0.1036 * x ** 4                     # closed trailing edge
        )

        if m > 0.0 and p > 0.0:
            if x < p:
                yc = m / p ** 2 * (2.0 * p * x - x ** 2)
                dyc = 2.0 * m / p ** 2 * (p - x)
            else:
                yc = m / (1.0 - p) ** 2 * ((1.0 - 2.0 * p) + 2.0 * p * x - x ** 2)
                dyc = 2.0 * m / (1.0 - p) ** 2 * (p - x)
        else:
            yc, dyc = 0.0, 0.0

        th = math.atan(dyc)
        upper.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lower.append((x + yt * math.sin(th), yc - yt * math.cos(th)))

    # Upper surface TE -> LE, then lower LE -> TE. Drop duplicated endpoints.
    pts = list(reversed(upper)) + lower[1:-1]
    return pts


# At least this much skin must remain around a buried boom, each side.
BOOM_MIN_SKIN_MM = 1.5

# Hinge gap between a control surface and the fixed surface it is cut out of.
CONTROL_GAP_MM = 1.5


def naca_yt_yc(code: str, x: float) -> tuple[float, float]:
    """Half-thickness and camber of a 4-digit section at x/c, in chord units.

    Same formulation as naca4(), factored out so the boom burial depth and the
    control-surface hinge lines are DERIVED from the section rather than being
    separate hand-entered numbers that can drift away from it.
    """
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0

    yt = 5.0 * t * (
        0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
        + 0.2843 * x ** 3 - 0.1036 * x ** 4
    )
    if m > 0.0 and p > 0.0:
        if x < p:
            yc = m / p ** 2 * (2.0 * p * x - x ** 2)
        else:
            yc = m / (1.0 - p) ** 2 * ((1.0 - 2.0 * p) + 2.0 * p * x - x ** 2)
    else:
        yc = 0.0
    return yt, yc


def chord_frac_at_thickness(code: str, chord: float, needed: float) -> float:
    """Aft-most x/c at which the section is still `needed` metres thick.

    Walks aft from max thickness. Used to decide how far back a buried boom can
    run before the skin would have to bulge around it -- a limit that follows
    from the airfoil and the local chord, so it re-derives itself if either
    changes.
    """
    best = 0.30                                   # max thickness for 4-digit
    for i in range(30, 96):
        x = i / 100.0
        yt, _ = naca_yt_yc(code, x)
        if 2.0 * yt * chord >= needed:
            best = x
        else:
            break
    return best


def aft_section_points(code: str, chord: float, hinge: float, twist: float,
                       n: int = 40) -> list[tuple[float, float]]:
    """Closed polygon of the AFT portion of a section, origin at the hinge.

    This is what makes a control surface an actual aerofoil slice instead of a
    grey box. The returned polygon runs along the upper skin from the hinge to
    the trailing edge, then back along the lower skin, and is expressed
    relative to the hinge point on the camber line -- so a link placed at the
    hinge needs no visual offset, and the surface rotates about its real hinge
    line rather than about its own centroid.

    Frame matches section_points(): +x forward, so the trailing edge is at
    NEGATIVE x.
    """
    _, yc_h = naca_yt_yc(code, hinge)

    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    for i in range(n + 1):
        # Cosine spacing again, clustered at the trailing edge where the
        # section closes and straight spacing produces slivers.
        f = 0.5 * (1.0 - math.cos(math.pi * i / n))
        x = hinge + (1.0 - hinge) * f
        yt, yc = naca_yt_yc(code, x)
        # +x forward, measured from the hinge; z relative to the hinge camber.
        # Y negated for the same reason as section_points(): the loft plane's
        # in-plane Y axis is (0,0,-1), so an un-negated camber lands upside
        # down and the surface would be cut out of the wrong side of the wing.
        X = (hinge - x) * chord
        upper.append((X, -(yc + yt - yc_h) * chord))
        lower.append((X, -(yc - yt - yc_h) * chord))

    pts = upper + list(reversed(lower))[1:-1]
    c, s = math.cos(twist), math.sin(twist)
    return [(px * c - py * s, px * s + py * c) for px, py in pts]


def section_points(code: str, chord: float, twist: float,
                   pitch_about: float = 0.25,
                   n: int = 80) -> list[tuple[float, float]]:
    """Scale a unit section to chord and rotate it by `twist` about a point.

    Twist is applied here, in 2-D, rather than by rotating the loft plane.
    Rotating the plane also moves the section along the span and quietly
    changes the lofted area.
    """
    # NACA coordinates run 0 = leading edge -> 1 = trailing edge, but the model
    # frame is FLU with +x FORWARD. So the section must be mirrored: the round
    # leading edge belongs at +x and the sharp trailing edge at -x.
    #
    # Writing (x - pitch_about) instead of (pitch_about - x) puts the aerofoil
    # on backwards -- a blunt trailing edge into the airflow. It still lofts, it
    # still flies in the sim (LiftDrag does not read the mesh), and it looks
    # almost right in a render, which is exactly why it is worth stating.
    # Point count must scale with chord. A propeller tip section is ~12 mm
    # across; 80 cosine-spaced points there puts neighbouring vertices closer
    # than the modelling tolerance and the wire fails to close with
    # "Face can only be created with closed wires".
    # ⚠ SIGN OF Y. The loft planes are built as
    #     Plane(origin=..., x_dir=(1,0,0), z_dir=(0,1,0))
    # whose in-plane Y axis is z_dir x x_dir = (0,0,-1). So a sketch Y of +1
    # lands at model z of -1, and feeding camber straight through puts the
    # aerofoil UPSIDE DOWN: NACA 2410's cambered surface faced downwards.
    #
    # Measured with probe_ctrl_fit.py, extruding one root section through the
    # same plane construction with no winglet or dihedral to confuse it:
    #     sketch Y  -10.2 .. +21.5 mm   ->   model z  -21.5 .. +10.2 mm
    #
    # Negating here puts camber up for everything that uses this function --
    # wing, winglet, tail, propeller blades and the control surfaces -- rather
    # than sprinkling compensating minus signs at each call site.
    #
    # This never showed up in flight because the Gazebo LiftDrag plugins read
    # coefficients and joint angles, never the mesh. It was a picture bug that
    # would have become a real one the moment anyone printed or meshed it.
    c, s = math.cos(twist), math.sin(twist)
    out = []
    for x, y in naca4(code, n=n):
        X = (pitch_about - x) * chord     # +x forward
        Y = -y * chord                    # -> +z after the plane's Y flip
        out.append((X * c - Y * s, X * s + Y * c))
    return out


# ---------------------------------------------------------------------------
# Wing
# ---------------------------------------------------------------------------

def chords() -> tuple[float, float]:
    """Root and tip chord from the mean aerodynamic chord and taper."""
    lam = P.WING_TAPER
    # For a straight-tapered wing, MAC = (2/3) c_root (1 + l + l^2) / (1 + l)
    c_root = P.WING_CHORD * 3.0 * (1.0 + lam) / (2.0 * (1.0 + lam + lam ** 2))
    return c_root, c_root * lam


def build_wing(cut: bool = True):
    """One lofted half-wing, mirrored. Taper, sweep, dihedral and washout.

    `cut=False` returns the wing WITHOUT the aileron recesses, for probes that
    need to measure how much the cutter actually removes.
    """
    c_root, c_tip = chords()
    semi = P.WING_SPAN / 2.0

    def half(sign: float):
        with BuildPart() as part:
            # Root section, in the x-z plane at y = 0.
            root_plane = Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))
            with BuildSketch(root_plane):
                with BuildLine():
                    Polyline(*section_points(P.WING_NACA, c_root, 0.0), close=True)
                make_face()

            # Tip section: swept AFT (negative x -- +x is forward), raised by
            # dihedral, washed out.
            dx = -semi * math.tan(P.WING_LE_SWEEP)
            dz = semi * math.tan(P.WING_DIHEDRAL)
            tip_plane = Plane(origin=(dx, sign * semi, dz),
                              x_dir=(1, 0, 0), z_dir=(0, 1, 0))
            with BuildSketch(tip_plane):
                with BuildLine():
                    Polyline(*section_points(P.WING_NACA, c_tip,
                                             P.WING_TWIST_TIP), close=True)
                make_face()

            loft()

        # --- winglet ---------------------------------------------------------
        # Lofted separately rather than as extra sections on the wing loft:
        # blending straight from a horizontal tip section to a 72 deg canted
        # one twists the surface badly. A distinct winglet with a visible
        # junction is both cleaner geometry and what most aircraft actually
        # have.
        h = P.WINGLET_HEIGHT_FRAC * semi
        cant = P.WINGLET_CANT
        uy, uz = sign * math.cos(cant), math.sin(cant)
        root = (dx, sign * semi, dz)
        with BuildPart() as wl:
            for frac, c, tw in ((0.0, c_tip, 0.0),
                                (1.0, c_tip * P.WINGLET_TAPER, P.WINGLET_TOE)):
                origin = (root[0] - frac * h * math.tan(P.WINGLET_SWEEP),
                          root[1] + frac * h * uy,
                          root[2] + frac * h * uz)
                plane = Plane(origin=origin, x_dir=(1, 0, 0), z_dir=(0, uy, uz))
                with BuildSketch(plane):
                    with BuildLine():
                        Polyline(*section_points(P.WING_NACA, c, tw, n=60),
                                 close=True)
                    make_face()
            loft()

        return part.part + wl.part

    wing = half(+1.0) + half(-1.0)

    # Cut the aileron recesses. Without this the wing keeps its full chord and
    # the aileron solids simply sit ON TOP of it -- two solids in the same
    # space, which renders as a panel glued to the wing with a doubled surface,
    # not as an inset control surface. The cutter is the aileron inflated by
    # CONTROL_GAP_MM so a real hinge gap remains.
    if not cut:
        return wing
    g = CONTROL_GAP_MM / 1000.0
    for sgn in (+1.0, -1.0):
        wing = wing - _aileron_cutter(sgn, g)
    return wing


# ---------------------------------------------------------------------------
# Fuselage
# ---------------------------------------------------------------------------

def _ellipse_pts(a: float, b: float, n: int = 72) -> list[tuple[float, float]]:
    """Superellipse cross-section. Slightly squared so it reads as a fuselage
    rather than a flying sausage.

    72 points, not 24. The visible faceting in the first renders was here and
    in the STL tessellation tolerance, NOT in the loft -- the surface was
    always smooth, the triangles approximating it were not.
    """
    e = 2.4
    pts = []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        ct, st = math.cos(th), math.sin(th)
        x = a * math.copysign(abs(ct) ** (2.0 / e), ct)
        y = b * math.copysign(abs(st) ** (2.0 / e), st)
        pts.append((x, y))
    return pts


def build_fuselage():
    """Loft through the stations in params.FUSELAGE_STATIONS."""
    L = P.FUSELAGE_LENGTH
    # Put the wing leading edge where params says it is, so the fuselage and
    # the wing are positioned by the same numbers.
    nose_x = P.CG_MAC_FRACTION * P.WING_CHORD + 0.30 * L

    with BuildPart() as part:
        for frac, half_h, half_w in P.FUSELAGE_STATIONS:
            x = nose_x - frac * L
            plane = Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
            with BuildSketch(plane):
                with BuildLine():
                    Polyline(*_ellipse_pts(half_w, half_h), close=True)
                make_face()
        loft()
    return part.part


# ---------------------------------------------------------------------------
# Tail
# ---------------------------------------------------------------------------

def _ruddervator_cutter(sign: float, gap: float):
    """Prism covering one ruddervator, inflated by `gap`."""
    d = P.solve()
    r = P.ruddervator_geometry(sign)
    _, uy, uz = r["axis"]
    c_root, c_tip = P.TAIL_CHORD, P.TAIL_CHORD * 0.72
    hinge = r["hinge"]
    stations = []
    for s in (r["s_mid"] - r["span"] / 2.0 - gap,
              r["s_mid"] + r["span"] / 2.0 + gap):
        f = s / d.tail_panel_span
        c = c_root + (c_tip - c_root) * f
        stations.append((
            (-P.TAIL_SURFACE_ARM + (0.25 - hinge) * P.TAIL_CHORD,
             s * uy, 0.010 + s * uz), c))
    return _hinge_prism(stations, extra_fwd=gap, depth=0.3)


def build_tail(cut: bool = True):
    """V-tail: two symmetric-section panels splayed at the derived dihedral.

    Replaces the separate horizontal stabiliser and vertical fin. Two surfaces
    instead of three means less wetted area, one fewer junction generating
    interference drag, and fewer parts. PX4 allocates it natively as
    CA_SV_CS*_TYPE 7 / 8.

    The dihedral angle is NOT a style choice -- params.solve() derives it from
    the pitch and yaw effectiveness the aircraft actually needs, via
    tan^2(gamma) = yaw_area / pitch_area. Sizing a V-tail by its raw panel area
    leaves it undersized in both axes.
    """
    d = P.solve()
    gamma = d.tail_dihedral
    span = d.tail_panel_span
    c_root = P.TAIL_CHORD
    c_tip = c_root * 0.72
    x0 = -P.TAIL_SURFACE_ARM
    z0 = 0.010

    parts = []
    for sgn in (+1.0, -1.0):
        # Panel runs outboard and upward along (0, cos g, sin g).
        uy, uz = sgn * math.cos(gamma), math.sin(gamma)
        with BuildPart() as panel:
            for frac, c in ((0.0, c_root), (1.0, c_tip)):
                origin = (x0, frac * span * uy, z0 + frac * span * uz)
                plane = Plane(origin=origin, x_dir=(1, 0, 0),
                              z_dir=(0, uy, uz))
                with BuildSketch(plane):
                    with BuildLine():
                        Polyline(*section_points(P.TAIL_NACA, c, 0.0, n=60),
                                 close=True)
                    make_face()
            loft()
        parts.append(panel.part)

    out = parts[0]
    for p in parts[1:]:
        out = out + p

    if not cut:
        return out
    # Recess the ruddervators, same reasoning as the ailerons: otherwise the
    # moving surfaces sit on top of the fixed panel as duplicate geometry.
    g = CONTROL_GAP_MM / 1000.0
    for sgn in (+1.0, -1.0):
        out = out - _ruddervator_cutter(sgn, g)
    return out


def build_prop(diameter: float, blades: int = 2):
    """A twisted, tapered propeller.

    The rotor visuals were flat black cylinders -- correct as a swept-disc
    abstraction, ugly in a render, and they read as frisbees rather than
    propellers. This builds actual blades: high pitch at the root falling off
    towards the tip, which is what a real prop looks like and what makes the
    tilt obvious on camera when a nacelle rotates.
    """
    R = diameter / 2.0
    hub_r = 0.055 * diameter
    stations = (          # (r/R, chord/R, twist deg)
        (0.16, 0.150, 34.0),
        (0.40, 0.185, 22.0),
        (0.70, 0.160, 14.0),
        (1.00, 0.095, 9.0),
    )

    with BuildPart() as blade:
        for frac, chord_f, tw in stations:
            plane = Plane(origin=(0, frac * R, 0),
                          x_dir=(1, 0, 0), z_dir=(0, 1, 0))
            with BuildSketch(plane):
                with BuildLine():
                    Polyline(*section_points("0012", chord_f * R,
                                             math.radians(tw), 0.30, n=28),
                             close=True)
                make_face()
        loft()

    part = blade.part
    out = part
    for i in range(1, blades):
        out = out + Rot(Z=360.0 * i / blades) * part

    with BuildPart() as hub:
        with BuildSketch(Plane.XY):
            with BuildLine():
                Polyline(*_ellipse_pts(hub_r, hub_r, 24), close=True)
            make_face()
        extrude(amount=0.028 * diameter, both=True)

    return out + hub.part


def build_booms():
    """The carbon booms that carry the nacelles.

    params.py places the wing rotors WING_ROTOR_AHEAD_OF_LE (100 mm) forward of
    the leading edge, and the BOM has always listed "carbon boom, 16 mm OD tube"
    for exactly this. The geometry never had them, so every render showed three
    motors hanging in mid-air with nothing holding them up -- a mounting scheme
    that exists in the parts list and nowhere in the model.

    Wing booms run fore/aft THROUGH the wing at the rotor station -- inside the
    section, not slung under it.

    ⚠ CORRECTED. The boom used to sit at `z_wing - 0.014`, i.e. 14 mm below the
    chord line, which put it almost entirely OUTSIDE the skin: measured with
    probe_wing_section.py, the section at this station runs -8.3 .. +18.4 mm
    about the chord line, and the boom spanned -22.0 .. -6.0 mm. It hung 13.7 mm
    proud in the airstream, and applied the nacelle's bending load on a lever
    arm instead of in shear through the spar.

    Buried, the boom centres on the CAMBER LINE at 30% chord (+5.0 mm), leaving
    5.4 mm of structure above and below a 16 mm tube. Both the height and the
    rear anchor are DERIVED from the section here rather than typed in, so they
    follow if the airfoil, chord or taper changes.
    """
    d = P.solve()
    c_root, c_tip = chords()
    semi = P.WING_SPAN / 2.0
    r = (P.BOOM_DIA_MM / 1000.0) / 2.0

    y = P.WING_ROTOR_Y
    frac = y / semi
    c_local = c_root + (c_tip - c_root) * frac
    z_wing = y * math.tan(P.WING_DIHEDRAL)
    # Quarter chord at this station, in model x (swept aft).
    x_qc = (P.CG_MAC_FRACTION * P.WING_CHORD - 0.25 * P.WING_CHORD) \
        - y * math.tan(P.WING_LE_SWEEP)

    # Centre the tube on the camber line at max thickness (30% chord for a
    # 4-digit section), which is the deepest part of the box.
    _, yc30 = naca_yt_yc(P.WING_NACA, 0.30)
    z_boom = z_wing + yc30 * c_local

    # The section is only deep enough to swallow the tube over part of the
    # chord. Find the AFT station where the skin would start to bulge, and stop
    # the boom there instead of letting it poke out of the thin trailing region.
    needed = 2.0 * r + 2.0 * (BOOM_MIN_SKIN_MM / 1000.0)
    x_aft_frac = chord_frac_at_thickness(P.WING_NACA, c_local, needed)

    le_x = x_qc + 0.25 * c_local
    x_front = d.wing_rotor_arm + 0.02          # just ahead of the nacelle
    x_back = le_x - x_aft_frac * c_local       # last station that can bury it
    length = x_front - x_back

    # ⚠ Assert containment HERE, where the z actually lives. params.check() can
    # only verify that the section is deep enough to hold a 16 mm tube -- which
    # was true all along. It could never catch the real defect, because the
    # boom's z was a literal in this file that params.py never saw. A check has
    # to sit where the number is.
    yt30, _ = naca_yt_yc(P.WING_NACA, 0.30)
    upper = z_wing + (yc30 + yt30) * c_local
    lower = z_wing + (yc30 - yt30) * c_local
    if not (lower + 0.0015 <= z_boom - r and z_boom + r <= upper - 0.0015):
        raise ValueError(
            f"boom is not buried in the wing: tube spans "
            f"{(z_boom - r) * 1000:+.1f}..{(z_boom + r) * 1000:+.1f} mm, "
            f"skin spans {lower * 1000:+.1f}..{upper * 1000:+.1f} mm "
            f"at y={y:.3f} m")

    with BuildPart() as part:
        for sgn in (+1, -1):
            with BuildSketch(Plane(origin=((x_front + x_back) / 2.0,
                                           sgn * y, z_boom),
                                   x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
                with BuildLine():
                    Polyline(*_ellipse_pts(r, r, 24), close=True)
                make_face()
            extrude(amount=length / 2.0, both=True)

        # --- nacelle pylon fairings ----------------------------------------
        # ⚠ Burying the tube is not sufficient on its own. The wing section is
        # only thicker than the 16 mm boom between about 8% and 62% chord, and
        # the nacelle sits WING_ROTOR_AHEAD_OF_LE (100 mm) FORWARD of the
        # leading edge. So roughly 140 mm of bare tube was always going to be
        # exposed ahead of the wing, whatever z it sits at -- which is what was
        # still visible in the render after the burial fix.
        #
        # Real aircraft in this layout do not leave a rod hanging out in front
        # of the wing; they fair it into the nacelle. This lofts a streamlined
        # pylon from the wing's own leading-edge section forward to the motor
        # mount, swallowing the tube completely.
        for sgn in (+1, -1):
            le_local = le_x
            stations = (
                # (x, half-height, half-width) along the fairing
                (le_local + 0.005, 0.043 * c_local, 0.030 * c_local),
                (le_local - 0.030 + 0.5 * (x_front - le_local), 0.038 * c_local,
                 0.026 * c_local),
                (x_front - 0.004, 0.021, 0.017),
            )
            for i, (fx, hh, hw) in enumerate(stations):
                plane = Plane(origin=(fx, sgn * y, z_boom),
                              x_dir=(0, 1, 0), z_dir=(1, 0, 0))
                with BuildSketch(plane):
                    with BuildLine():
                        Polyline(*_ellipse_pts(hw, hh, 40), close=True)
                    make_face()
            loft()

        # Tail pylon: a streamlined fin standing on the fuselage waist,
        # carrying the lift rotor clear of the boundary layer. Aerofoil
        # section, not a round tube -- it sits in the freestream for the whole
        # cruise and a cylinder there would be pure parasitic drag.
        x_rotor = -d.tail_rotor_arm
        z_base = P.fuselage_half_height_at(x_rotor) - 0.006
        z_top = P.tail_rotor_z()
        for frac, c in ((0.0, 0.115), (1.0, 0.082)):
            z = z_base + frac * (z_top - z_base)
            plane = Plane(origin=(x_rotor, 0.0, z),
                          x_dir=(1, 0, 0), z_dir=(0, 0, 1))
            with BuildSketch(plane):
                with BuildLine():
                    Polyline(*section_points("0012", c, 0.0, 0.35, n=40),
                             close=True)
                make_face()
        loft()

    return part.part


@lru_cache(maxsize=1)
def _wing_uncut():
    """The wing before the aileron recesses, cached (it is slow to loft)."""
    return build_wing(cut=False)


@lru_cache(maxsize=1)
def _tail_uncut():
    return build_tail(cut=False)


def _hinge_prism(stations, extra_fwd: float, depth: float = 0.5):
    """A big prism lying AFT of a hinge line, for splitting a surface.

    `stations` is [(origin, chord), ...] -- one per end of the hinge line, each
    origin already in the model frame at that station's hinge point.

    Why a prism and not a lofted aerofoil: the control surface must match the
    wing EXACTLY, and the wing is a loft between root and tip sections. An
    independently-built exact-NACA surface disagrees with that loft by
    fractions of a millimetre, which on a part only a few mm thick left 30% of
    it outside the wing -- measured with probe_ctrl_fit.py. Cutting the wing
    with a prism and taking the pieces guarantees a perfect fit by
    construction, whatever the loft actually did.
    """
    with BuildPart() as part:
        for origin, chord in stations:
            plane = Plane(origin=origin, x_dir=(1, 0, 0), z_dir=(0, 1, 0))
            with BuildSketch(plane):
                with BuildLine():
                    # +x forward, so "aft of the hinge" is negative x.
                    Polyline(
                        (extra_fwd, -depth), (extra_fwd, depth),
                        (-1.2 * chord, depth), (-1.2 * chord, -depth),
                        close=True)
                make_face()
        loft()
    return part.part


def _aileron_cutter(sign: float, gap: float):
    """Prism covering one aileron, inflated by `gap` in chord and span."""
    a = P.aileron_geometry()
    hinge = a["hinge"]
    _, yc_h = P.naca_yt_yc(P.WING_NACA, hinge)
    stations = []
    for y in (a["y0"] - gap, a["y1"] + gap):
        st = P.wing_station(y)
        c = st["chord"]
        stations.append((
            (st["x_qc"] + (0.25 - hinge) * c, sign * y, st["z"] + yc_h * c), c))
    return _hinge_prism(stations, extra_fwd=gap)


def build_aileron(sign: float, gap: float = 0.0, at_origin: bool = True):
    """One aileron, lofted from the real section, origin at its hinge line.

    `gap` inflates the surface slightly: chord forward of the hinge and span at
    both ends. That inflated copy is what gets SUBTRACTED from the wing, so the
    aileron sits in a recess with a visible hinge gap instead of overlapping
    the wing solid. Two coincident solids is what made the surfaces read as
    slabs glued to the wing rather than part of it.

    `at_origin=False` returns it positioned in the wing frame, ready to cut.

    Geometry comes from params.aileron_geometry() so the mesh and the SDF joint
    are placed from ONE calculation. Previously the SDF computed a box position
    from its own copy of the numbers, which is how the slabs ended up floating
    clear of a wing that is tapered, swept and dihedralled.
    """
    a = P.aileron_geometry()
    # The surface IS the wing's own aft portion, not a lookalike built beside
    # it. Intersecting guarantees the two share a surface exactly.
    solid = _wing_uncut() & _aileron_cutter(sign, gap)
    if at_origin:
        solid = solid.moved(
            Location((-a["x"], -sign * a["y_mid"], -a["z"])))
    return solid


def build_ruddervator(sign: float, gap: float = 0.0, at_origin: bool = True):
    """One V-tail ruddervator, hinged on the panel's own span axis.

    The panel lies along (0, cos g, sin g), so the moving surface lofts along
    that axis too -- not along y. Deflected together these are an elevator,
    differentially a rudder.

    Same `gap` / `at_origin` contract as build_aileron: the inflated copy is
    subtracted from the tail so the surface is inset rather than overlaid.
    """
    r = P.ruddervator_geometry(sign)
    solid = _tail_uncut() & _ruddervator_cutter(sign, gap)
    if at_origin:
        solid = solid.moved(Location((-r["x"], -r["y"], -r["z"])))
    return solid


PARTS = {
    "wing": build_wing,
    "fuselage": build_fuselage,
    "tail": build_tail,
    "booms": build_booms,
    "prop_wing": lambda: build_prop(P.WING_PROP_DIAMETER),
    "prop_tail": lambda: build_prop(P.TAIL_PROP_DIAMETER),
    # Control surfaces as real lofted aerofoil slices, not grey boxes.
    "aileron_left": lambda: build_aileron(+1.0),
    "aileron_right": lambda: build_aileron(-1.0),
    "ruddervator_left": lambda: build_ruddervator(+1.0),
    "ruddervator_right": lambda: build_ruddervator(-1.0),
}


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")
    c_root, c_tip = chords()
    print(f"NACA {P.WING_NACA}  root chord {c_root * 1000:.1f} mm  "
          f"tip {c_tip * 1000:.1f} mm  taper {P.WING_TAPER:.2f}")
    print()

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    STEP_DIR.mkdir(parents=True, exist_ok=True)

    failures = 0
    for name, builder in PARTS.items():
        try:
            part = builder()
        except Exception as exc:                       # noqa: BLE001
            print(f"  FAIL {name}: {exc}")
            failures += 1
            continue

        valid = part.is_valid
        vol = part.volume
        bb = part.bounding_box()
        status = "OK " if (valid and vol > 0) else "BAD"
        if status == "BAD":
            failures += 1
        print(f"  {status} {name:10s} vol {vol * 1e6:9.1f} cm^3  "
              f"bbox {bb.size.X:5.3f} x {bb.size.Y:5.3f} x {bb.size.Z:5.3f} m")

        # Tessellation tolerance matters more than anything else for how the
        # renders look. The default is far too coarse for a 1.2 m body: it
        # produced visible flats down the fuselage that read as bad modelling
        # when the underlying surface was fine.
        # 1e-4 m gave a beautiful wing at 42k triangles, which is over the
        # <=20k budget these meshes are supposed to respect. 2.5e-4 is
        # visually indistinguishable at render scale and roughly a quarter
        # the count.
        export_stl(part, str(MESH_DIR / f"{name}.stl"),
                   tolerance=2.5e-4, angular_tolerance=0.10)
        export_step(part, str(STEP_DIR / f"airframe_{name}.step"))

    print(f"\nmeshes -> {MESH_DIR}")
    if failures:
        raise SystemExit(f"{failures} part(s) failed")


if __name__ == "__main__":
    main()
