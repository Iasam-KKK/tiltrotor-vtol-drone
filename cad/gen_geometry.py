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
    Align, Box, BuildLine, BuildPart, BuildSketch, Cylinder, Location,
    Locations, Mode, Plane, Polyline, Vector, export_step, export_stl, extrude,
    loft, make_face, mirror, Rot,
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
        # --- BLENDED winglet --------------------------------------------------
        # ⚠ Two problems made the junction read as "barely connected", and only
        # one of them was the hard corner:
        #
        #   1. The winglet went straight from 0 to 72 deg cant in one step, so
        #      the wing and the winglet met at a crease with no blend.
        #   2. The winglet root section was drawn at ZERO twist while the wing
        #      tip carries WING_TWIST_TIP (-2 deg washout). The two sections at
        #      the same station were therefore DIFFERENT SHAPES -- the surfaces
        #      genuinely did not line up, which is why it looked detached
        #      rather than merely sharp.
        #
        # Now lofted through several stations with the cant ramping smoothly
        # (t^0.7, so it leaves the wing nearly tangent and curls up), and the
        # root section matches the wing tip exactly -- same chord, same twist,
        # zero cant. That makes it one continuous surface.
        # ⚠ ALWAYS BUILT FOR THE +y SIDE, THEN MIRRORED.
        # Building it directly for sign=-1 puts the aerofoil upside down on the
        # right winglet, and nothing about the code looks wrong. The plane's
        # in-plane Y is z_dir x x_dir:
        #     sign=+1, phi=0 -> (0,cos,sin) x (1,0,0) = (0, 0, -1)
        #     sign=-1, phi=0 -> (0,-cos,sin) x (1,0,0) = (0, 0, +1)
        # -- opposite, so the right winglet's section was flipped relative to
        # the wing it grows out of. That is why one tip looked right and the
        # other did not. Mirroring a known-good solid cannot drift.
        h = P.WINGLET_HEIGHT_FRAC * semi
        n_bl = 6
        px_, py_, pz_ = dx, semi, dz
        with BuildPart() as wl:
            for i in range(n_bl + 1):
                t = i / n_bl
                phi = P.WINGLET_CANT * (t ** 0.7)
                c_i = c_tip + (c_tip * P.WINGLET_TAPER - c_tip) * t
                tw_i = P.WING_TWIST_TIP + P.WINGLET_TOE * t
                uy_i, uz_i = math.cos(phi), math.sin(phi)
                plane = Plane(origin=(px_, py_, pz_), x_dir=(1, 0, 0),
                              z_dir=(0, uy_i, uz_i))
                with BuildSketch(plane):
                    with BuildLine():
                        Polyline(*section_points(P.WING_NACA, c_i, tw_i, n=60),
                                 close=True)
                    make_face()
                if i < n_bl:
                    # Step along the curving path using the MID-segment cant,
                    # so the discretised path follows the intended curve rather
                    # than cutting its corners.
                    ds = h / n_bl
                    phi_m = P.WINGLET_CANT * (((i + 0.5) / n_bl) ** 0.7)
                    px_ -= ds * math.tan(P.WINGLET_SWEEP)
                    py_ += ds * math.cos(phi_m)
                    pz_ += ds * math.sin(phi_m)
            loft(ruled=False)

        winglet = wl.part if sign > 0 else mirror(wl.part, about=Plane.XZ)
        return part.part + winglet

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
        wing = wing - _servo_bay(sgn)
    return wing


def _servo_bay(sign: float):
    """Pocket in the wing for one aileron servo.

    Placed at SERVO_BAY_CHORD_FRAC, forward of the hinge, because that is where
    the section is still deep enough: 19.8 mm at 55% chord against 18 mm at the
    75% hinge, for a 13.6 mm servo that needs skin on both sides.

    Lying flat -- the servo's 30 mm height runs SPANWISE, not through the
    thickness, which no 20 mm wing section could accommodate.
    """
    a = P.aileron_geometry()
    st = P.wing_station(a["y_mid"])
    c = st["chord"]
    le_x = st["x_qc"] + 0.25 * c
    x_c = le_x - P.SERVO_BAY_CHORD_FRAC * c
    _, yc_bay = P.naca_yt_yc(P.WING_NACA, P.SERVO_BAY_CHORD_FRAC)
    z_c = st["z"] + yc_bay * c

    cl = P.SERVO_MOUNT_CLEARANCE_MM / 1000.0
    ln = P.SURFACE_SERVO_L_MM / 1000.0 + cl
    ht = P.SURFACE_SERVO_H_MM / 1000.0 + cl      # spanwise
    wd = P.SURFACE_SERVO_W_MM / 1000.0 + cl      # through thickness

    # ⚠ A closed internal void is invisible AND uninstallable. The first version
    # of this cut a sealed pocket inside the wing: nothing showed in any render
    # because nothing broke the skin, and there was no way to get a servo into
    # it or a screwdriver onto it. Real bays open through the LOWER surface and
    # are closed with a cover. Extending the cut downward past the skin makes
    # the opening real.
    depth = wd + (0.04 if P.SERVO_BAY_OPENS_THROUGH else 0.0)
    z_off = -(depth - wd) / 2.0

    with BuildPart() as bay:
        with Locations((x_c, sign * a["y_mid"], z_c + z_off)):
            Box(ln, ht, depth)
    return bay.part


def _tube(p0, p1, radius, n=20):
    """A straight tube between two points."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = tuple((p0[i] + p1[i]) / 2.0 for i in range(3))
    axis = (dx / length, dy / length, dz / length)
    # Any vector not parallel to the axis works as the in-plane reference.
    ref = (0, 0, 1) if abs(axis[2]) < 0.9 else (1, 0, 0)
    with BuildPart() as t:
        plane = Plane(origin=mid, x_dir=ref, z_dir=axis)
        with BuildSketch(plane):
            with BuildLine():
                Polyline(*_ellipse_pts(radius, radius, n), close=True)
            make_face()
        extrude(amount=length / 2.0, both=True)
    return t.part


def build_structure():
    """The carbon primary structure: longerons, tail boom and wing spars.

    This is what actually carries the aircraft. The printed shells become
    fairings; loads run in carbon.

    ⚠ The longerons do NOT run the whole length. params.check() rejected that
    on the first attempt -- a 46 mm pair needs 58 mm of width and the tail boom
    is 33 mm across. They run through the wide forward body where the equipment
    bay wants the stiffness, and a SINGLE tube carries on aft. Same reason
    full-size aircraft do it.

    The wing spars are CONTINUOUS THROUGH THE CENTRELINE, angled along the
    dihedral, meeting in a spar box at F5. That is the whole point: the joint
    is not at the root, where the bending moment peaks. Each panel sleeves onto
    the protruding spar and is pinned, so the pin only ever sees shear.
    """
    nose = P.fuselage_nose_x()
    r_lon = P.LONGERON_DIA_MM / 2000.0
    r_boom = P.TAILBOOM_DIA_MM / 2000.0
    r_spar = P.WING_SPAR_DIA_MM / 2000.0
    z_lon = P.LONGERON_Z_MM / 1000.0
    y_lon = P.LONGERON_SPACING_MM / 2000.0

    parts = []

    # --- fuselage longerons, nose to the boom junction ---
    for sgn in (+1, -1):
        parts.append(_tube((nose - 0.030, sgn * y_lon, z_lon),
                           (P.LONGERON_AFT_X, sgn * y_lon, z_lon), r_lon))

    # --- single tail boom, overlapping the pair for a bonded splice ---
    parts.append(_tube((P.LONGERON_AFT_X + 0.090, 0.0, z_lon),
                       (-P.TAIL_SURFACE_ARM - 0.030, 0.0, z_lon), r_boom))

    # --- wing spars, continuous through the centreline along the dihedral ---
    semi = P.WING_SPAN / 2.0
    y_tip = P.WING_SPAR_SPAN_FRAC * semi
    for frac in (P.WING_SPAR_CHORD_FRAC, P.WING_SPAR_REAR_CHORD_FRAC):
        _, yc = P.naca_yt_yc(P.WING_NACA, frac)
        for sgn in (+1, -1):
            st_t = P.wing_station(y_tip)
            x_t = st_t["x_qc"] + (0.25 - frac) * st_t["chord"]
            z_t = st_t["z"] + yc * st_t["chord"]
            st_r = P.wing_station(0.0)
            x_r = st_r["x_qc"] + (0.25 - frac) * st_r["chord"]
            z_r = st_r["z"] + yc * st_r["chord"]
            # Model frame: the wing mesh is offset by quarter_x downstream, so
            # apply it here too or the spars sit ahead of their own wing.
            qx = P.wing_root_quarter_chord_x()
            parts.append(_tube((qx + x_r, 0.0, z_r),
                               (qx + x_t, sgn * y_tip, z_t), r_spar))

    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def build_formers():
    """Printed bulkheads threaded onto the longerons.

    These ARE the internal mounts: equipment straps to them, they hold the
    shell's shape, and bonded to the rods they turn two tubes plus a skin into
    a semi-monocoque. Each is the fuselage section at its station, lightened,
    with holes for whatever passes through it.
    """
    t = P.FORMER_THICK_MM / 1000.0
    rim = P.FORMER_RIM_MM / 1000.0
    r_lon = P.LONGERON_DIA_MM / 2000.0
    y_lon = P.LONGERON_SPACING_MM / 2000.0
    z_lon = P.LONGERON_Z_MM / 1000.0

    parts = []
    for _name, x in P.FORMERS:
        hh = P.fuselage_half_height_at(x)
        hw = P.fuselage_half_width_at(x)
        with BuildPart() as f:
            plane = Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
            with BuildSketch(plane):
                with BuildLine():
                    Polyline(*_ellipse_pts(hw, hh, 48), close=True)
                make_face()
            extrude(amount=t / 2.0, both=True)
            # Lightening hole -- a former is a ring, not a disc, or it is just
            # ballast.
            if hw > rim * 2.2 and hh > rim * 2.2:
                with BuildSketch(plane):
                    with BuildLine():
                        Polyline(*_ellipse_pts(hw - rim, hh - rim, 40),
                                 close=True)
                    make_face()
                extrude(amount=t, both=True, mode=Mode.SUBTRACT)
            # Longeron holes, back in the rim.
            for sgn in (+1, -1):
                with BuildSketch(Plane(origin=(x, sgn * y_lon, z_lon),
                                       x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
                    with BuildLine():
                        Polyline(*_ellipse_pts(r_lon + 0.0004,
                                               r_lon + 0.0004, 20), close=True)
                    make_face()
                extrude(amount=t, both=True, mode=Mode.SUBTRACT)
        parts.append(f.part)

    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def _control_horn(origin, normal, chord_dir):
    """A real control horn: bolt flange + drilled blade.

    Built in a canonical frame (+X along the chord, +Z standing off the
    surface) and then placed, which is far less error-prone than trying to
    orient boxes directly -- the same class of mistake that flipped the
    right-hand winglet and one of the ruddervator horns.

    Returns (solid, hole_point) where hole_point is the OUTERMOST hole, i.e.
    where the pushrod actually attaches. Returning it means the rod cannot be
    drawn to a point the horn does not have.
    """
    h = P.CONTROL_HORN_H_MM / 1000.0
    t = P.CONTROL_HORN_T_MM / 1000.0
    bl = P.HORN_BASE_L_MM / 1000.0
    bw = P.HORN_BASE_W_MM / 1000.0
    bt = P.HORN_BASE_T_MM / 1000.0
    blade_l = P.HORN_BLADE_L_MM / 1000.0
    hole_r = P.HORN_HOLE_DIA_MM / 2000.0

    with BuildPart() as horn:
        # Flange, lying on the surface.
        with Locations((0, 0, bt / 2.0)):
            Box(bl, bw, bt)
        # Blade, standing off it.
        with Locations((-0.001, 0, bt + h / 2.0)):
            Box(blade_l, t, h)
        # Adjustment holes, drilled across the blade.
        for f in P.HORN_HOLES:
            with Locations(Location((-blade_l * 0.28, 0, bt + h * f),
                                    (90, 0, 0))):
                Cylinder(radius=hole_r, height=t * 3.0, mode=Mode.SUBTRACT)
    solid = horn.part

    pl = Plane(origin=origin, x_dir=chord_dir, z_dir=normal)
    outer = P.HORN_HOLES[-1]
    # from_local_coords, not Location * Vector -- build123d has no operator for
    # transforming a point by a Location, and the error it raises for trying
    # says nothing useful.
    hole_pt = pl.from_local_coords(
        Vector(-blade_l * 0.28, 0, bt + h * outer))
    return solid.moved(pl.location), (hole_pt.X, hole_pt.Y, hole_pt.Z)


def build_linkages():
    """Control horns and pushrods -- the visible half of every servo run.

    Without these the aircraft has servos hidden in bays connected to control
    surfaces by nothing at all. Each run is: servo horn (in the bay) ->
    pushrod -> control horn (on the moving surface).

    Drawn at neutral. They are cosmetic in the simulator -- LiftDrag reads the
    joint angle, never the linkage -- but they are what makes the mechanism
    legible, and their absence is what made the servo bays look like they were
    not there.
    """
    a = P.aileron_geometry()
    st = P.wing_station(a["y_mid"])
    c = st["chord"]
    le_x = st["x_qc"] + 0.25 * c
    x_servo = le_x - P.SERVO_BAY_CHORD_FRAC * c
    _, yc_bay = P.naca_yt_yc(P.WING_NACA, P.SERVO_BAY_CHORD_FRAC)
    z_servo = st["z"] + yc_bay * c

    horn_h = P.CONTROL_HORN_H_MM / 1000.0
    horn_t = P.CONTROL_HORN_T_MM / 1000.0
    rod_r = P.PUSHROD_DIA_MM / 2000.0

    parts = []
    for sgn in (+1.0, -1.0):
        y = sgn * a["y_mid"]
        # Both horns stand DOWN off the lower surface; the chord direction is
        # +x. Real horns, with a bolt flange and drilled blade, so the rod
        # attaches at a hole that actually exists.
        nrm = (0.0, 0.0, -1.0)
        chord = (1.0, 0.0, 0.0)

        srv, srv_hole = _control_horn((x_servo, y, z_servo - 0.006), nrm, chord)
        parts.append(srv)
        ctl, ctl_hole = _control_horn((a["x"] - 0.006, y, a["z"] - 0.004),
                                      nrm, chord)
        parts.append(ctl)
        parts.append(_tube(srv_hole, ctl_hole, rod_r, n=12))

    # --- ruddervator runs ---------------------------------------------------
    # Servos are surface-mounted on the fuselage at the tail root, arms facing
    # aft, with pushrods out to horns near the inboard end of each ruddervator.
    # Longer than the aileron run by construction -- that is the price of a
    # servo you can actually unscrew and replace -- and the buckling check
    # confirms a 2 mm rod still carries it with margin.
    rod_r_t = P.RUDDERVATOR_ROD_DIA_MM / 2000.0
    d_rv = P.solve()
    for sgn in (+1.0, -1.0):
        rv = P.ruddervator_geometry(sgn)
        st = _tail_servo_station(sgn)
        _, uy, uz = rv["axis"]

        # Servo body, sitting proud of the skin.
        sl = P.TAIL_SERVO_L_MM / 1000.0
        sw = P.TAIL_SERVO_W_MM / 1000.0
        sh = P.TAIL_SERVO_H_MM / 1000.0
        with BuildPart() as body:
            with Locations(st["origin"]):
                Box(sl, sw, sh)
        parts.append(body.part)

        # Output arm, standing outboard off the case.
        arm_o = (st["origin"][0] - sl * 0.35,
                 st["origin"][1] + sgn * sw * 0.5,
                 st["origin"][2])
        srv, srv_hole = _control_horn(arm_o, (0.0, sgn * 1.0, 0.0),
                                      (1.0, 0.0, 0.0))
        parts.append(srv)

        # Horn on the ruddervator, near its inboard end.
        s_h = P.TAIL_HORN_SPAN_FRAC * d_rv.tail_panel_span
        nrm = (0.0, sgn * uz, -abs(uy))
        c_base = (rv["x"] - 0.010, s_h * uy, 0.010 + s_h * uz)
        ctl, ctl_hole = _control_horn(c_base, nrm, (1.0, 0.0, 0.0))
        parts.append(ctl)

        parts.append(_tube(srv_hole, ctl_hole, rod_r_t, n=12))

    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def _tail_servo_station(sign: float) -> dict:
    """Where the ruddervator servo sits: SURFACE-MOUNTED on the fuselage.

    ⚠ Was inside the V-tail panel. It fits there (13.0 mm of section against a
    9.6 mm servo) but that is not how these are built, and more importantly a
    servo laminated inside a tail panel cannot be replaced. Real V-tails of
    this size carry both servos screwed to the fuselage skin at the tail root,
    bodies proud, arms facing aft.
    """
    d = P.solve()
    rv = P.ruddervator_geometry(sign)
    _, uy, uz = rv["axis"]
    x = P.TAIL_SERVO_MOUNT_X
    hw = P.fuselage_half_width_at(x)
    w = P.TAIL_SERVO_W_MM / 1000.0
    # Body sits ON the skin, so its centre is half a case outboard of it.
    y = sign * (hw + w / 2.0)
    return dict(x=x, s=P.TAIL_HORN_SPAN_FRAC * d.tail_panel_span,
                uy=uy, uz=uz, half_w=hw,
                origin=(x, y, 0.004),
                normal=(0.0, sign * 1.0, 0.0))


def _tail_servo_bay(sign: float):
    """Shallow recess in the fuselage skin so the servo sits flush-ish.

    The servo is external now, so this is a seat and a wire pass-through, not
    a pocket that swallows the body.
    """
    st = _tail_servo_station(sign)
    cl = P.SERVO_MOUNT_CLEARANCE_MM / 1000.0
    ln = P.TAIL_SERVO_L_MM / 1000.0 + cl
    ht = P.TAIL_SERVO_H_MM / 1000.0 + cl
    depth = 0.004
    with BuildPart() as bay:
        with Locations((st["x"], sign * (st["half_w"] - depth / 2.0), 0.004)):
            Box(ln, depth, ht)
    return bay.part

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
    # The ruddervator servos moved into the V-tail panels, so the fuselage no
    # longer carries bays for them.
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
                        Polyline(*section_points(P.TAIL_NACA, c,
                                                 P.TAIL_INCIDENCE, n=60),
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
        # Servo bay in the fixed part of the panel, same arrangement as the
        # aileron bay in the wing.
        out = out - _tail_servo_bay(sgn)
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
    x_qc = P.wing_root_quarter_chord_x() - y * math.tan(P.WING_LE_SWEEP)

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
    # Control horns and pushrods -- the visible half of each servo run.
    "linkages": build_linkages,
    # Carbon primary structure and the printed formers threaded onto it.
    "structure": build_structure,
    "formers": build_formers,
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
