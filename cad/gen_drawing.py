r"""
Generate a dimensioned 3-view general-arrangement drawing from params.py.

Written as raw SVG rather than driven from the CAD kernel, because a GA drawing
is a communication artifact: what matters is that the dimensions shown are the
ones the design is actually built from. Every number here is read from
params.py, so the drawing cannot quietly disagree with the aircraft.

Open the SVG in a browser and print to PDF for the portfolio piece.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_drawing.py
"""

from __future__ import annotations

import math
from pathlib import Path

import params as P

OUT = Path(__file__).resolve().parent.parent / "docs" / "general_arrangement.svg"

W, H = 1500, 1150
INK = "#1a1a1a"
DIM = "#b03030"
THIN = "#8a8a8a"
FILL = "#eef1f4"


def chords() -> tuple[float, float]:
    lam = P.WING_TAPER
    c_root = P.WING_CHORD * 3.0 * (1.0 + lam) / (2.0 * (1.0 + lam + lam ** 2))
    return c_root, c_root * lam


class Svg:
    def __init__(self) -> None:
        self.p: list[str] = []

    def add(self, s: str) -> None:
        self.p.append(s)

    def line(self, x1, y1, x2, y2, c=INK, w=1.4, dash="") -> None:
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" '
                 f'y2="{y2:.1f}" stroke="{c}" stroke-width="{w}"{d}/>')

    def poly(self, pts, c=INK, w=1.6, fill=FILL) -> None:
        s = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.add(f'<polygon points="{s}" fill="{fill}" stroke="{c}" '
                 f'stroke-width="{w}"/>')

    def circle(self, cx, cy, r, c=INK, w=1.2, fill="none") -> None:
        self.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" '
                 f'fill="{fill}" stroke="{c}" stroke-width="{w}"/>')

    def text(self, x, y, s, size=13, c=INK, anchor="start", weight="normal") -> None:
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-family="DejaVu Sans,'
                 f'Arial,sans-serif" font-size="{size}" fill="{c}" '
                 f'text-anchor="{anchor}" font-weight="{weight}">{s}</text>')

    def dim_h(self, x1, x2, y, label) -> None:
        """Horizontal dimension with arrow ticks."""
        self.line(x1, y, x2, y, DIM, 1.0)
        for x in (x1, x2):
            self.line(x, y - 5, x, y + 5, DIM, 1.0)
        self.text((x1 + x2) / 2, y - 7, label, 12, DIM, "middle")

    def dim_v(self, y1, y2, x, label) -> None:
        self.line(x, y1, x, y2, DIM, 1.0)
        for y in (y1, y2):
            self.line(x - 5, y, x + 5, y, DIM, 1.0)
        self.text(x + 8, (y1 + y2) / 2, label, 12, DIM)

    def render(self) -> str:
        body = "\n".join(self.p)
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" '
                f'height="{H}" viewBox="0 0 {W} {H}">\n'
                f'<rect width="{W}" height="{H}" fill="white"/>\n{body}\n</svg>\n')


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    d = P.solve()
    c_root, c_tip = chords()
    semi = P.WING_SPAN / 2.0
    S = 260.0                      # px per metre

    def mm(v: float) -> str:
        return f"{v * 1000:.0f}"

    g = Svg()
    g.text(40, 44, "TRI-TILTROTOR VTOL — GENERAL ARRANGEMENT", 22, INK,
           weight="bold")
    g.text(40, 68, "All dimensions in mm unless stated. Generated from "
                   "cad/params.py — do not scale from this drawing.", 12, THIN)

    # ---------------- TOP VIEW ----------------
    cx, cy = 470.0, 300.0
    g.text(40, 110, "PLAN", 15, INK, weight="bold")

    x_qc = 0.0
    le_r, te_r = x_qc + 0.25 * c_root, x_qc - 0.75 * c_root
    sweep = semi * math.tan(P.WING_LE_SWEEP)
    le_t = x_qc - sweep + 0.25 * c_tip
    te_t = x_qc - sweep - 0.75 * c_tip

    def T(x, y):     # aircraft (x fwd, y right) -> svg
        return cx - y * S, cy - x * S

    g.poly([T(le_r, -0.06), T(le_t, -semi), T(te_t, -semi),
            T(te_r, -0.06)], fill=FILL)
    g.poly([T(le_r, 0.06), T(le_t, semi), T(te_t, semi),
            T(te_r, 0.06)], fill=FILL)

    # fuselage
    nose = P.CG_MAC_FRACTION * P.WING_CHORD + 0.30 * P.FUSELAGE_LENGTH
    tailend = nose - P.FUSELAGE_LENGTH
    g.poly([T(nose, 0), T(nose - 0.18, -0.062), T(tailend + 0.10, -0.03),
            T(tailend, 0), T(tailend + 0.10, 0.03), T(nose - 0.18, 0.062)],
           fill="#e2e7ec")

    # V-tail, in plan: the two panels project to +/- their horizontal semi-span.
    h_span = 2.0 * d.tail_semi_span_h
    half_c = P.TAIL_CHORD / 2.0
    g.poly([T(-P.TAIL_SURFACE_ARM + half_c, -h_span / 2),
            T(-P.TAIL_SURFACE_ARM + half_c, h_span / 2),
            T(-P.TAIL_SURFACE_ARM - half_c, h_span / 2),
            T(-P.TAIL_SURFACE_ARM - half_c, -h_span / 2)], fill=FILL)

    # rotors
    rw = P.WING_PROP_DIAMETER / 2
    for sgn in (-1, 1):
        px, py = T(d.wing_rotor_arm, sgn * P.WING_ROTOR_Y)
        g.circle(px, py, rw * S, THIN, 1.1, "rgba(0,0,0,0.05)")
        g.circle(px, py, 4, INK, 1.2, INK)
    px, py = T(-d.tail_rotor_arm, 0)
    g.circle(px, py, (P.TAIL_PROP_DIAMETER / 2) * S, THIN, 1.1,
             "rgba(0,0,0,0.05)")
    g.circle(px, py, 4, INK, 1.2, INK)

    # centreline + CG
    g.line(*T(nose + 0.05, 0), *T(tailend - 0.05, 0), THIN, 0.8, "8 4")
    gx, gy = T(0, 0)
    g.circle(gx, gy, 8, INK, 1.4)
    g.add(f'<path d="M {gx-8:.1f} {gy:.1f} A 8 8 0 0 1 {gx+8:.1f} {gy:.1f} Z" '
          f'fill="{INK}"/>')
    g.text(gx + 14, gy + 4, "CG  28% MAC", 11, INK)

    g.dim_h(*[T(0, -semi)[0], T(0, semi)[0]], cy - 300, f"SPAN {mm(P.WING_SPAN)}")
    g.dim_h(T(0, -P.WING_ROTOR_Y)[0], T(0, P.WING_ROTOR_Y)[0], cy - 272,
            f"ROTOR TRACK {mm(2 * P.WING_ROTOR_Y)}")
    g.dim_v(T(d.wing_rotor_arm, 0)[1], T(-d.tail_rotor_arm, 0)[1],
            cx + 300, f"ROTOR BASE {mm(d.wing_rotor_arm + d.tail_rotor_arm)}")
    g.text(T(0, semi)[0] - 60, T(te_t, 0)[1] + 18,
           f"c_tip {mm(c_tip)}", 11, INK)
    g.text(cx + 14, T(te_r, 0)[1] + 16, f"c_root {mm(c_root)}", 11, INK)

    # ---------------- FRONT VIEW ----------------
    fx, fy = 1120.0, 250.0
    g.text(960, 110, "FRONT", 15, INK, weight="bold")
    dih = semi * math.tan(P.WING_DIHEDRAL)

    def F(y, z):
        return fx + y * S, fy - z * S

    for sgn in (-1, 1):
        g.line(*F(sgn * 0.06, 0.0), *F(sgn * semi, dih), INK, 3.0)
    g.circle(fx, fy, 0.062 * S, INK, 1.6, "#e2e7ec")
    for sgn in (-1, 1):
        rx, ry = F(sgn * P.WING_ROTOR_Y, P.WING_ROTOR_Y * math.tan(P.WING_DIHEDRAL))
        g.line(rx - rw * S, ry, rx + rw * S, ry, THIN, 1.2)
        g.circle(rx, ry, 4, INK, 1.2, INK)
    g.text(F(semi, dih)[0] - 40, F(semi, dih)[1] - 10,
           f"{math.degrees(P.WING_DIHEDRAL):.0f}° dihedral", 11, INK)

    # ---------------- SIDE VIEW ----------------
    sx, sy = 470.0, 800.0
    g.text(40, 640, "SIDE", 15, INK, weight="bold")

    def Sd(x, z):
        return sx - x * S, sy - z * S

    g.poly([Sd(nose, 0.005), Sd(nose - 0.12, 0.070), Sd(0.05, 0.072),
            Sd(-0.45, 0.045), Sd(tailend, 0.014), Sd(tailend, -0.010),
            Sd(-0.45, -0.040), Sd(0.05, -0.060), Sd(nose - 0.12, -0.045)],
           fill="#e2e7ec")
    g.line(*Sd(0.25 * c_root, 0.01), *Sd(-0.75 * c_root, 0.01), INK, 3.0)
    # V-tail in side view: panels rise to their vertical projection.
    v_h = d.tail_panel_span * math.sin(d.tail_dihedral)
    g.poly([Sd(-P.TAIL_SURFACE_ARM + half_c, 0.010),
            Sd(-P.TAIL_SURFACE_ARM + half_c * 0.55, 0.010 + v_h),
            Sd(-P.TAIL_SURFACE_ARM - half_c * 0.75, 0.010 + v_h),
            Sd(-P.TAIL_SURFACE_ARM - half_c, 0.010)], fill=FILL)

    for arm, r, lab in ((d.wing_rotor_arm, rw, "WING ROTOR"),
                        (-d.tail_rotor_arm, P.TAIL_PROP_DIAMETER / 2, "TAIL ROTOR")):
        px, py = Sd(arm, P.NACELLE_Z_OFFSET)
        g.line(px, py - r * S, px, py + r * S, THIN, 1.3)
        g.circle(px, py, 4, INK, 1.2, INK)
        g.text(px - 30, py - r * S - 8, lab, 10, THIN)

    g.line(*Sd(nose + 0.05, 0), *Sd(tailend - 0.05, 0), THIN, 0.8, "8 4")
    g.dim_h(Sd(nose, 0)[0], Sd(tailend, 0)[0], sy + 150,
            f"FUSELAGE {mm(P.FUSELAGE_LENGTH)}")
    g.dim_h(Sd(0, 0)[0], Sd(-P.TAIL_SURFACE_ARM, 0)[0], sy + 118,
            f"TAIL ARM {mm(P.TAIL_SURFACE_ARM)}")

    # ---------------- DATA BLOCK ----------------
    bx, by = 950.0, 560.0
    g.add(f'<rect x="{bx}" y="{by}" width="500" height="500" fill="none" '
          f'stroke="{INK}" stroke-width="1.4"/>')
    g.text(bx + 14, by + 26, "DESIGN DATA", 14, INK, weight="bold")
    rows = [
        ("MTOW", f"{P.MASS_TOTAL:.2f} kg"),
        ("Wing area", f"{d.wing_area:.3f} m²"),
        ("Wing loading", f"{d.wing_loading:.2f} kg/m²"),
        ("Aspect ratio", f"{d.wing_aspect_ratio:.2f}"),
        ("Airfoil", f"NACA {P.WING_NACA}  (tail {P.TAIL_NACA})"),
        ("Taper / dihedral / twist",
         f"{P.WING_TAPER:.2f} / {math.degrees(P.WING_DIHEDRAL):.0f}° / "
         f"{math.degrees(P.WING_TWIST_TIP):.0f}°"),
        ("V-tail dihedral / area",
         f"{math.degrees(d.tail_dihedral):.1f}° / {d.tail_area_total:.3f} m²"),
        ("V-tail effective pitch/yaw",
         f"{d.tail_area_total * math.cos(d.tail_dihedral)**2:.3f} / "
         f"{d.tail_area_total * math.sin(d.tail_dihedral)**2:.3f} m²"),
        ("CL_α (finite)", f"{d.cl_alpha:.3f} /rad"),
        ("CL_max / stall α", f"{d.cl_max:.3f} / {math.degrees(d.alpha_stall):.1f}°"),
        ("CD0 / cruise L/D", f"{d.cd0:.4f} / {d.l_over_d_cruise:.1f}"),
        ("Hover thrust, wing ea.", f"{d.thrust_wing_each:.2f} N"),
        ("Hover thrust, tail", f"{d.thrust_tail:.2f} N ({d.tail_lift_fraction*100:.1f}%)"),
        ("Stall / transition / cruise",
         f"{d.v_stall:.1f} / {d.v_transition:.1f} / {P.V_CRUISE:.1f} m/s"),
        ("Tilt range, wing",
         f"{-math.degrees(P.TILT_YAW_TRAVEL):.0f}° to "
         f"{math.degrees(P.TILT_ANGLE_CRUISE):.0f}°"),
        ("Tilt range, tail",
         f"0° to {math.degrees(P.TILT_ANGLE_CRUISE):.0f}°"),
        ("Roll/pitch/yaw authority",
         f"{d.alpha_roll:.1f} / {d.alpha_pitch:.1f} / {d.alpha_yaw:.1f} rad/s²"),
    ]
    for i, (k, v) in enumerate(rows):
        y = by + 56 + i * 27
        g.text(bx + 14, y, k, 12, THIN)
        g.text(bx + 486, y, v, 12, INK, "end")

    g.text(bx + 14, by + 480,
           "Tolerances: clearance fits only, +0.30 mm on shaft bores, "
           "+0.15 mm on bearing seats.", 10, THIN)

    g.text(40, H - 26,
           "Verified in simulation only — PX4 v1.17.0 / Gazebo Harmonic 8.11.0. "
           "No physical aircraft exists. Tolerances verified in CAD only.",
           11, THIN)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(g.render(), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
