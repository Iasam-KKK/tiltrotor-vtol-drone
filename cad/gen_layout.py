r"""
Emit docs/equipment-bay.svg — a dimensioned internal layout drawing.

Answers "where does the electronics actually go" with arithmetic against the
lofted fuselage section, not by eye. The body outline is drawn from the SAME
FUSELAGE_STATIONS table the CAD lofts, so the drawing cannot show a bay the
aircraft does not have.

Every box is checked by params.check() against the section at its own station
before this runs, so anything drawn here is known to fit.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_layout.py
"""

from __future__ import annotations

import math
from pathlib import Path

import params as P

OUT = Path(__file__).resolve().parent.parent / "docs" / "equipment-bay.svg"

SCALE = 420.0          # px per metre
MARGIN = 90.0
COLORS = {
    "Nose camera": "#c94f4f",
    "GPS / compass": "#4f7fc9",
    "Flight controller": "#3f9d5a",
    "Airspeed sensor": "#8a6fc9",
    "Battery 6S": "#c98f3f",
    "ESC x3": "#4fa8b8",
    "BEC": "#a8574f",
}


def profile() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Upper and lower fuselage outlines in model coordinates."""
    nose = P.fuselage_nose_x()
    upper, lower = [], []
    for frac, hh, _hw in P.FUSELAGE_STATIONS:
        x = nose - frac * P.FUSELAGE_LENGTH
        upper.append((x, +hh))
        lower.append((x, -hh))
    return upper, lower


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    upper, lower = profile()
    xs = [x for x, _ in upper]
    x_max, x_min = max(xs), min(xs)
    z_max = max(abs(z) for _, z in upper) * 2.4

    W = (x_max - x_min) * SCALE + 2 * MARGIN
    H = z_max * 2 * SCALE + 2 * MARGIN + 120

    def px(x: float) -> float:
        return MARGIN + (x_max - x) * SCALE

    def pz(z: float) -> float:
        return MARGIN + 60 + (z_max - z) * SCALE

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" '
         f'height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}">',
         '<style>'
         'text{font-family:DejaVu Sans,Arial,sans-serif}'
         '.t{font-size:11px;fill:#222}.s{font-size:9px;fill:#666}'
         '.h{font-size:15px;font-weight:600;fill:#111}'
         '.d{stroke:#999;stroke-width:0.8;fill:none}'
         '</style>',
         f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fbfbfb"/>',
         f'<text class="h" x="{MARGIN:.0f}" y="34">Tri-tiltrotor — equipment bay, '
         f'side view (looking from port)</text>',
         f'<text class="s" x="{MARGIN:.0f}" y="52">Generated from cad/params.py. '
         f'Fuselage outline lofted through {len(P.FUSELAGE_STATIONS)} stations. '
         f'All dimensions mm. +x forward.</text>']

    # --- fuselage outline ---
    pts = " ".join(f"{px(x):.1f},{pz(z):.1f}" for x, z in upper)
    pts += " " + " ".join(f"{px(x):.1f},{pz(z):.1f}"
                          for x, z in reversed(lower))
    s.append(f'<polygon points="{pts}" fill="#e9eef2" stroke="#5a6b78" '
             f'stroke-width="1.6"/>')

    # --- centreline / CG ---
    s.append(f'<line class="d" x1="{px(x_max):.1f}" y1="{pz(0):.1f}" '
             f'x2="{px(x_min):.1f}" y2="{pz(0):.1f}" stroke-dasharray="6 4"/>')
    s.append(f'<circle cx="{px(0):.1f}" cy="{pz(0):.1f}" r="6" fill="none" '
             f'stroke="#c00" stroke-width="1.4"/>')
    s.append(f'<line x1="{px(0):.1f}" y1="{pz(0) - 6:.1f}" x2="{px(0):.1f}" '
             f'y2="{pz(0) + 6:.1f}" stroke="#c00" stroke-width="1.4"/>')
    s.append(f'<text class="t" x="{px(0) + 9:.1f}" y="{pz(0) - 10:.1f}" '
             f'fill="#c00">CG (x=0)</text>')

    # --- equipment boxes ---
    for name, x_c, ln, wd, ht, m in P.EQUIPMENT:
        x0, x1 = x_c + ln / 2000.0, x_c - ln / 2000.0
        h = ht / 1000.0
        col = COLORS.get(name, "#888")
        s.append(f'<rect x="{px(x0):.1f}" y="{pz(h / 2):.1f}" '
                 f'width="{(x0 - x1) * SCALE:.1f}" height="{h * SCALE:.1f}" '
                 f'fill="{col}" fill-opacity="0.75" stroke="{col}" '
                 f'stroke-width="1.2" rx="2"/>')

    # --- callouts, stacked below so they never overlap the body ---
    y = pz(-z_max) + 26
    s.append(f'<text class="t" x="{MARGIN:.0f}" y="{y:.0f}" '
             f'style="font-weight:600">Bay contents — L x W x H, mass, station '
             f'from CG</text>')
    y += 6
    for name, x_c, ln, wd, ht, m in P.EQUIPMENT:
        y += 17
        col = COLORS.get(name, "#888")
        half_w = P.fuselage_half_width_at(x_c) * 1000.0
        half_h = P.fuselage_half_height_at(x_c) * 1000.0
        s.append(f'<rect x="{MARGIN:.0f}" y="{y - 9:.0f}" width="11" '
                 f'height="11" fill="{col}" fill-opacity="0.75" stroke="{col}"/>')
        s.append(
            f'<text class="t" x="{MARGIN + 18:.0f}" y="{y:.0f}">'
            f'{name} — {ln:.0f} x {wd:.0f} x {ht:.0f} mm, {m * 1000:.0f} g, '
            f'x = {x_c * 1000:+.0f} mm '
            f'<tspan class="s">(bay there: {2 * half_w:.0f} x {2 * half_h:.0f} mm)'
            f'</tspan></text>')
        # Leader from the box to its row.
        s.append(f'<line class="d" x1="{px(x_c):.1f}" y1="{pz(0):.1f}" '
                 f'x2="{MARGIN + 12:.0f}" y2="{y - 4:.0f}" '
                 f'stroke-dasharray="2 3" stroke="{col}" stroke-opacity="0.5"/>')

    y += 26
    total = sum(m for *_, m in P.EQUIPMENT)
    d = P.solve()
    s.append(f'<text class="t" x="{MARGIN:.0f}" y="{y:.0f}">'
             f'Equipment total {total * 1000:.0f} g of a '
             f'{(P.MASS_BATTERY + P.MASS_AVIONICS + P.MASS_PAYLOAD) * 1000:.0f} g '
             f'budget. Fuselage {P.FUSELAGE_LENGTH * 1000:.0f} mm long, '
             f'max section {2 * max(h for _, h, _ in P.FUSELAGE_STATIONS) * 1000:.0f} mm '
             f'tall, fineness {d.fineness_ratio:.1f}.</text>')
    y += 17
    s.append(f'<text class="s" x="{MARGIN:.0f}" y="{y:.0f}">'
             f'Servos are NOT in the fuselage: 4 control-surface servos sit in '
             f'the wing and V-tail bays, 2 tilt servos in the nacelle yokes. '
             f'Servo rail needs '
             f'{(P.N_SERVO_SURFACE + P.N_SERVO_TILT) * P.SERVO_STALL_CURRENT_A:.0f} A '
             f'peak, hence the separate BEC.</text>')

    s.append('</svg>')
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(s), encoding="utf-8")
    print(f"wrote {OUT}")
    print(f"  {len(P.EQUIPMENT)} items, {total * 1000:.0f} g total")


if __name__ == "__main__":
    main()
