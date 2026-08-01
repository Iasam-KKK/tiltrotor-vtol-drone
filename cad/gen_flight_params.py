r"""
Emit sim/ros2/flight_params.json — the numbers the ROS 2 nodes need.

The teleop node runs under WSL's system python inside a ROS 2 workspace; it
cannot import params.py, which lives in a Windows-side 3.12 venv with
build123d. Without this file the glide speed and glide angle would have to be
retyped into the node, and the moment params.py changed they would disagree --
exactly the drift the whole generated-from-one-source structure exists to
prevent.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_flight_params.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import params as P

OUT = Path(__file__).resolve().parent.parent / "sim" / "ros2" / "flight_params.json"


def build() -> dict:
    d = P.solve()
    return {
        "generated_from": "cad/params.py",
        "note": "DO NOT EDIT. Regenerate with cad/gen_flight_params.py",
        "envelope": {
            "v_stall_ms": round(d.v_stall, 3),
            "v_transition_ms": round(d.v_transition, 3),
            "v_cruise_ms": P.V_CRUISE,
        },
        "glide": {
            # Best L/D is where induced drag equals parasite drag:
            #   CL = sqrt(CD0 pi e AR),  (L/D)max = 0.5 sqrt(pi e AR / CD0)
            # Derived in params.solve(), never asserted -- which is what lets
            # verify_glide.sh be a test of the polar rather than a demo.
            "l_over_d_max": round(d.l_over_d_max, 3),
            "v_best_glide_ms": round(d.v_best_glide, 3),
            "glide_angle_deg": round(math.degrees(d.glide_angle), 3),
            "sink_rate_ms": round(d.sink_rate, 3),
            # For contrast: L/D at the cruise speed is necessarily lower,
            # because cruise is off the best-glide point.
            "l_over_d_at_cruise": round(d.l_over_d_cruise, 3),
        },
        "tilt": {
            "hover_deg": round(math.degrees(P.TILT_ANGLE_HOVER), 3),
            "cruise_deg": round(math.degrees(P.TILT_ANGLE_CRUISE), 3),
            "yaw_travel_deg": round(math.degrees(P.TILT_YAW_TRAVEL), 3),
        },
    }


def main() -> None:
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    data = build()
    OUT.write_text(json.dumps(data, indent=2), encoding="utf-8")
    g = data["glide"]
    print(f"wrote {OUT}")
    print(f"  best L/D {g['l_over_d_max']:.2f} at {g['v_best_glide_ms']:.2f} m/s")
    print(f"  glide angle {g['glide_angle_deg']:.2f} deg, "
          f"sink {g['sink_rate_ms']:.3f} m/s")


if __name__ == "__main__":
    main()
