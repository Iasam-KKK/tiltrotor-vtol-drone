# Tri-Tiltrotor VTOL — PX4 + Gazebo Harmonic

![Tri-tiltrotor transitioning from hover to forward flight](media/hero.gif)

A **three-rotor tilt-rotor VTOL** for PX4 SITL, built to the layout production
aircraft in this class actually use: two tilting rotors on the wing ahead of the
CG, and one **fixed, hover-only lift rotor** on a pylon at a waisted fuselage
station, ahead of a V-tail.

PX4 ships a **quad** tiltrotor (four motors, two of them tilting). This is a
**tri** — one fewer motor, ESC and propeller — and upstream provides no model
for it.

**Why the lift rotor is fixed, not tilting.** A tilting tail rotor keeps
producing thrust in cruise, but it also lays a wake over whatever sits behind
it. A hover-only rotor is stopped in cruise, so the V-tail behind it flies in
clean air in the only regime where tail effectiveness matters for stability.
The fuselage narrows **57%** ahead of the pylon so the rotor draws air already
squeezed clear of most of the boundary layer. The cost is a stopped disc in
cruise — about 1 N against a 3.3 N requirement — which is why the BOM specifies
a **folding** propeller, and why `check()` refuses a fixed rotor without one.

```bash
git clone <this repo> && cd 04-tiltrotor-vtol
bash sim/install_into_px4.sh ~/PX4-Autopilot   # symlink airframe + model
cd ~/PX4-Autopilot && make px4_sitl gz_tri_tiltrotor
# then at the pxh> prompt:
#   commander takeoff
#   commander transition
```

| Hover — rotors up | Cruise — rotors forward |
|---|---|
| ![hover](media/renders/hover_hero.png) | ![cruise](media/renders/cruise_hero.png) |

Both rendered from the **same CAD**, at the two ends of the tilt range. Nothing
was moved by hand: `render/assembly.json` is generated from `cad/params.py`, so
the pictures and the simulator are driven by one set of numbers.

```bash
powershell -File render/render_all.ps1     # 12 renders, ~6 s each on GPU
```

---

## Verified, with numbers

Nothing below is asserted. Each row is produced by a script in `sim/` that you
can re-run.

| Claim | Measurement | How |
|---|---|---|
| Design closes | **87/87** invariants | `cad/params.py` |
| Model is valid | **13/13** | `sim/validate_model.sh` |
| CAD is manufacturable | STEP round-trip **0.000000 mm³** | `cad/gen_nacelle.py` |
| **It hovers** | climbed to **10.071 m** vs 10.0 m commanded | `sim/verify_hover.sh` |
| **It transitions** | both tilting nacelles **0.5° → 89.5°** | `sim/verify_transition.sh` |
| Control surfaces fit the wing | cutter **100.0%** inside the wing solid | `cad/probe_ctrl_fit.py` |
| Every avionics box fits its station | 7 items vs the lofted section | `cad/params.py` |
| **It glides** | measured L/D vs the derived polar | `sim/verify_glide.sh` |

⚠️ The hover and transition rows above were measured **before** the pitch and
geometry corrections below. They have not been re-run since. Treat them as
evidence the mechanism works, not as current numbers.

Nacelle angle through the manoeuvre, read from Gazebo's joint state rather than
from a log line (0° = thrust up, 90° = thrust forward):

```
tilt_left    37.1   0.5   3.1   1.4   3.5   3.7  38.0  89.5  89.5  89.5
tilt_right   37.1   0.5   7.6   5.8   7.9   8.0  41.5  89.6  89.6  89.6
             spawn  ├──── hover ────┤ ├rotating┤ ├─── cruise ───┤
```

The fixed lift rotor is emitted as a `fixed` joint, so it correctly never
appears in this data — and all 3 rotors are confirmed spinning.

⚠️ These numbers were re-measured after the airframe changed. An earlier
revision with a *tilting* tail rotor was verified separately; those results are
not carried forward here, because a different rotor layout, tail and mass
distribution is a different aircraft.

---

## The aircraft

| | |
|---|---|
| MTOW / span / wing area | 4.80 kg · 2.00 m · 0.520 m² |
| Wing loading · aspect ratio | 9.23 kg/m² · 7.69 (**8.31 effective**, winglets) |
| Airfoil | NACA 2410, taper 0.65, 4° dihedral, −2° washout, winglets |
| Rotor stations from CG | wing **+0.173 m** (tilting), lift rotor **−0.700 m** (fixed) |
| Tail | V-tail at **−0.870 m**, 36.4° dihedral, 0.128 m² |
| Hover trim (solved) | wing 19.19 N each, lift rotor 9.32 N (**19.8%** of lift) |
| Fuselage | 1.55 m, fineness **12.7**, waisted 70% at the pylon |
| Stall → transition → cruise | 10.92 → 14.20 → 19.0 m/s |
| Cruise L/D · best L/D | **14.1** at 19 m/s · **15.65** at 15.01 m/s |
| Unpowered glide | **3.66°**, sink **0.957 m/s** |
| Servos · peak servo current | 4 surface + 2 tilt · **15 A** (needs its own BEC) |
| Payload | 1080p nose camera, 45 g, 78° HFOV, 15° down |

### Why the rotors straddle the CG

A layout with both tilt rotors *behind* the CG cannot be trimmed in hover: total
lift acts aft of the centre of mass and produces an unopposed nose-down moment
of W·L that no combination of thrust and tilt can cancel. Putting rotors on both
sides of the CG is what makes hover pitch controllable, and it is enforced as an
invariant (`rotors straddle the CG`) rather than left to inspection.

### Yaw is single-path, deliberately

The wing pair carries yaw by differential tilt (ArduPilot calls the equivalent
`Q_TILT_TYPE=2`), travelling 15° aft of vertical. The tail nacelle **cannot**
contribute yaw — a centreline force in the x–z plane produces no moment about z
— so it carries pitch instead (`CA_SV_TL2_CT=2`). Yaw authority is 4.96 rad/s²
from wing vectoring alone.

---

## One source of truth

Every dimension lives in [`cad/params.py`](cad/params.py) and nowhere else.
The Gazebo model, the PX4 airframe, the lofted geometry and the printed nacelle
are all *generated* from it, so they cannot drift apart.

```
cad/params.py ──┬─→ gen_sdf.py           → sim/models/tri_tiltrotor/model.sdf
                ├─→ gen_airframe.py      → sim/airframes/4030_gz_tri_tiltrotor
                ├─→ gen_geometry.py      → lofted wing / fuselage / tail /
                │                          control surfaces / booms / props
                ├─→ gen_nacelle.py       → printable cradle, yoke, fit coupon
                ├─→ gen_manifest.py      → render/assembly.json  (Blender)
                ├─→ gen_flight_params.py → sim/ros2/flight_params.json  (ROS 2)
                ├─→ gen_layout.py        → docs/equipment-bay.svg
                └─→ gen_bom.py           → docs/BOM.csv
```

`gen_flight_params.py` exists because the ROS 2 teleop node runs under WSL's
system Python and cannot import `params.py` (Windows-side 3.12 venv with
build123d). Without it the glide speed would have to be retyped into the node
and would drift the moment the polar changed.

## Running it

See [`RUN.md`](RUN.md) — three commands, plus the environment traps that make
the difference between "it works" and "it silently does nothing".

`params.check()` runs 50 invariants in arithmetic *before* any CAD kernel or
simulator starts, and refuses to emit an aircraft that cannot fly. It has
already caught, on real runs:

- a mass budget short by **0.88 kg** (battery folded into "fuselage")
- a horizontal tail sitting **inside the tail rotor's disc**, which would have
  flown in its own wash
- its own bad arithmetic — a bolt-pattern check that double-counted and failed
  a design that was fine

### Six defects this structure caught, and one it didn't

Every one of these passed the invariant suite as it stood. They are listed with
what was wrong, how it was actually found, and what now prevents a repeat —
because "we have 87 checks" is worth nothing next to what the checks missed.

| Defect | Found by | Why the checks missed it |
|---|---|---|
| **Aerofoil built upside down.** `Plane(x_dir=(1,0,0), z_dir=(0,1,0))` has in-plane Y = `(0,0,-1)`, so NACA 2410's camber pointed at the ground on the wing, winglets, tail and propeller blades. | Extruding one root section in isolation: sketch Y `-21.5..+10.2` → model z `-10.2..+21.5` | Nothing inspected the mesh. `LiftDrag` reads coefficients and joint angles, never geometry, so it flew correctly while looking wrong. A first probe read the wing **bounding box** and concluded the opposite — the winglet at +152 mm dominates it. |
| **Wing boom slung under the wing.** Centred 14 mm below the chord line, spanning −22.0…−6.0 mm against a skin at −8.3…+18.4 mm: 13.7 mm proud, applying nacelle load on a lever arm instead of in shear. | Measuring the section at the boom station | The boom's `z` was a literal in `gen_geometry.py` that `params.py` never saw. Now asserted **where the number lives**, not in `check()`. |
| **Control surfaces overlapping the wing.** Built as separate solids while the wing kept its full chord — two solids in one space. | Volume arithmetic: the wing lost 57.5 cm³ to a 478 cm³ cutter | No check compared the surfaces to the wing. Now the surface *is* the wing's own aft portion (`wing ∩ prism`), 100% by construction. |
| **V-tail bolted to nothing.** The body ended at −0.872 m; the tail root chord runs to −1.005 m. 133 mm of a 180 mm root cantilevered into open air. | Reading the station table against the tail chord | The invariant was called *"fuselage is long enough to carry the tail"* and compared **two lengths** (1.35 vs 0.956 m). It never asked where anything was. |
| **Takeoff nose-up.** `CA_ROTOR*_CT` defaults to 6.5 for every rotor; ours are 38 N and 25 N. PX4 asked the tail for 9.32 N and got ~66%, leaving ~2.2 N·m of unopposed nose-up at 0.700 m aft. | PX4's own `module.yaml`: *"Thrust = CT · u²"* | Stock `4020_gz_tiltrotor` omits `CT` too — correctly, because its four rotors are identical and the error is common-mode. Ours are not. |
| **CG 11.2 mm off.** `base_link`'s inertial sat at the origin while five links hung mass elsewhere; every `CA_ROTOR*_PX` is quoted from the CG. | Summing m·x across the emitted SDF | `params.check()` verified the *design* CG arithmetic. Nothing verified that the *generated SDF* put mass where the design said. |
| **BOM had no control-surface servos.** Listed 3 tilt servos, zero aileron or ruddervator servos, while PX4 has allocated four (`CA_SV_CS0..3`) since the V-tail was adopted. | Reading the BOM against the airframe | Nothing cross-checked the parts list against the control allocation. An aircraft built to that list would have had no roll, pitch or yaw in forward flight. |

The pattern worth taking away: **five of the seven were checks whose *name*
claimed a guarantee their *arithmetic* could not deliver.** A check with a
constant on both sides (`3 <= 4`) cannot fail, and cannot help.

### Aerodynamics are derived, not typed in

Coefficients come from the NACA 2412 section via Prandtl, not from judgement:

| | Hand-guessed | Derived |
|---|---|---|
| CL_α | 5.20 /rad | **4.742** |
| CL_max | 1.20 | **1.305** |
| CD0 | 0.028 | **0.0219** |
| Stall | 15° asserted | **13.7°**, follows from CL_max/CL_α |

---

## Against what PX4 ships

| | stock `gz_tiltrotor` | this |
|---|---|---|
| Configuration | quad, 2 of 4 tilt | **tri**, 2 tilting + 1 fixed lift |
| Motors / ESCs / props | 4 | **3** |
| Fuselage | one box, 0.55 × 2.144 × 0.05 m | lofted through 15 stations, **waisted** at the pylon |
| Main wing lift surface | **none** — elevons stand in for the wing | per-panel, at computed centres of pressure |
| Wing | rectangular plank | NACA 2410, tapered, dihedral, washout, **winglets** |
| Tail | separate stabiliser + fin | **V-tail**, dihedral derived from required effectiveness |
| Rudder | ±0.01 rad, LiftDrag **commented out** | functional, both ruddervators live |
| Airspeed sensor | absent | present (required to gate transition) |
| Aero coefficients | hand-entered | **derived from the section** |
| Verification | none | 4 scripts, 74 assertions |

---

## Repository layout

```
cad/     params.py + four generators, STEP output
sim/     model, airframe, worlds, and the verification scripts
media/   video masters (generated; see sim/capture_video.sh)
docs/    build notes and the environment gotchas
```

### Scripts

| Script | Does |
|---|---|
| `sim/install_into_px4.sh` | symlinks airframe + model into a PX4 checkout, idempotent |
| `sim/validate_model.sh` | SDFormat validity, structure, headless load |
| `sim/verify_hover.sh` | commands takeoff, measures altitude from Gazebo's pose topic |
| `sim/verify_transition.sh` | reads actual tilt joint angles through the manoeuvre |
| `sim/capture_video.sh` | headless flight recording, Gazebo → ROS 2 → MP4 |
| `sim/run_gui.sh` | interactive run with the Gazebo GUI |

---

## Pinned versions

Reproducibility is the point; these are the versions it was verified against.

| | |
|---|---|
| PX4-Autopilot | **v1.17.0** |
| Gazebo | **Harmonic, gz-sim 8.11.0** |
| ROS 2 | **Jazzy** (used for the video bridge) |
| Ubuntu | **24.04 noble** (WSL2) |
| build123d | **0.11.1** on Python 3.12 |
| Airframe ID | **4030** `gz_tri_tiltrotor` |

---

## Honest limitations

- **Simulation only.** No physical aircraft exists. Nothing here has been flown
  or wind-tunnel tested, and the nacelle tolerances are verified in CAD only.
- **Aerodynamic coefficients are derived from thin-airfoil theory and published
  section data**, not from CFD or measurement. They are traceable, not exact.
- **Inertia is estimated** from a rod-and-point-mass model, good to perhaps
  ±25%. The control-authority margins are wide enough that this does not change
  any conclusion, but it is an estimate.
- **The transition is verified as "the nacelles rotate through 90° in flight".**
  Sustained trimmed cruise and the back-transition are not yet characterised.
- **The Gazebo GUI does not render under WSLg** — the window maps at full size
  and composites solid black, every render engine and Qt backend, measured by
  pixel dump (`mean=0.00`). It renders correctly in an **xrdp session**
  (`sim/setup_xfce_xrdp.sh`), where 3D still runs on the GPU via Mesa's `d3d12`
  driver over `/dev/dxg`. Cause of the black desktop there in turn: WSLg
  exports `WAYLAND_DISPLAY` into every process, GTK prefers Wayland, and every
  XFCE component exits. `sim/fix_xrdp_display.sh` pins the session to X11.
- **Flown only in offboard mode from `sim/ros2/teleop_tiltrotor.py`.** There is
  no RC input and no QGroundControl on the development machine.
- **The pitch and geometry corrections are reasoned and generated, not yet
  re-flown.** `sim/verify_takeoff_pitch.sh` exists to confirm them and has not
  been run since the fixes landed.
- **No electrical design exists.** The BOM now carries the right servo count and
  a BEC sized for a 15 A peak servo rail, but there is no schematic, no power
  distribution board and no wiring harness. That is a KiCad job and it is the
  gap between "the aerodynamics and control allocation are verified" and "this
  could be built".
- **`MASS_FUSELAGE` is still 0.85 kg** after the body grew from 1.35 to 1.55 m.
  The mass budget closes only because that number is hand-entered.

## Licence

MIT.
