# Social copy — tri-tiltrotor VTOL

Written to the 8-slide carousel structure in the launch plan §7. Every number
below is measured, not estimated; if a claim cannot be traced to a script in
`sim/`, it is not here.

---

## LinkedIn carousel (8 slides, native PDF)

**1 — The finished thing**
> A VTOL that takes off like a helicopter and flies like a plane.
> Three rotors, not four. Built and verified in PX4 + Gazebo.
> Files are free.

**2 — The constraint**
> PX4 ships a **quad** tiltrotor: four motors, two of them tilting.
> That is four ESCs, four props and four mounts to buy, print and fail.
> I wanted the same job done with three.

**3 — The problem nobody mentions**
> My first sketch put both tilt rotors at the back.
> It cannot hover. Total lift acts behind the centre of mass and there is
> nothing to oppose the nose-down moment.
> The rotors have to **straddle** the CG. That is the whole design.

**4 — CAD, with the maths visible**
> Every dimension lives in one file: `params.py`.
> It runs **50 invariants in arithmetic before any CAD kernel starts** and
> refuses to build an aircraft that cannot fly.
> It caught a mass budget short by 0.88 kg, and a horizontal tail sitting
> inside the tail rotor's own wash.

**5 — The simulation**
> PX4 v1.17.0, Gazebo Harmonic, custom airframe 4030.
> Hover: commanded 10 m, **climbed to 10.07 m**.
> Read from Gazebo's pose topic, not from a log line.

**6 — IT TRANSITIONS**
> The wing nacelles rotate **0° → 90°** in flight, read straight off the
> Gazebo joint angles rather than from a log line.
> The third rotor never tilts: it is a fixed lift rotor on a pylon, stopped
> in cruise with a folding prop, sitting at a fuselage waist that narrows
> **57%** so it breathes clean air.
> That is the layout production VTOLs use, and it is why.

**7 — What went wrong**
> The aerofoil was on **backwards** — blunt trailing edge into the airflow.
> It lofted fine, flew fine in sim (the aero model reads coefficients, not
> geometry) and looked almost right in a render.
> Found it by looking at a frame. One sign flip.

**8 — Free**
> Full repo: CAD, URDF, Gazebo world, PX4 airframe, 4 verification scripts.
> One command to run it.
> `github.com/<user>/tri-tiltrotor-vtol`

---

## Shorts / Reels (vertical, 30–45 s)

Cut from `media/master_vertical.mp4` — the nacelle camera.

- **0:00–0:03** hold on the aircraft, rotors vertical, sitting still
- **0:03–0:08** takeoff, straight up. Caption: *"Helicopter mode."*
- **0:08–0:16** the nacelle rotates. Caption: *"Watch the motor."*
- **0:16–0:25** level cruise, rotors forward. Caption: *"Plane mode."*
- **0:25–0:30** end card: *"All 3 rotors tilt. PX4 only ships 2 of 4. Free."*

The single most valuable second of footage is the nacelle mid-rotation. Do not
cut away from it.

---

## X / Twitter

> PX4's stock tiltrotor tilts 2 of its 4 rotors.
>
> I built one where all 3 tilt — including the tail rotor, which becomes a
> pusher in cruise.
>
> Hover: 10.07 m against 10 m commanded.
> Transition: all three nacelles 0° → 89.6°.
>
> Repo, airframe and verification scripts are free. 🧵

---

## MakerWorld / printable listing

**Title:** Tilt nacelle module for tri-rotor VTOL — 3× identical, PX4 airframe included

**Body:**
> The tilting motor nacelle from a tri-tiltrotor VTOL. Three identical modules
> make the whole aircraft: two on the wing, one at the tail.
>
> Cradle holds a 28xx motor on a 19 mm M3 pattern. Yoke clamps a 16 mm carbon
> boom and carries 686ZZ bearings on a 6 mm shaft. Servo sizing was computed
> against the actual tilt load — 20 kg·cm gives 7.9× margin.
>
> **Print the fit coupon first.** It is a few grams and verifies the bolt
> pattern and bearing seat before you commit filament to the real part.
>
> Clearance fits throughout, no press fits: +0.30 mm on shaft bores, +0.15 mm
> on bearing seats.
>
> ⚠️ **Tolerances verified in CAD only.** I do not own this aircraft — it is
> verified in simulation (PX4 v1.17.0 + Gazebo Harmonic), and the full ROS 2 /
> PX4 package is linked. The sim is the evidence, not a photo.

---

## Honesty rules for all of the above

- Never write "tested" or "flown" — it is **simulated**, and saying so is the
  differentiator, not the weakness.
- Every number quoted must come from a script in `sim/` that a reader can run.
- Lead with the failure in slide 7. The aerofoil-backwards story is more
  credible than any success slide, and it is true.
