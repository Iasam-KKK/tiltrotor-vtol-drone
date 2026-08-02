# RUN — tri-tiltrotor simulation

Three commands, three terminals. Copy-paste, no thinking required.

---

## 0. Connect to the desktop first

⚠ **The Gazebo GUI must be launched from the xrdp session, not from a plain
`wsl.exe` window.** Under WSLg the window opens and composites solid black —
measured, every render-engine and Qt-backend combination, `mean=0.00`. A
WSLg-launched shell has `DISPLAY=:0` and puts you straight back there.

1. Windows **Remote Desktop** (`mstsc`) → `localhost:3390`
2. Session type **Xorg**, your WSL username + password
3. Open a terminal **inside that desktop**

Sanity check before anything else — want `D3D12`, not `llvmpipe`:

```bash
glxinfo -B | grep -i 'OpenGL renderer'
```

---

## The three commands

### 1. PX4 + Gazebo  — run this one INSIDE the RDP desktop

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/run_gui.sh
```

Wait for `INFO [commander] Ready for takeoff!` and the `pxh>` prompt.
Takes ~40 s (build check + Gazebo spawn).

### 2. uXRCE-DDS agent — any terminal, no X needed

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/ros2/run_agent.sh
```

Without this there are **no** `/fmu/*` topics and nothing errors — `ros2 topic
list` just comes back short. Leave it running.

### 3. Keyboard teleop — any terminal

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/ros2/teleop.sh
```

Wait for `position acquired: x=… y=…`. If it says
`waiting for PX4 position data` for more than 8 s it will tell you what to check.

```
space   arm            -> wait for  [PX4] now ARMED
o       offboard       -> wait for  [PX4] nav_state -> 14 (OFFBOARD)
w / s   climb / descend
a / d   YAW left/right     <- the vectored-yaw demo: the nacelles split apart
i / k   forward / back
j / l   slide left / right
t / b   transition to forward flight / back to hover
g       GLIDE — unpowered descent at best L/D
p       leave glide, back to powered flight
q       quit (disarms first)
```

### Glide test

```bash
# in teleop:  space -> o -> w (climb) -> t (forward flight) -> g (glide)
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/verify_glide.sh
```

Measures horizontal travel ÷ altitude lost from Gazebo's pose topic and
compares it against the L/D the drag polar predicts (**15.65 at 15.01 m/s**,
sink **0.957 m/s**). That number is derived in `params.py` from the NACA
section and the planform, so this is a test of the aerodynamic model, not a
demonstration of a descent.

---

## Worth having open too

Camera follows the aircraft instead of watching it fly away:

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/follow_drone.sh
# OFFSET="0 -6 2"  ... side-on, best for watching the nacelles tilt
# OFFSET="0 0 12"  ... top-down, best for seeing yaw
# --stop           ... release the camera
```

Live nacelle angles — this is how you *see* yaw happen:

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/watch_tilt.sh
```

---

## When something is wrong

One command, tells you what is actually true rather than what should be:

```bash
bash /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/sim/status.sh
```

It reports process state, whether the GUI window is really mapped, and — the
one that matters — the **publisher count** on `/fmu/out/vehicle_status`. A topic
appearing in `ros2 topic list` proves nothing: the teleop node's own
subscriptions create those names with no agent connected at all.

| Symptom | Cause |
|---|---|
| GUI black | launched from WSLg, not the RDP session |
| No `/fmu` data | agent not running, or started before PX4 (restart it *after*) |
| Teleop stuck "waiting for position" | same as above |
| Nothing moves | not armed — `space` first, watch for `[PX4] now ARMED` |
| GUI died, sim still running | `bash sim/run_gui_client.sh -b` reattaches without killing the flight |

---

## Rebuilding the model after a CAD change

`params.py` is the only place any dimension lives; everything else is generated.

```bash
cd /mnt/e/ME/UAV/projects/04-tiltrotor-vtol/cad
/mnt/e/ME/UAV/.venv-cad/Scripts/python.exe gen_geometry.py   # meshes
/mnt/e/ME/UAV/.venv-cad/Scripts/python.exe gen_sdf.py        # Gazebo model
/mnt/e/ME/UAV/.venv-cad/Scripts/python.exe gen_airframe.py   # PX4 airframe
/mnt/e/ME/UAV/.venv-cad/Scripts/python.exe gen_manifest.py   # Blender manifest
```

The model is symlinked into PX4, so regenerating takes effect on the next run
with no reinstall. If the airframe file changed, PX4 needs a rebuild — that is
what `run_gui.sh` does for you.

---

## Looking at the CAD, and exporting STL

Both run from Windows PowerShell, not WSL — FreeCAD is a Windows install here.

```powershell
cd E:\ME\UAV\projects\04-tiltrotor-vtol
powershell -File tools\open_in_freecad.ps1   # GUI, coloured by role, annotated
powershell -File tools\export_stl.ps1        # headless -> cad\out\stl\
```

Both regenerate the STEP assembly from `params.py` first, so what you see and
what you export are what the parameters currently say.

STL output lands in `cad/out/stl/`:

| folder | frame | for |
|---|---|---|
| `assembly/` | flight positions | load all 19 together, get an assembled aircraft |
| `print/` | each part on the origin, z=0 | dropping straight into a slicer |
| `tri_tiltrotor_full.stl` | flight positions, one mesh | previews, marketplace, anything single-file |

⚠ **Units.** These STLs are in **millimetres**. The meshes in
`sim/models/tri_tiltrotor/meshes/` are in **metres** because Gazebo wants them
that way — a slicer reads that wing as 2.1 mm long. Don't cross the streams.

⚠ **13 of the 19 parts are over a 256 mm bed** (wing 2126 mm, structure
1841 mm, fuselage 1550 mm). They export correctly and are watertight, but they
are aircraft-scale assemblies, not print-ready parts — they need splitting with
joints before anyone slices them. The six that fit as-is are the nacelle yokes
and cradles, the tail motor mount, and `prop_tail`.
