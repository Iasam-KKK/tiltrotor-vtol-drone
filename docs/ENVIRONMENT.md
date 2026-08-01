# Environment notes and traps

Everything here was found the hard way while getting this project to run on
**Windows 11 + WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**. Each
entry cost real time; none of it is obvious from the upstream docs.

---

## PX4

### The source zip is missing 29 submodules

A GitHub source archive (`/archive/refs/tags/v1.17.0.zip`) contains **none** of
PX4's 29 submodules, and the build fails one at a time, each attempt getting
further than the last. SITL needs only ten of them:

```
src/modules/mavlink/mavlink              src/lib/heatshrink/heatshrink
src/lib/events/libevents                 src/lib/crypto/monocypher
src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
src/lib/cdrstream/cyclonedds             src/lib/crypto/libtomcrypt
src/lib/cdrstream/rosidl                 src/lib/crypto/libtommath
src/drivers/gps/devices
```

The other 19 are NuttX, other simulators, and board-specific drivers — over a
gigabyte that SITL never touches.

### Submodules must be pinned, or the build fails confusingly

Cloning them at `HEAD` produces a **compile error, not a checkout error**:

```
src/drivers/gps/gps.cpp:541: error: 'class GPSHelper' has no member named 'shouldInjectRTCM'
```

That is version skew: v1.17.0's source against a newer PX4-GPSDrivers. Eight of
our ten were at the wrong commit. The pinned SHAs are recoverable from the
GitHub contents API even though the zip dropped them:

```
GET /repos/PX4/PX4-Autopilot/contents/<submodule-path>?ref=v1.17.0   -> .sha
```

Then `git fetch --depth 1 origin <sha> && git checkout FETCH_HEAD`.

### The Makefile refuses to build without `.git`

```
Makefile:39: *** YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.
```

It only tests `$(wildcard .git)`, so `git init` satisfies it — but the build
later uses `git describe` for version strings, so tag it too:

```bash
git init -q && git add -A && git commit -qm "snapshot" && git tag v1.17.0
```

### Run SITL from the build directory

`rcS` and the baked ROMFS resolve relative to the working directory:

```bash
cd build/px4_sitl_default && ./bin/px4 etc/init.d-posix/rcS
```

Running `./build/px4_sitl_default/bin/px4 -s etc/init.d-posix/rcS` from the
source root gives `Error opening startup file`.

### `commander mode takeoff` is not a command

```
ERROR [commander] argument takeoff unsupported.
```

It is `commander takeoff`, and it arms as part of its own sequence. Arming
separately first gets you `Disarmed by auto preflight disarming`.

---

## Control allocation

### Tilt angle zero means UP

From `src/modules/control_allocator/module.yaml`, `CA_SV_TL{i}_MINA`:

> *"Defines the tilt angle when the servo is at the minimum. An angle of zero
> means upwards."*

So **0° = hover, +90° = towards the `CA_SV_TL{i}_TD` azimuth = cruise**, range
−90…+90. Vectored yaw therefore needs travel *past vertical in the negative
direction*, not past +90. Getting this backwards produces a model that looks
entirely plausible and cannot yaw.

### Servo order is control surfaces first, then tilts

PX4 allocates `CA_SV_CS0..n` before `CA_SV_TL0..n`. Stock `4020_gz_tiltrotor`
demonstrates it: 3 surfaces on `SIM_GZ_SV_FUNC1-3`, then the tilt-angle
parameters land on servos 4 and 5. Put tilts first in the SDF and tilt commands
silently drive your ailerons.

`gz` topic `servo_N` corresponds to PX4 `SIM_GZ_SV_*{N+1}` — gz is 0-indexed,
PX4 is 1-indexed.

### Up to four tilt servos are supported natively

`__max_num_tilts: 4`. A tri-tiltrotor needs no custom allocator.

---

## Gazebo

### The airspeed sensor name is hard-coded

`GZBridge.cpp:281` subscribes to exactly:

```
/world/<world>/model/<model>/link/airspeed_link/sensor/air_speed/air_speed
```

The link **must** be `airspeed_link` and the sensor **must** be `air_speed`.
An identical sensor on `base_link` under any other name publishes correctly and
is never read. The symptom is not an error:

```
INFO  [airspeed_selector] No airspeed sensor detected. Switch to non-airspeed mode.
WARN  [health_and_arming_checks] Preflight Fail: Airspeed invalid
```

which reads like a sensor fault rather than a naming one.

### Rotor spin axes must NOT be `expressed_in="__model__"`

The tilt joint axes may be. The **rotor** axes must be in the joint frame so
they rotate with the nacelle. Pin them to the model frame and thrust points up
forever regardless of tilt — the aircraft hovers perfectly and never
transitions, which presents as a controller bug.

### `use_parent_model_frame` is legacy SDF 1.4

Stock PX4 models still use it. The modern spelling is the attribute
`<xyz expressed_in="__model__">`.

### XML comments cannot contain `--`

A generated banner reading `GENERATED FILE -- DO NOT EDIT` makes the whole
document unparseable.

### `<save enabled="true">` on a camera does not work here

It writes no files and the run dumps core. Take frames off the image **topic**
instead — see `sim/record_video.py`.

### `joint_state` carries a pose AND an angle

Each joint block has `pose { position { x y z } }` (a **translation**) and,
for revolute joints, `axis1 { position: }` (the **angle**, radians). Scraping
`position:` naively yields nonsense — an early version of
`verify_transition.sh` reported −103 231° of nacelle tilt and still printed
PASS.

---

## WSLg

### `GALLIUM_DRIVER=d3d12` must be set in every script

Ubuntu's `~/.bashrc` returns early for non-interactive shells, and
non-interactive bash does not source it at all. **Any** script run via
`wsl -- bash foo.sh` therefore sees no `GALLIUM_DRIVER` and silently falls back
to llvmpipe software rendering, on a machine whose interactive shells are fully
GPU accelerated.

### ogre2 cannot create a GL context

```
glx: failed to create drisw screen
[GUI] [Err] [Application.cc:912] [QT] Failed to create OpenGL context
```

`--render-engine ogre` (v1) works. PX4 passes this through with
`PX4_GZ_SIM_RENDER_ENGINE=ogre`, which reaches both the server and the GUI.
This is **not** a GPU capability problem — `glxinfo` reports OpenGL 4.6 core on
D3D12 (RTX 5070), accelerated.

### Mesh visuals DO work — the crash was elsewhere

An earlier reading of this said mesh rendering was broken. It is not. Mesh
visuals render fine in the capture pipeline. The segfault was specific to
loading `test.sdf` with the Sensors system present, and the stock PX4 x500
crashed the same way, so it was never our geometry.

The validation world now omits `gz-sim-sensors-system` entirely — it exists to
check structure and physics, neither of which needs a renderer — and the
capture world keeps it. Separate concerns, separate worlds.

### The Gazebo GUI window does not render

The process is healthy, the world connects, no errors are logged, and the X
window is created — at **1×1 pixels**, or later composited as a solid black
rectangle titled `[WARN:COPY MODE]`. Clearing `~/.gz/**/gui.config` does not
help. Neither does `QT_QPA_PLATFORM=xcb`.

**Do not fight this.** Render headless and record from the image topics
(`sim/capture_video.sh`). It is more reliable, it produces the video the
deliverables need anyway, and it removes the compositor from the critical path.

### Shell quoting across PowerShell → WSL → bash

Both wrappers interpolate before bash sees the string:

- **PowerShell** expands `$VAR` inside double quotes and treats `|` as a pipe.
- **Git Bash** expands `$PATH` — whose Windows value contains
  `Program Files (x86)`, and the parentheses break the inner bash parse.
- Git Bash also rewrites `/mnt/...` via MSYS path conversion, producing
  `C:/Program Files/Git/mnt/...`.

**Write a script file and run `wsl -- bash /path/script.sh`.** Inline commands
with variables, pipes or absolute paths will bite you.

### `/tmp` is tmpfs

A WSL restart deletes it. Logs you intend to read afterwards belong somewhere
persistent — this project uses `~/tritilt_logs`.

---

## Build tooling

Ubuntu 24.04 minimal ships **no pip and no ensurepip**. Bootstrap without sudo:

```bash
wget -c https://bootstrap.pypa.io/get-pip.py
python3 get-pip.py --user --break-system-packages
python3 -m pip install --user --break-system-packages kconfiglib jsonschema ninja
```

`~/.local/bin` is not on `PATH` for non-interactive shells; export it.

**Gazebo does not need a separate install.** ROS 2 Jazzy's vendor packages
provide the whole of Harmonic, including the `gz` CLI at
`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`, and PX4 links against them
happily. Installing standalone `gz-harmonic` alongside would risk the verified
ROS stack for no gain.
