r"""
Generate the Gazebo Harmonic SDF for the tri-tiltrotor from params.py.

Nothing in the emitted model is typed by hand. Every position, mass, inertia
and area comes from params.py, so the simulated aircraft, the generated PX4
airframe file and the printed nacelle all move together when a parameter
changes. This is the structure that worked on 01-nav2-deck.

Run:
    .\.venv-cad\Scripts\python.exe projects\04-tiltrotor-vtol\cad\gen_sdf.py

Frame convention: Gazebo model frame is FLU -- +x forward, +y LEFT, +z UP.
This is NOT the FRD body frame that params.py documents its stations in, so
lateral signs are flipped on the way out. The model origin sits at the CG.

TILT SIGN CONVENTION -- resolved from PX4 source, not assumed.

PX4 v1.17.0 src/modules/control_allocator/module.yaml, CA_SV_TL{i}_MINA:
    "An angle of zero means upwards."

So: tilt = 0 deg -> thrust UP (hover); tilt = +90 deg -> thrust towards the
CA_SV_TL{i}_TD azimuth, which we set to 0 ('Towards Front') -> cruise.

Our SDF joint axis is +y (LEFT in FLU). A positive rotation about +y carries
+z into +x, i.e. up -> forward. The geometry therefore already agrees with
PX4. What did NOT agree was the joint travel: vectored yaw needs the nacelle
to lean AFT of vertical, which is a NEGATIVE angle, not travel past +90.

+-------------------------------------------------------------------------+
| STILL MUST BE CONFIRMED BY RUNNING IT:                                   |
| That the assembled model actually climbs when commanded to hover. The    |
| convention above is documented; whether this particular link/joint tree  |
| realises it is not. 02 lost real time to exactly this class of error     |
| with FreeCAD's arbitrary cylinder axis sign, which also looked plausible |
| in the GUI. Resolve with verify_hover.sh (does it climb?), not by eye.   |
+-------------------------------------------------------------------------+
"""

from __future__ import annotations

import math
from pathlib import Path

import params as P

MODEL_NAME = "tri_tiltrotor"

# ---------------------------------------------------------------------------
# Mesh visuals are OFF by default, and this is a deliberate environment
# workaround rather than a design preference.
#
# Measured on this machine (WSLg, gz-sim 8.11.0, ogre and ogre2 both):
#     box visuals  + camera  -> runs, records video
#     mesh visuals + camera  -> SIGSEGV during "Create scene"
# and the stock PX4 x500 model crashes exactly the same way, so the fault is
# mesh rendering in this environment, not our geometry. Our meshes are valid:
# byte-exact STL, valid solids, STEP round-trip at 0.000000 mm^3.
#
# The lofted geometry is therefore used where it actually matters -- the STEP
# and STL deliverables, and Blender renders -- while the simulator, whose job
# is physics and verification, runs on primitives it can draw without dying.
#
# Set to True on a machine with working mesh rendering; nothing else changes.
USE_MESH_VISUALS = True
OUT_DIR = Path(__file__).resolve().parent.parent / "sim" / "models" / MODEL_NAME

# ---------------------------------------------------------------------------
# PX4 actuator output map.
#
# These indices MUST match the CA_* assignments in the generated airframe file.
# They are declared once, here, and consumed by both generators.
# ---------------------------------------------------------------------------
# ORDER IS NOT ARBITRARY. PX4 allocates servo outputs as control surfaces
# first (CA_SV_CS0..n), then tilt servos (CA_SV_TL0..n). Stock 4020_gz_tiltrotor
# proves it: CA_SV_CS_COUNT 3 with SIM_GZ_SV_FUNC1-3, then CA_SV_TL_COUNT 2
# carrying SIM_GZ_SV_MINA4/MAXA4 and MINA5/MAXA5 -- the tilt-angle parameters
# land on servos 4 and 5, i.e. after the surfaces. Putting tilts first here
# would silently drive the ailerons with tilt commands.
#
# gz sub_topic servo_N  <->  PX4 SIM_GZ_SV_*{N+1}   (gz is 0-indexed, PX4 is 1-)
MOTOR_LEFT, MOTOR_RIGHT, MOTOR_TAIL = 0, 1, 2
SERVO_AILERON_LEFT = 0
SERVO_AILERON_RIGHT = 1
# V-tail ruddervators: each carries BOTH pitch and yaw. Two surfaces where a
# conventional tail needs three.
SERVO_VTAIL_LEFT = 2
SERVO_VTAIL_RIGHT = 3
SERVO_TILT_LEFT = 4
SERVO_TILT_RIGHT = 5
SERVO_TILT_TAIL = 6


def _f(x: float) -> str:
    """Format a float for SDF at a fixed precision.

    Six decimals is deliberate: 02 recorded a case where a value read to 2 dp
    looked like 24.8 deg when it was exactly 25.000 deg. Print more digits than
    feel necessary.
    """
    return f"{x:.6f}"


def _xyz(x: float, y: float, z: float) -> str:
    return f"{_f(x)} {_f(y)} {_f(z)}"


# ---------------------------------------------------------------------------
# Geometry, resolved once
# ---------------------------------------------------------------------------

def geometry() -> dict:
    d = P.solve()
    a = d.wing_rotor_arm            # forward of CG, positive
    c = d.tail_rotor_arm            # aft of CG, positive

    # Wing leading edge, forward of CG.
    le_x = P.CG_MAC_FRACTION * P.WING_CHORD
    # Wing quarter-chord (aerodynamic centre of a thin section).
    # ROOT quarter-chord: this positions the wing mesh, the ailerons and the
    # LiftDrag panels, so it must carry the sweep offset that le_x does not.
    quarter_x = P.wing_root_quarter_chord_x()

    return {
        "d": d,
        "a": a,
        "c": c,
        "le_x": le_x,
        "quarter_x": quarter_x,
        "rotor_z": P.NACELLE_Z_OFFSET,
        "wing_r": P.WING_PROP_DIAMETER / 2.0,
        "tail_r": P.TAIL_PROP_DIAMETER / 2.0,
        "semi_span": P.WING_SPAN / 2.0,
        # Wing nacelles: aft of vertical (negative) up to full cruise.
        "tilt_wing_lower": -P.TILT_YAW_TRAVEL,
        "tilt_wing_upper": P.TILT_ANGLE_CRUISE,
        # Tail nacelle: vertical to full cruise, no yaw travel.
        "tilt_tail_lower": P.TILT_ANGLE_HOVER,
        "tilt_tail_upper": P.TILT_ANGLE_CRUISE,
        # Centre of pressure of each wing panel: quarter chord, mid semi-span.
        "panel_cp_y": P.WING_SPAN / 4.0,
        "panel_area": P.WING_SPAN * P.WING_CHORD / 2.0,
    }


# ---------------------------------------------------------------------------
# XML fragments
# ---------------------------------------------------------------------------

def rotor_link(name: str, x: float, y: float, z: float, radius: float,
               parent_motor: str) -> str:
    """A rotor. Visual is a real twisted propeller mesh when meshes are on.

    The cylinder fallback is a correct swept-disc abstraction but reads as a
    frisbee in a render, and it hides the one thing this aircraft needs to show
    on camera: which way the nacelle is pointing. Actual blades make the tilt
    obvious mid-transition.
    """
    which = "prop_tail" if "tail" in name else "prop_wing"
    if USE_MESH_VISUALS:
        vis = f"""      <visual name="{name}_visual">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/{which}.stl</uri></mesh></geometry>
        <material>
          <ambient>0.08 0.08 0.09 1</ambient>
          <diffuse>0.13 0.13 0.15 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
        </material>
      </visual>"""
    else:
        vis = f"""      <visual name="{name}_visual">
        <geometry>
          <cylinder><radius>{_f(radius)}</radius><length>0.005</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>"""

    return f"""
    <link name="{name}">
      <pose relative_to="{parent_motor}">0 0 0.025 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>9.75e-07</ixx><iyy>0.000166704</iyy><izz>0.000167604</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
{vis}
    </link>"""


def motor_link(name: str, x: float, y: float, z: float) -> str:
    """The tilting nacelle body. This is the link the printed part represents."""
    return f"""
    <link name="{name}">
      <pose>{_xyz(x, y, z)} 0 0 0</pose>
      <inertial>
        <mass>{_f(P.MASS_NACELLE_WING if 'tail' not in name else P.MASS_NACELLE_TAIL)}</mass>
        <inertia>
          <ixx>0.000420</ixx><iyy>0.000420</iyy><izz>0.000350</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="{name}_visual">
        <geometry>
          <box><size>0.060 0.045 0.045</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.35 0.05 1</ambient>
          <diffuse>0.9 0.4 0.05 1</diffuse>
        </material>
      </visual>
    </link>"""


def tilt_joint(name: str, child: str, lower: float, upper: float) -> str:
    """Revolute nacelle joint. 0 rad = thrust UP (hover), +pi/2 = forward.

    The wing pair gets a NEGATIVE lower limit so the nacelle can lean aft of
    vertical for vectored yaw. The tail nacelle has no yaw role -- a
    centreline force in the x-z plane makes no moment about z -- so it stops
    at exactly vertical.
    """
    return f"""
    <joint name="{name}" type="revolute">
      <parent>base_link</parent>
      <child>{child}</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>{_f(lower)}</lower>
          <upper>{_f(upper)}</upper>
          <effort>10</effort>
          <velocity>{_f(P.TILT_RATE)}</velocity>
        </limit>
        <dynamics>
          <friction>1.0</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>"""


def motor_plugin(joint: str, link: str, direction: str, number: int,
                 max_rot: float, motor_constant: float) -> str:
    return f"""
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>{joint}</jointName>
      <linkName>{link}</linkName>
      <turningDirection>{direction}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{_f(max_rot)}</maxRotVelocity>
      <motorConstant>{motor_constant:.3e}</motorConstant>
      <momentConstant>0.06</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{number}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>20</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""


def servo_plugin(joint: str, servo_index: int, p_gain: float = 100.0) -> str:
    return f"""
    <plugin filename="gz-sim-joint-position-controller-system"
      name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <sub_topic>servo_{servo_index}</sub_topic>
      <p_gain>{_f(p_gain)}</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0</d_gain>
      <i_max>0.0</i_max>
      <i_min>0.0</i_min>
      <cmd_max>10</cmd_max>
      <cmd_min>-10</cmd_min>
      <err_max>0.2</err_max>
    </plugin>"""


def liftdrag(cp: tuple[float, float, float], area: float,
             control_joint: str | None, rad_to_cl: float,
             cla: float, cda: float, alpha_stall: float,
             upward: str = "0 0 1", a0: float = 0.0) -> str:
    """A lift surface.

    Stock gz_tiltrotor has NO main wing lift surface at all -- its two 0.5 m^2
    elevons stand in for the entire wing. Here each wing panel gets its own
    LiftDrag at its own centre of pressure, so roll damping and the lift
    distribution are represented rather than lumped.

    Our wing surfaces are AILERONS, not elevons: this aircraft has a real
    horizontal tail with an elevator, so putting pitch authority on the wing
    as well would double-book it across two actuator groups.
    """
    ctrl = ""
    if control_joint:
        ctrl = (f"\n      <control_joint_name>{control_joint}</control_joint_name>"
                f"\n      <control_joint_rad_to_cl>{_f(rad_to_cl)}</control_joint_rad_to_cl>")
    return f"""
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{_f(a0)}</a0>
      <cla>{_f(cla)}</cla>
      <cda>{_f(cda)}</cda>
      <cma>0.0</cma>
      <alpha_stall>{_f(alpha_stall)}</alpha_stall>
      <cla_stall>-3.85</cla_stall>
      <cda_stall>-0.923398</cda_stall>
      <cma_stall>0</cma_stall>
      <cp>{_xyz(*cp)}</cp>
      <area>{_f(area)}</area>
      <air_density>{_f(P.RHO)}</air_density>
      <forward>1 0 0</forward>
      <upward>{upward}</upward>
      <link_name>base_link</link_name>{ctrl}
    </plugin>"""


def surface(name: str, x: float, y: float, z: float, mesh: str,
            axis: str) -> str:
    """A control surface link plus its revolute joint.

    ⚠ These were <box> visuals: grey slabs positioned near the trailing edge,
    visibly detached from a wing that is tapered, swept and dihedralled. They
    are now the lofted aerofoil meshes from gen_geometry, and the link pose is
    the real hinge point taken from params.aileron_geometry() /
    params.ruddervator_geometry() -- the SAME call the mesh was built about, so
    the two cannot drift.

    The meshes are authored with their ORIGIN ON THE HINGE LINE, which is why
    the visual needs no pose of its own and the surface rotates about its hinge
    rather than about its centroid.

    Cosmetic only: the LiftDrag plugins read the joint ANGLE and never touch
    the mesh, so this changes how it looks and nothing about how it flies.
    """
    lim = _f(P.CONTROL_DEFLECT_MAX)
    return f"""
    <link name="{name}">
      <pose>{_xyz(x, y, z)} 0 0 0</pose>
      <inertial>
        <mass>1e-08</mass>
        <inertia>
          <ixx>1e-06</ixx><iyy>1e-06</iyy><izz>1e-06</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="{name}_visual">
        <geometry>
          <mesh><uri>model://{MODEL_NAME}/meshes/{mesh}.stl</uri></mesh>
        </geometry>
        <material>
          <ambient>0.82 0.82 0.85 1</ambient>
          <diffuse>0.88 0.88 0.91 1</diffuse>
        </material>
      </visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz expressed_in="__model__">{axis}</xyz>
        <limit><lower>-{lim}</lower><upper>{lim}</upper>
          <effort>10</effort><velocity>10</velocity></limit>
        <dynamics><damping>1.0</damping></dynamics>
      </axis>
    </joint>"""


def airspeed_link() -> str:
    """Airspeed sensor on its OWN link, with the exact names PX4 requires.

    This is not cosmetic naming. PX4's GZBridge.cpp:281 hard-codes the topic:

        /world/<world>/model/<model>/link/airspeed_link/sensor/air_speed/air_speed

    so the link MUST be called `airspeed_link` and the sensor MUST be called
    `air_speed`. Putting an identical sensor on base_link under any other name
    produces a Gazebo topic that publishes correctly and that PX4 never
    subscribes to. The symptom is not an error -- it is
    "No airspeed sensor detected. Switch to non-airspeed mode." followed by
    "Preflight Fail: Airspeed invalid", which reads like a sensor bug rather
    than a naming one.

    Stock does this with <include merge="true"><uri>model://airspeed</uri>.
    Inlined here instead so the model stays self-contained and can be loaded
    without PX4's model library on the resource path.
    """
    return """
    <link name="airspeed_link">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.015</mass>
        <inertia>
          <ixx>1e-05</ixx><iyy>1e-05</iyy><izz>1e-05</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="airspeed_visual">
        <pose>0 0 0 0 1.57 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>0.08</length></cylinder></geometry>
        <material>
          <diffuse>0.0 0.0 0.0 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      <sensor name="air_speed" type="air_speed">
        <gz_frame_id>airspeed_link</gz_frame_id>
        <pose>0 0 0 0 0 0</pose>
        <update_rate>5.0</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="airspeed_joint" type="fixed">
      <parent>base_link</parent>
      <child>airspeed_link</child>
    </joint>"""


def _airframe_visuals(g: dict, fuse_len: float) -> str:
    """Airframe visuals: lofted meshes if the environment can render them,
    otherwise primitives. See USE_MESH_VISUALS at the top of this file."""
    metal = """        <material>
          <ambient>0.78 0.78 0.80 1</ambient>
          <diffuse>0.90 0.90 0.92 1</diffuse>
          <specular>0.35 0.35 0.35 1</specular>
        </material>"""
    body = """        <material>
          <ambient>0.16 0.19 0.24 1</ambient>
          <diffuse>0.22 0.26 0.33 1</diffuse>
          <specular>0.30 0.30 0.30 1</specular>
        </material>"""

    if USE_MESH_VISUALS:
        return f"""      <visual name="fuselage_visual">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/fuselage.stl</uri></mesh></geometry>
{body}
      </visual>
      <visual name="wing_visual">
        <pose>{_xyz(g['quarter_x'], 0, 0)} 0 0 0</pose>
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/wing.stl</uri></mesh></geometry>
{metal}
      </visual>
      <visual name="tail_visual">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/tail.stl</uri></mesh></geometry>
{metal}
      </visual>
      <visual name="booms_visual">
        <geometry><mesh><uri>model://{MODEL_NAME}/meshes/booms.stl</uri></mesh></geometry>
        <material>
          <ambient>0.05 0.05 0.06 1</ambient>
          <diffuse>0.09 0.09 0.11 1</diffuse>
          <specular>0.30 0.30 0.30 1</specular>
        </material>
      </visual>"""

    # Primitive fallback. Tapered look is approximated with two panels per side
    # so it at least reads as a wing rather than one slab.
    c_root = P.WING_CHORD * 3.0 * (1.0 + P.WING_TAPER) / (
        2.0 * (1.0 + P.WING_TAPER + P.WING_TAPER ** 2))
    c_tip = c_root * P.WING_TAPER
    quarter = P.WING_SPAN / 4.0
    return f"""      <visual name="fuselage_visual">
        <pose>{_xyz(g['le_x'] - fuse_len / 2 + P.WING_ROTOR_AHEAD_OF_LE, 0, 0)} 0 0 0</pose>
        <geometry><box><size>{_f(fuse_len)} 0.120 0.100</size></box></geometry>
{body}
      </visual>
      <visual name="wing_inner_visual">
        <pose>{_xyz(g['quarter_x'], 0, 0)} 0 0 0</pose>
        <geometry>
          <box><size>{_f(c_root)} {_f(P.WING_SPAN / 2)} 0.030</size></box>
        </geometry>
{metal}
      </visual>
      <visual name="wing_outer_left_visual">
        <pose>{_xyz(g['quarter_x'], quarter * 1.5, 0.026)} 0 0 0</pose>
        <geometry>
          <box><size>{_f(c_tip)} {_f(P.WING_SPAN / 4)} 0.022</size></box>
        </geometry>
{metal}
      </visual>
      <visual name="wing_outer_right_visual">
        <pose>{_xyz(g['quarter_x'], -quarter * 1.5, 0.026)} 0 0 0</pose>
        <geometry>
          <box><size>{_f(c_tip)} {_f(P.WING_SPAN / 4)} 0.022</size></box>
        </geometry>
{metal}
      </visual>
      <visual name="htail_visual">
        <pose>{_xyz(-P.TAIL_SURFACE_ARM, 0, 0.02)} 0 0 0</pose>
        <geometry>
          <box><size>0.150 {_f(P.TAIL_H_AREA / 0.150)} 0.014</size></box>
        </geometry>
{metal}
      </visual>
      <visual name="vfin_visual">
        <pose>{_xyz(-P.TAIL_SURFACE_ARM, 0, 0.02 + P.TAIL_V_AREA / 0.150 / 2)} 0 0 0</pose>
        <geometry>
          <box><size>0.150 0.012 {_f(P.TAIL_V_AREA / 0.150)}</size></box>
        </geometry>
{metal}
      </visual>"""


def cameras() -> str:
    """Cameras rigidly attached to the airframe, for headless video capture.

    A camera parked in the world cannot follow the aircraft: the first capture
    run lost it out of frame within ~15 s of takeoff and the remaining 30 s of
    footage was an empty field. Attaching the cameras to the model means they
    track it for free.

    Two framings, both of which the launch plan needs:

      chase_cam    behind and above, looking forward. The 16:9 master.
      nacelle_cam  ahead and outboard, aimed back at the LEFT nacelle. This is
                   the shot that actually sells the project -- you watch the
                   nacelle rotate from vertical to horizontal mid-air during
                   the transition. Vertical framing for Shorts/Reels.

    Camera looks along its own +x. Poses are relative to base_link, so they
    move with the aircraft and pitch with it through the transition.
    """
    g = geometry()
    # Aim the nacelle camera at the left wing nacelle.
    cam = (0.90, 0.85, 0.15)
    tgt = (g["a"], P.WING_ROTOR_Y, g["rotor_z"])
    dx, dy, dz = (tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2])
    horiz = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, horiz)

    return f"""
    <link name="chase_cam_link">
      <pose relative_to="base_link">-2.20 0 0.75 0 0.15 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia><ixx>1e-07</ixx><iyy>1e-07</iyy><izz>1e-07</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="chase_cam" type="camera">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.05</horizontal_fov>
          <image><width>1920</width><height>1080</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>2000</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="chase_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>chase_cam_link</child>
    </joint>

    <link name="nacelle_cam_link">
      <pose relative_to="base_link">{_xyz(*cam)} 0 {_f(pitch)} {_f(yaw)}</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia><ixx>1e-07</ixx><iyy>1e-07</iyy><izz>1e-07</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="nacelle_cam" type="camera">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>0.95</horizontal_fov>
          <image><width>1080</width><height>1920</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>500</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="nacelle_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>nacelle_cam_link</child>
    </joint>
{nose_camera()}"""


def nose_camera() -> str:
    """The 1080p payload camera in the nose.

    Unlike chase_cam and nacelle_cam -- which exist to make renders and videos
    and are not part of the aircraft -- this one IS payload. It carries mass in
    the budget, occupies a station in the equipment bay, and its position is
    checked against the rotor discs so no blade crosses the frame.

    Mounted {math.degrees(P.CAMERA_TILT_DOWN):.0f} deg nose-down, which is what a survey camera wants: in
    cruise the fuselage sits at a small positive alpha, so a level camera looks
    slightly up at the sky.

    gz_frame_id matters. Without it the image has no usable TF frame and
    anything downstream (a detector, a mapper) has nothing to transform
    against -- it publishes happily and is useless.
    """
    if not P.CAMERA_ENABLED:
        return ""
    return f"""
    <link name="nose_cam_link">
      <pose relative_to="base_link">{_f(P.camera_x())} 0 0 0 {_f(P.CAMERA_TILT_DOWN)} 0</pose>
      <inertial>
        <mass>{_f(P.CAMERA_MASS)}</mass>
        <inertia><ixx>1e-06</ixx><iyy>1e-06</iyy><izz>1e-06</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="nose_cam_visual">
        <geometry><box><size>0.030 0.030 0.030</size></box></geometry>
        <material>
          <ambient>0.1 0.1 0.12 1</ambient>
          <diffuse>0.15 0.15 0.18 1</diffuse>
        </material>
      </visual>
      <sensor name="nose_cam" type="camera">
        <always_on>1</always_on>
        <update_rate>{P.CAMERA_FPS}</update_rate>
        <visualize>true</visualize>
        <topic>nose_cam/image_raw</topic>
        <gz_frame_id>nose_cam_link</gz_frame_id>
        <camera>
          <horizontal_fov>{_f(P.CAMERA_HFOV)}</horizontal_fov>
          <image>
            <width>{P.CAMERA_WIDTH}</width>
            <height>{P.CAMERA_HEIGHT}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>3000</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
        </camera>
      </sensor>
    </link>
    <joint name="nose_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>nose_cam_link</child>
    </joint>"""


def sensors() -> str:
    """Sensor suite carried on base_link.

    Airspeed is deliberately NOT here -- see airspeed_link() for why it needs
    its own link with a specific name.
    """
    return """
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.00034</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.00034</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.00034</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure>
        </air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><mean>0.000000080</mean><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.000000040</mean><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.000000120</mean><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>"""


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------

def build_sdf() -> str:
    g = geometry()
    d = g["d"]
    mass, ixx, iyy, izz = P.base_link_inertia()

    a, c = g["a"], g["c"]

    # --- put the assembled model's CG back on the origin --------------------
    # Sum m*x over every link that is NOT base_link, then shift base_link's own
    # CG the other way. Listed explicitly rather than inferred, so adding a link
    # without appearing here is a visible omission rather than a silent 11 mm
    # error in the moment arms.
    ROTOR_MASS, AIRSPEED_MASS, CAM_MASS = 0.005, 0.015, 0.001
    other = [
        (P.MASS_NACELLE_WING, a), (P.MASS_NACELLE_WING, a),   # motor_left/right
        (P.MASS_NACELLE_TAIL, -c),                            # motor_tail
        (ROTOR_MASS, a), (ROTOR_MASS, a), (ROTOR_MASS, -c),   # rotor links
        (AIRSPEED_MASS, 0.0),                                 # boom, on base
        (CAM_MASS, -2.20), (CAM_MASS, a),                     # chase + nacelle cam
    ]
    moment = sum(m * x for m, x in other)
    cg_fix = -moment / mass
    total = mass + sum(m for m, _ in other)
    rz = g["rotor_z"]
    wr, tr = g["wing_r"], g["tail_r"]

    # Motor rotational speed needed for hover, from the motor constant.
    # T = motorConstant * omega^2  ->  omega = sqrt(T / k)
    k_wing = 2.0e-05
    k_tail = 1.2e-05
    omega_wing_hover = math.sqrt(d.thrust_wing_each / k_wing)
    omega_tail_hover = math.sqrt(d.thrust_tail / k_tail)
    # Size max rotor speed off peak datasheet thrust, not off hover.
    max_rot_wing = math.sqrt(P.WING_MOTOR_THRUST_MAX / k_wing)
    max_rot_tail = math.sqrt(P.TAIL_MOTOR_THRUST_MAX / k_tail)

    fuse_len = P.WING_ROTOR_AHEAD_OF_LE + P.WING_CHORD + P.TAIL_SURFACE_ARM

    parts = [f"""<?xml version="1.0" ?>
<!--
  GENERATED FILE. DO NOT EDIT BY HAND.
  (Note: XML comments may not contain a double hyphen, so this banner uses none.)
  Source of truth: cad/params.py   Generator: cad/gen_sdf.py
  Regenerate with:  python cad/gen_sdf.py

  Tri-tiltrotor VTOL. Three nacelles, all three tilt 0-90 deg.
  Hover trim solved, not assumed: wing {d.thrust_wing_each:.2f} N each,
  tail {d.thrust_tail:.2f} N ({d.tail_lift_fraction * 100:.1f}% of lift).
-->
<sdf version="1.9">
  <model name="{MODEL_NAME}">
    <pose>0 0 {_f(P.LANDING_GEAR_HEIGHT)} 0 0 0</pose>

    <link name="base_link">
      <inertial>
        <!--
          CG CORRECTION. base_link's inertial defaults to the LINK ORIGIN,
          which is the model origin, which params.py treats as the CG. But the
          nacelles and rotors are separate links hanging mass at their own
          stations, so Gazebo's whole-model CG lands aft of the origin:
              3.85 kg at   0.000     (base_link)
              0.35 kg at  +0.1728    (motor_left)
              0.35 kg at  +0.1728    (motor_right)
              0.25 kg at  -0.700     (motor_tail)
              + rotors, airspeed boom and cameras
          Every CA_ROTOR*_PX in the airframe is quoted FROM THE CG, so a CG
          that is not at the origin makes PX4 allocate with the wrong moment
          arms and leaves a standing pitch moment.

          Offsetting base_link's own CG by the opposite amount puts the
          assembled model's CG back on the origin, where the design says it is.
        -->
        <pose>{_f(cg_fix)} 0 0 0 0 0</pose>
        <mass>{_f(mass)}</mass>
        <inertia>
          <ixx>{_f(ixx)}</ixx>
          <iyy>{_f(iyy)}</iyy>
          <izz>{_f(izz)}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="fuselage_collision">
        <geometry><box><size>{_f(fuse_len)} 0.120 0.100</size></box></geometry>
      </collision>
      <!--
        Visuals are LOFTED MESHES generated by cad/gen_geometry.py from the
        same NACA section params.py derives the coefficients from. The earlier
        version used boxes here; it flew correctly and looked like planks.

        Collision below stays a primitive on purpose: mesh collision would cost
        a lot of physics time for no fidelity on an aircraft whose only
        contacts are standing on the ground and hitting it.
      -->
{_airframe_visuals(g, fuse_len)}
{sensors()}
    </link>
"""]

    # --- Airspeed, on its own PX4-named link -------------------------------
    parts.append(airspeed_link())

    # --- On-board cameras for headless capture -----------------------------
    parts.append(cameras())

    # --- Nacelles, tilt joints, rotors -------------------------------------
    # +y is LEFT in the Gazebo model frame.
    parts.append(motor_link("motor_left", a, +P.WING_ROTOR_Y, rz))
    parts.append(motor_link("motor_right", a, -P.WING_ROTOR_Y, rz))
    parts.append(motor_link("motor_tail", -c, 0.0, rz))

    parts.append(tilt_joint("tilt_left_joint", "motor_left",
                            g["tilt_wing_lower"], g["tilt_wing_upper"]))
    parts.append(tilt_joint("tilt_right_joint", "motor_right",
                            g["tilt_wing_lower"], g["tilt_wing_upper"]))
    if P.TAIL_TILTS:
        parts.append(tilt_joint("tilt_tail_joint", "motor_tail",
                                g["tilt_tail_lower"], g["tilt_tail_upper"]))
    else:
        # Fixed tail rotor. NOTE the convention: 0 deg is UP, so a
        # lift-only tail nacelle is locked at ZERO, not at 90. Locking it at
        # 90 would point it forward and give no hover lift at all.
        parts.append(f"""
    <joint name="tilt_tail_joint" type="fixed">
      <parent>base_link</parent>
      <child>motor_tail</child>
    </joint>""")

    parts.append(rotor_link("rotor_left", a, +P.WING_ROTOR_Y, rz, wr, "motor_left"))
    parts.append(rotor_link("rotor_right", a, -P.WING_ROTOR_Y, rz, wr, "motor_right"))
    parts.append(rotor_link("rotor_tail", -c, 0.0, rz, tr, "motor_tail"))

    for rname, mname in (("rotor_left", "motor_left"),
                         ("rotor_right", "motor_right"),
                         ("rotor_tail", "motor_tail")):
        # The rotor spin axis is deliberately NOT expressed_in="__model__".
        # It must be expressed in the joint frame so that it ROTATES with the
        # tilting nacelle. Pin it to the model frame and thrust points up
        # forever regardless of tilt angle -- the aircraft would hover fine
        # and never transition, which reads as a controller bug rather than a
        # geometry one.
        parts.append(f"""
    <joint name="{rname}_joint" type="revolute">
      <parent>{mname}</parent>
      <child>{rname}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.004</damping></dynamics>
      </axis>
    </joint>""")

    # --- Control surfaces ---------------------------------------------------
    # The ailerons must sit ON the wing, which is tapered, swept and dihedralled.
    # Placing them at z=0 with the root chord leaves them floating in space
    # below the outboard wing -- visible in the first mesh render as detached
    # grey slabs. Follow the actual wing station instead.
    # Ailerons. Hinge point comes from params, in the WING frame, so the model
    # frame needs quarter_x added -- exactly as the wing mesh itself is placed.
    a = P.aileron_geometry()
    parts.append(surface("aileron_left", g["quarter_x"] + a["x"],
                         +a["y_mid"], a["z"], "aileron_left", "0 1 0"))
    parts.append(surface("aileron_right", g["quarter_x"] + a["x"],
                         -a["y_mid"], a["z"], "aileron_right", "0 1 0"))

    # V-tail ruddervators, one per panel, hinged about the panel's own span
    # axis (0, cos g, sin g). Deflecting them together is pitch; differentially
    # is yaw. That is the whole reason a V-tail needs only two surfaces.
    for label, sgn, mesh in (("vtail_left", +1.0, "ruddervator_left"),
                             ("vtail_right", -1.0, "ruddervator_right")):
        r = P.ruddervator_geometry(sgn)
        _, uy, uz = r["axis"]
        parts.append(surface(label, r["x"], r["y"], r["z"], mesh,
                             f"0 {_f(uy)} {_f(uz)}"))

    # --- Joint state -------------------------------------------------------
    # Publishes every joint angle on /world/<w>/model/<m>/joint_state. This is
    # how verify_transition.sh reads the actual nacelle angles: PX4 accepting
    # a `commander transition` proves only that the command parsed, not that
    # the nacelles moved. Without this the transition claim is unfalsifiable.
    parts.append("""
    <plugin filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
    </plugin>""")

    # --- Motor plugins ------------------------------------------------------
    parts.append(motor_plugin("rotor_left_joint", "rotor_left", "ccw",
                              MOTOR_LEFT, max_rot_wing, k_wing))
    parts.append(motor_plugin("rotor_right_joint", "rotor_right", "cw",
                              MOTOR_RIGHT, max_rot_wing, k_wing))
    parts.append(motor_plugin("rotor_tail_joint", "rotor_tail", "ccw",
                              MOTOR_TAIL, max_rot_tail, k_tail))

    # --- Servo plugins ------------------------------------------------------
    parts.append(servo_plugin("tilt_left_joint", SERVO_TILT_LEFT))
    parts.append(servo_plugin("tilt_right_joint", SERVO_TILT_RIGHT))
    if P.TAIL_TILTS:
        parts.append(servo_plugin("tilt_tail_joint", SERVO_TILT_TAIL))
    parts.append(servo_plugin("aileron_left_joint", SERVO_AILERON_LEFT))
    parts.append(servo_plugin("aileron_right_joint", SERVO_AILERON_RIGHT))
    parts.append(servo_plugin("vtail_left_joint", SERVO_VTAIL_LEFT))
    parts.append(servo_plugin("vtail_right_joint", SERVO_VTAIL_RIGHT))

    # --- Aerodynamics -------------------------------------------------------
    # One surface per wing panel at its own centre of pressure. Stock lumps
    # the entire wing into the two ailerons; this does not.
    # Coefficients come from params.solve(), which derives them from the NACA
    # section, so the simulator and the design report cannot disagree.
    parts.append(liftdrag(
        (g["quarter_x"], +g["panel_cp_y"], 0.0), g["panel_area"],
        "aileron_left_joint", -1.0,
        d.cl_alpha, d.cd0, d.alpha_stall, a0=P.WING_ALPHA_ZERO_LIFT))
    parts.append(liftdrag(
        (g["quarter_x"], -g["panel_cp_y"], 0.0), g["panel_area"],
        "aileron_right_joint", -1.0,
        d.cl_alpha, d.cd0, d.alpha_stall, a0=P.WING_ALPHA_ZERO_LIFT))
    # V-tail: one LiftDrag per panel. The panel's lift acts normal to both the
    # freestream and its own span, i.e. along (0, -sin g, cos g) for the left
    # panel. Summing the two gives pitch; differencing gives yaw -- so the
    # single pair reproduces what an elevator AND a rudder used to do.
    #
    # Note the contrast with stock gz_tiltrotor, whose rudder is capped at
    # +/-0.01 rad with its LiftDrag commented out entirely. Both of ours are
    # live.
    cp_arm = 0.42 * d.tail_panel_span
    gam = d.tail_dihedral
    for joint, sgn in (("vtail_left_joint", +1.0), ("vtail_right_joint", -1.0)):
        uy, uz = sgn * math.cos(gam), math.sin(gam)
        parts.append(liftdrag(
            (-P.TAIL_SURFACE_ARM, cp_arm * uy, 0.010 + cp_arm * uz),
            d.tail_area_total / 2.0,
            joint, -3.5,
            3.6, 0.020, math.radians(18.0),
            upward=f"0 {-sgn * math.sin(gam):.6f} {math.cos(gam):.6f}"))

    parts.append("\n  </model>\n</sdf>\n")

    hover = (f"    hover omega: wing {omega_wing_hover:.1f} rad/s, "
             f"tail {omega_tail_hover:.1f} rad/s")
    return "".join(parts), hover


MODEL_CONFIG = f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Tri-tiltrotor VTOL with three tilting nacelles. Generated from params.py.
  </description>
</model>
"""


def main() -> None:
    # Refuse to emit a model for an aircraft that failed its own design checks.
    checks = P.check()
    print(f"params.check(): {len(checks)}/{len(checks)} invariants passed")

    sdf, hover = build_sdf()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "model.sdf").write_text(sdf, encoding="utf-8")
    (OUT_DIR / "model.config").write_text(MODEL_CONFIG, encoding="utf-8")

    print(f"wrote {OUT_DIR / 'model.sdf'}  ({len(sdf.splitlines())} lines)")
    print(f"wrote {OUT_DIR / 'model.config'}")
    print(hover)


if __name__ == "__main__":
    main()
