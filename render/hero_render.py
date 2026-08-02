#!/usr/bin/env python3
"""
Render the tri-tiltrotor in Blender, headless, from render/assembly.json.

Why Blender and not Gazebo: Gazebo's renderer exists to show you a simulation,
not to make a picture someone will buy from. This produces the hero stills for
the marketplace listing, the GitHub README and the LinkedIn carousel.

Part placement comes entirely from assembly.json, which is generated from
cad/params.py. Blender holds no geometry numbers of its own, so a change to the
design cannot leave the renders showing the old aircraft.

Run:
    "D:/Blender/blender.exe" -b --factory-startup \
        --python render/hero_render.py -- --pose transition --samples 128
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

# Blender passes script args after a bare "--".
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--pose", default="transition",
                choices=["hover", "transition", "cruise"])
ap.add_argument("--samples", type=int, default=96)
ap.add_argument("--res", type=int, default=1920)
ap.add_argument("--views", default="hero,front,top,detail")
ap.add_argument("--engine", default="CYCLES")
ap.add_argument("--ground", action="store_true",
                help="add a floor plane. Off by default: every pose here is "
                     "an in-flight attitude, and a lit floor under a flying "
                     "aircraft reads as haze rather than as ground.")
ap.add_argument("--save-blend", default="",
                help="write a .blend and skip rendering, for interactive "
                     "inspection of the assembly")
args = ap.parse_args(argv)

PROJECT = Path(__file__).resolve().parent.parent
MANIFEST = PROJECT / "render" / "assembly.json"
OUTDIR = PROJECT / "media" / "renders"
OUTDIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# scene setup
# ---------------------------------------------------------------------------

def clear_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def _set(node, name, value) -> None:
    """Set a socket only if this Blender version has it.

    Principled BSDF gained Coat/Sheen sockets in 4.0 and renames them
    occasionally; a missing socket should soften the look, not kill the render.
    """
    if name in node.inputs:
        node.inputs[name].default_value = value


def _new_mat(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    return m, nt, nt.nodes["Principled BSDF"]


def _object_coords(nt, scale=1.0, rot=(0.0, 0.0, 0.0)):
    """Object-space texture coordinates, so the weave and the layer lines stay
    locked to the part instead of swimming when a nacelle rotates with tilt."""
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (scale, scale, scale)
    mp.inputs["Rotation"].default_value = rot
    nt.links.new(mp.inputs["Vector"], tc.outputs["Object"])
    return mp.outputs["Vector"]


def make_materials() -> dict:
    mats = {}

    # -- painted composite skin ------------------------------------------
    # Wing, tail and control surfaces. Two-layer: pigment under a clear coat,
    # which is what actually makes a curved lofted surface read as curved --
    # the coat gives a tight specular that tracks the section, and the diffuse
    # base underneath keeps it from looking like chrome.
    m, nt, b = _new_mat("skin")
    _set(b, "Base Color", (0.82, 0.84, 0.87, 1.0))
    _set(b, "Roughness", 0.34)
    _set(b, "Metallic", 0.0)
    _set(b, "Coat Weight", 0.65)
    _set(b, "Coat Roughness", 0.08)
    _set(b, "Sheen Weight", 0.05)
    # Faint orange-peel, the texture every real painted surface has.
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 190.0
    noise.inputs["Detail"].default_value = 4.0
    nt.links.new(noise.inputs["Vector"], _object_coords(nt))
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.06
    nt.links.new(bump.inputs["Height"], noise.outputs["Fac"])
    nt.links.new(b.inputs["Normal"], bump.outputs["Normal"])
    mats["skin"] = m

    # -- fuselage shell ---------------------------------------------------
    m, nt, b = _new_mat("body")
    _set(b, "Base Color", (0.028, 0.038, 0.062, 1.0))
    _set(b, "Roughness", 0.26)
    _set(b, "Metallic", 0.0)
    _set(b, "Coat Weight", 1.0)
    _set(b, "Coat Roughness", 0.05)
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 150.0
    noise.inputs["Detail"].default_value = 3.0
    nt.links.new(noise.inputs["Vector"], _object_coords(nt))
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.05
    nt.links.new(bump.inputs["Height"], noise.outputs["Fac"])
    nt.links.new(b.inputs["Normal"], bump.outputs["Normal"])
    mats["body"] = m

    # -- woven carbon -----------------------------------------------------
    # Booms, spar structure and linkages. A checker at tow scale, mixed
    # between two near-blacks and driving a bump, is enough to read as twill
    # at these camera distances; the anisotropy is what sells it, because
    # carbon's highlight smears along the tow rather than staying round.
    m, nt, b = _new_mat("carbon")
    _set(b, "Roughness", 0.28)
    # Carbon fibre is not a metal. Driving Metallic up gave the booms a
    # glassy, see-through look because a metal with a dark base colour
    # reflects almost everything and transmits the rest of the impression.
    _set(b, "Metallic", 0.10)
    # A full-strength coat on a near-black base made a closed tube look
    # transparent -- the mesh is watertight, the highlight was just reading as
    # a far wall seen through a near one.
    _set(b, "Anisotropic", 0.30)
    _set(b, "Coat Weight", 0.45)
    _set(b, "Coat Roughness", 0.12)
    weave = nt.nodes.new("ShaderNodeTexChecker")
    weave.inputs["Color1"].default_value = (0.010, 0.011, 0.013, 1.0)
    weave.inputs["Color2"].default_value = (0.030, 0.032, 0.038, 1.0)
    # ~0.45 mm tows. At 520 the squares were 2 mm and read as a chessboard.
    weave.inputs["Scale"].default_value = 2200.0
    nt.links.new(weave.inputs["Vector"], _object_coords(nt))
    nt.links.new(b.inputs["Base Color"], weave.outputs["Color"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.18
    nt.links.new(bump.inputs["Height"], weave.outputs["Fac"])
    nt.links.new(b.inputs["Normal"], bump.outputs["Normal"])
    mats["carbon"] = m

    # -- printed PLA ------------------------------------------------------
    # Nacelle cradle, yoke, formers, tail motor mount. These are the parts a
    # buyer actually prints, so they should look printed: banded, matte, and
    # slightly translucent at the edges. Band period is set well above the
    # real 0.2 mm layer height on purpose -- at 1920 px across a 2 m aircraft
    # a true layer line lands under one pixel and aliases into moire.
    m, nt, b = _new_mat("printed")
    # Deeper and more saturated than the colour you actually want: AgX pulls
    # saturation out of bright chroma, and (0.72, 0.25, 0.02) came back off
    # the render as pale salmon rather than as orange PLA.
    _set(b, "Base Color", (0.46, 0.105, 0.008, 1.0))
    _set(b, "Roughness", 0.56)
    _set(b, "Metallic", 0.0)
    _set(b, "Coat Weight", 0.18)
    _set(b, "Coat Roughness", 0.35)
    # Rotate the band axis so lines run around Z, the way the part prints.
    layers = nt.nodes.new("ShaderNodeTexWave")
    layers.wave_type = "BANDS"
    layers.bands_direction = "Z"
    layers.inputs["Scale"].default_value = 820.0
    layers.inputs["Distortion"].default_value = 0.0
    nt.links.new(layers.inputs["Vector"], _object_coords(nt))
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.38
    nt.links.new(bump.inputs["Height"], layers.outputs["Fac"])
    nt.links.new(b.inputs["Normal"], bump.outputs["Normal"])
    mats["printed"] = m

    # -- propeller --------------------------------------------------------
    # Carbon-filled nylon: darker and glossier than the structural carbon,
    # with no visible weave because injection-moulded blades have none.
    m, nt, b = _new_mat("prop")
    _set(b, "Base Color", (0.016, 0.017, 0.020, 1.0))
    _set(b, "Roughness", 0.24)
    _set(b, "Metallic", 0.1)
    _set(b, "Coat Weight", 0.8)
    _set(b, "Coat Roughness", 0.10)
    mats["prop"] = m

    return mats


def import_stl(path: Path):
    before = set(bpy.data.objects)
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    new = list(set(bpy.data.objects) - before)
    return new


def build_aircraft(manifest: dict, mats: dict):
    objs = []
    by_part = {}
    for p in manifest["parts"]:
        path = PROJECT / p["mesh"]
        if not path.exists():
            print(f"  MISSING {path}")
            continue
        for ob in import_stl(path):
            s = p.get("scale", 1.0)
            ob.scale = (s, -s if p.get("mirror") else s, s)
            ob.location = Vector(p["loc"])
            ob.rotation_euler = tuple(p["rot"])
            ob.data.materials.clear()
            ob.data.materials.append(mats[p["material"]])
            # Smooth shading; STL is faceted by construction.
            for poly in ob.data.polygons:
                poly.use_smooth = True
            # STL carries no normals, so smooth-shade it and split the hard
            # edges back out. EDGE_SPLIT is deprecated in recent Blender; fall
            # back to flat shading rather than losing the part entirely.
            try:
                mod = ob.modifiers.new("smooth", "EDGE_SPLIT")
                mod.split_angle = math.radians(35)
            except (RuntimeError, TypeError):
                for poly in ob.data.polygons:
                    poly.use_smooth = False
            objs.append(ob)
            by_part.setdefault(p["name"], []).append(ob)
    return objs, by_part


def setup_world() -> None:
    """A vertical gradient rather than a flat colour.

    This is not decoration. The clear coat on the skin and the anisotropic
    carbon both reflect the world, so a flat environment gives every curved
    surface the same specular value and the loft goes visually flat. A
    gradient puts a bright band in those reflections that moves as the
    surface turns, which is what makes the wing look like a wing.
    """
    world = bpy.data.worlds.new("world")
    bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    bg = nt.nodes["Background"]
    bg.inputs["Strength"].default_value = 1.0

    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(sep.inputs["Vector"], tc.outputs["Generated"])

    # In a world shader the coordinate is a *direction*, so Z runs -1 (nadir)
    # to +1 (zenith). Feeding that straight into a ColorRamp clamps the whole
    # lower hemisphere to one value and the gradient disappears exactly where
    # the camera is looking. Remap to 0..1 first.
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.inputs["From Min"].default_value = -1.0
    rng.inputs["From Max"].default_value = 1.0
    nt.links.new(rng.inputs["Value"], sep.outputs["Z"])

    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.18
    ramp.color_ramp.elements[0].color = (0.055, 0.062, 0.078, 1.0)   # below
    ramp.color_ramp.elements[1].position = 0.92
    ramp.color_ramp.elements[1].color = (0.007, 0.009, 0.014, 1.0)   # zenith
    nt.links.new(ramp.inputs["Fac"], rng.outputs["Result"])
    nt.links.new(bg.inputs["Color"], ramp.outputs["Color"])


def add_lights() -> None:
    # Three-point rig plus a broad overhead softbox. Key is a large area light
    # so the lofted surfaces show their curvature -- a point light makes a
    # NACA section look flat. The rim is the one that matters most here: it
    # separates a near-black fuselage from a near-black background, which no
    # amount of key light can do.
    for name, loc, energy, size, rot in (
        ("key",  (2.6, -3.2, 3.0), 1000.0, 3.2,
         (math.radians(48), 0, math.radians(38))),
        ("fill", (-3.4, -2.2, 1.4), 320.0, 4.5,
         (math.radians(72), 0, math.radians(-56))),
        ("rim",  (-1.6, 3.6, 2.2), 1100.0, 2.5,
         (math.radians(60), 0, math.radians(196))),
        ("top",  (0.0, 0.4, 4.2), 420.0, 7.0, (0.0, 0.0, 0.0)),
    ):
        d = bpy.data.lights.new(name, type="AREA")
        d.energy = energy
        d.size = size
        ob = bpy.data.objects.new(name, d)
        ob.location = loc
        ob.rotation_euler = rot
        bpy.context.collection.objects.link(ob)


def add_ground(z: float = -0.35) -> None:
    """Semi-gloss floor, and a radial fade so the plane has no visible edge.

    A matte plane reads as a grey card. A slightly reflective one puts a soft
    inverted image of the aircraft under it, which is what separates a
    product shot from a screenshot -- and it costs nothing to render.
    """
    bpy.ops.mesh.primitive_plane_add(size=60, location=(0, 0, z))
    g = bpy.context.object
    m = bpy.data.materials.new("ground")
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Roughness"].default_value = 0.34
    b.inputs["Metallic"].default_value = 0.0

    # Radial ramp: lighter under the aircraft, falling to the world colour
    # well before the plane's edge, so the horizon never shows as a hard line.
    tc = nt.nodes.new("ShaderNodeTexCoord")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = "SPHERICAL"
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (0.10, 0.10, 0.10)
    nt.links.new(mp.inputs["Vector"], tc.outputs["Object"])
    nt.links.new(grad.inputs["Vector"], mp.outputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.004, 0.005, 0.007, 1.0)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (0.038, 0.042, 0.050, 1.0)
    nt.links.new(ramp.inputs["Fac"], grad.outputs["Fac"])
    nt.links.new(b.inputs["Base Color"], ramp.outputs["Color"])
    g.data.materials.append(m)


def add_camera(loc, look_at=(0, 0, 0), lens=70.0):
    cam_d = bpy.data.cameras.new("cam")
    cam_d.lens = lens
    cam = bpy.data.objects.new("cam", cam_d)
    cam.location = loc
    bpy.context.collection.objects.link(cam)
    # Point it at the target.
    direction = Vector(look_at) - Vector(loc)
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def world_bounds(objs):
    """World-space bounding box of a set of objects."""
    pts = [ob.matrix_world @ Vector(c) for ob in objs for c in ob.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts),
                 min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts),
                 max(p.z for p in pts)))
    return lo, hi


def box_corners(lo, hi):
    return [Vector((x, y, z))
            for x in (lo.x, hi.x) for y in (lo.y, hi.y) for z in (lo.z, hi.z)]


def frame_camera(direction, lo, hi, lens, res_x, res_y, margin=1.06):
    """Place the camera along `direction` at the distance that just fits the box.

    Hardcoded camera positions are a latent bug in a parametric project: the
    span, the tilt pose and the nacelle stations all come from params.py, so
    any of them can change and silently push a wingtip out of frame.

    Fitting the *bounding sphere* is the easy version and it is wrong here --
    a 2 m span by 0.28 m thick aircraft has a sphere far larger than its
    silhouette from any useful angle, so the aircraft ends up a speck in the
    middle of the frame. This projects the eight box corners onto the camera's
    own axes and solves the distance each one demands, which fits the actual
    silhouette.
    """
    d = Vector(direction).normalized()
    centre = (lo + hi) * 0.5

    forward = -d
    up_hint = Vector((0.0, 0.0, 1.0))
    if abs(forward.dot(up_hint)) > 0.99:              # looking straight down
        up_hint = Vector((0.0, 1.0, 0.0))
    right = forward.cross(up_hint).normalized()
    up = right.cross(forward).normalized()

    sensor = 36.0                                     # Blender default, mm
    tan_h = (sensor * 0.5) / lens
    tan_v = (sensor * 0.5) * (res_y / res_x) / lens

    dist = 0.0
    for p in box_corners(lo, hi):
        v = p - centre
        # `depth` is how far this corner sits toward the camera; it eats into
        # the distance available, so a near corner needs the camera pushed back.
        depth = v.dot(d)
        dist = max(dist,
                   abs(v.dot(right)) * margin / tan_h + depth,
                   abs(v.dot(up)) * margin / tan_v + depth)

    return add_camera(tuple(centre + d * dist), tuple(centre), lens)


def configure_render(engine: str, samples: int, res: int) -> None:
    sc = bpy.context.scene
    sc.render.resolution_x = res
    sc.render.resolution_y = int(res * 9 / 16)
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = "PNG"

    # Colour management. AgX is the 4.x+ default and rolls highlights off far
    # more gracefully than Standard, which clips the specular on the coat to
    # flat white. The contrast look puts back the punch AgX takes out.
    try:
        sc.view_settings.view_transform = "AgX"
        for look in ("AgX - Medium High Contrast", "Medium High Contrast"):
            try:
                sc.view_settings.look = look
                break
            except (TypeError, ValueError):
                continue
    except (AttributeError, TypeError, ValueError) as exc:
        print(f"  colour management left at default: {exc}")

    try:
        sc.render.engine = engine
    except Exception:
        sc.render.engine = "BLENDER_EEVEE_NEXT"

    if sc.render.engine == "CYCLES":
        sc.cycles.samples = samples
        sc.cycles.use_denoising = True
        # Prefer GPU; fall back silently to CPU rather than failing the render.
        try:
            prefs = bpy.context.preferences.addons["cycles"].preferences
            for dt in ("OPTIX", "CUDA"):
                prefs.compute_device_type = dt
                prefs.get_devices()
                found = [d for d in prefs.devices if d.type == dt]
                if found:
                    for d in prefs.devices:
                        d.use = (d.type == dt)
                    sc.cycles.device = "GPU"
                    print(f"  cycles device: {dt} -> {[d.name for d in found]}")
                    break
            else:
                sc.cycles.device = "CPU"
                print("  cycles device: CPU (no GPU found)")
        except Exception as exc:
            print(f"  GPU setup skipped: {exc}")
            sc.cycles.device = "CPU"


# Direction the camera sits in, relative to the subject. Distance is solved
# per-render from the subject's own bounding sphere, so these never need
# retuning when the design changes.
VIEWS = {
    # name:     (direction from subject,   lens, margin)
    "hero":     ((0.78, -1.00, 0.34),      80.0, 1.10),
    "front":    ((0.04, -1.00, 0.11),      85.0, 1.06),
    "top":      ((0.00, -0.14, 1.00),      80.0, 1.06),
    "detail":   ((0.70, -1.00, 0.42),      90.0, 1.55),
}

# Views that frame one part instead of the whole aircraft.
SUBJECTS = {"detail": "nacelle_cradle_left"}


def main() -> None:
    manifests = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest = manifests[args.pose]

    print(f"pose      : {args.pose}  (tilt {manifest['tilt_deg']:.0f} deg)")
    print(f"aircraft  : {manifest['aircraft']}")

    clear_scene()
    mats = make_materials()
    objs, by_part = build_aircraft(manifest, mats)
    print(f"imported  : {len(objs)} objects")
    if not objs:
        raise SystemExit("nothing imported -- run cad/gen_geometry.py first")

    lo, hi = world_bounds(objs)
    print(f"extent    : {(hi - lo).x:.3f} x {(hi - lo).y:.3f} x "
          f"{(hi - lo).z:.3f} m")

    setup_world()
    add_lights()
    if args.ground:
        # Sit the floor just under the lowest point rather than at a fixed
        # height -- the tilt pose changes how far the nacelles hang down.
        add_ground(lo.z - 0.30)
    configure_render(args.engine, args.samples, args.res)

    sc = bpy.context.scene
    res_x, res_y = sc.render.resolution_x, sc.render.resolution_y

    if args.save_blend:
        # Park a camera so the file opens on something sensible.
        direction, lens, margin = VIEWS["hero"]
        frame_camera(direction, lo, hi, lens, res_x, res_y, margin)
        out = Path(args.save_blend)
        out.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(out))
        print(f"saved {out}")
        print("objects in scene:")
        for ob in sorted(bpy.data.objects, key=lambda o: o.name):
            if ob.type == "MESH":
                bb = ob.bound_box
                print(f"  {ob.name:28s} loc=({ob.location.x:+.3f}, "
                      f"{ob.location.y:+.3f}, {ob.location.z:+.3f})")
        return

    for view in args.views.split(","):
        view = view.strip()
        if view not in VIEWS:
            continue
        direction, lens, margin = VIEWS[view]

        # Most views frame the whole aircraft; the detail view frames one part.
        subject = SUBJECTS.get(view)
        if subject and by_part.get(subject):
            s_lo, s_hi = world_bounds(by_part[subject])
        else:
            s_lo, s_hi = lo, hi

        frame_camera(direction, s_lo, s_hi, lens, res_x, res_y, margin)
        out = OUTDIR / f"{args.pose}_{view}.png"
        sc.render.filepath = str(out)
        print(f"rendering {view} -> {out.name}")
        bpy.ops.render.render(write_still=True)

    print("done")


main()
