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


def make_materials() -> dict:
    def mat(name, base, rough, metal=0.0, spec=0.5):
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        b = m.node_tree.nodes["Principled BSDF"]
        b.inputs["Base Color"].default_value = (*base, 1.0)
        b.inputs["Roughness"].default_value = rough
        b.inputs["Metallic"].default_value = metal
        return m

    return {
        "skin":    mat("skin",    (0.88, 0.89, 0.91), 0.28, 0.05),
        "body":    mat("body",    (0.09, 0.12, 0.17), 0.35, 0.15),
        "printed": mat("printed", (0.85, 0.35, 0.05), 0.62, 0.00),
        "prop":    mat("prop",    (0.05, 0.05, 0.06), 0.40, 0.20),
        "carbon":  mat("carbon",  (0.045, 0.048, 0.055), 0.30, 0.30),
    }


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
            mod = ob.modifiers.new("smooth", "EDGE_SPLIT")
            mod.split_angle = math.radians(35)
            objs.append(ob)
    return objs


def setup_world() -> None:
    world = bpy.data.worlds.new("world")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.05, 0.07, 0.10, 1.0)
    bg.inputs["Strength"].default_value = 1.0


def add_lights() -> None:
    # Three-point-ish rig. Key is a large area light so the lofted surfaces
    # show their curvature -- a point light makes a NACA section look flat.
    for name, loc, energy, size, rot in (
        ("key",  (2.6, -3.2, 3.0), 900.0, 3.0, (math.radians(48), 0, math.radians(38))),
        ("fill", (-3.4, -2.2, 1.4), 240.0, 4.0, (math.radians(72), 0, math.radians(-56))),
        ("rim",  (-1.6, 3.6, 2.2), 600.0, 2.5, (math.radians(60), 0, math.radians(196))),
    ):
        d = bpy.data.lights.new(name, type="AREA")
        d.energy = energy
        d.size = size
        ob = bpy.data.objects.new(name, d)
        ob.location = loc
        ob.rotation_euler = rot
        bpy.context.collection.objects.link(ob)


def add_ground() -> None:
    bpy.ops.mesh.primitive_plane_add(size=60, location=(0, 0, -0.35))
    g = bpy.context.object
    m = bpy.data.materials.new("ground")
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (0.035, 0.04, 0.05, 1.0)
    b.inputs["Roughness"].default_value = 0.85
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


def configure_render(engine: str, samples: int, res: int) -> None:
    sc = bpy.context.scene
    sc.render.resolution_x = res
    sc.render.resolution_y = int(res * 9 / 16)
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = "PNG"

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


VIEWS = {
    # name:      (camera location,        look-at,          lens)
    "hero":      ((2.35, -2.75, 1.10),  (0.0, 0.0, 0.10),  70.0),
    "front":     ((0.05, -3.60, 0.35),  (0.0, 0.0, 0.10),  85.0),
    "top":       ((0.00, -0.02, 3.40),  (0.0, 0.0, 0.00),  60.0),
    "detail":    ((0.62, -0.78, 0.42),  (0.173, 0.40, 0.05), 95.0),
}


def main() -> None:
    manifests = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest = manifests[args.pose]

    print(f"pose      : {args.pose}  (tilt {manifest['tilt_deg']:.0f} deg)")
    print(f"aircraft  : {manifest['aircraft']}")

    clear_scene()
    mats = make_materials()
    objs = build_aircraft(manifest, mats)
    print(f"imported  : {len(objs)} objects")
    if not objs:
        raise SystemExit("nothing imported -- run cad/gen_geometry.py first")

    setup_world()
    add_lights()
    add_ground()
    configure_render(args.engine, args.samples, args.res)

    if args.save_blend:
        # Park a camera so the file opens on something sensible.
        loc, look, lens = VIEWS["hero"]
        add_camera(loc, look, lens)
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
        loc, look, lens = VIEWS[view]
        add_camera(loc, look, lens)
        out = OUTDIR / f"{args.pose}_{view}.png"
        bpy.context.scene.render.filepath = str(out)
        print(f"rendering {view} -> {out.name}")
        bpy.ops.render.render(write_still=True)

    print("done")


main()
