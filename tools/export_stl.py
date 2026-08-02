# Export every part of the tri-tiltrotor assembly to STL, in millimetres.
#
# Run headless -- no GUI, no clicking:
#     D:\FreeCAD\bin\freecadcmd.exe tools\export_stl.py
# or  powershell -File tools\export_stl.ps1     (regenerates the STEPs first)
#
# ---------------------------------------------------------------------------
# THE UNIT TRAP, which is the entire reason this script exists
# ---------------------------------------------------------------------------
# build123d models the airframe in METRES but writes STEP headers that declare
# MILLIMETRE units. So cad/out/assembly/wing.step says "2.126" and means
# 2.126 m, while every reader on earth reads 2.126 mm. The nacelle hardware is
# genuinely modelled in mm and carries an _mm suffix to say so.
#
# Same rule as tools/load_assembly.FCMacro: scale x1000 unless the name ends
# in _mm. Getting it backwards puts a 2 m wing next to a 2 mm one.
#
# The sim meshes in sim/models/tri_tiltrotor/meshes/ are NOT a substitute --
# those are raw metres (a slicer reads the wing as 2.1 mm long) and Gazebo is
# the only thing that wants them that way.
#
# ---------------------------------------------------------------------------
# TWO OUTPUT SETS, because they answer different questions
# ---------------------------------------------------------------------------
#   out/stl/assembly/  each part AT ITS FLIGHT POSITION -- load the folder and
#                      you get an assembled aircraft. For render, review, and
#                      anything that needs parts to line up.
#   out/stl/print/     each part translated so its bounding box sits on the
#                      origin with z=0 -- what a slicer wants. Loading a
#                      flight-positioned part into a slicer drops it 300 mm
#                      off the plate and it silently auto-arranges.
#
# Plus tri_tiltrotor_full.stl: the whole aircraft fused into one mesh, for
# marketplace previews and anything that takes a single file.
import os
import sys

import FreeCAD as App
import Part
import Mesh
import MeshPart

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
ASSY = os.path.join(ROOT, "cad", "out", "assembly")
OUT = os.path.join(ROOT, "cad", "out", "stl")
ASSY_OUT = os.path.join(OUT, "assembly")
PRINT_OUT = os.path.join(OUT, "print")

# Largest common FDM bed (Bambu A1 / P1S / X1C are all 256 mm cubes). Parts
# over this are reported, not silently written as unprintable files.
BED_MM = 256.0


def tight_bbox(shape):
    """Actual extents of the solid.

    ⚠ Shape.BoundBox is NOT tight. On NURBS it bounds the control hull, not
    the surface, and on these lofted parts it is wildly loose -- it reported
    the 52 mm-thick aileron as 1041 mm thick. Two things went wrong downstream
    before this was caught: the size-based mesh tolerance came out ~4x too
    coarse, and the print copies were translated by a bogus minimum so they
    did not land on the origin at all. optimalBoundingBox() tessellates first
    and gives the real answer.
    """
    try:
        return shape.optimalBoundingBox()
    except Exception:
        return shape.BoundBox


def mesh_params(diag):
    """Tessellation tolerance scaled to part size.

    A fixed deflection is wrong at both ends: 0.03 mm on a 2.1 m wing is
    millions of triangles, and 0.5 mm on a 38 mm bearing seat turns a round
    bore into a visible polygon. Tie it to the diagonal instead and every part
    lands in the same visual quality band.

    Angular deflection is what actually binds on the small round features --
    bearing seats and bolt bosses -- so it is the tighter of the two knobs.
    """
    linear = min(0.25, max(0.03, diag / 8000.0))
    return linear, 0.12                      # angular deflection in radians


def main():
    if not os.path.isdir(ASSY):
        raise SystemExit("no assembly dir -- run cad/gen_assembly_step.py first")
    files = sorted(f for f in os.listdir(ASSY) if f.lower().endswith(".step"))
    if not files:
        raise SystemExit("no STEP files -- run cad/gen_assembly_step.py first")

    for d in (ASSY_OUT, PRINT_OUT):
        if not os.path.isdir(d):
            os.makedirs(d)
        for f in os.listdir(d):
            if f.lower().endswith(".stl"):
                os.remove(os.path.join(d, f))

    print("%-24s %9s %8s %9s  %s" %
          ("part", "tris", "solid", "mm^3", "bbox mm (x y z)"))
    print("-" * 86)

    shapes = []
    oversize = []
    leaky = []
    total_tris = 0

    for fn in files:
        name = os.path.splitext(fn)[0]
        shape = Part.Shape()
        shape.read(os.path.join(ASSY, fn))

        if not name.endswith("_mm"):
            m = App.Matrix()
            m.scale(1000.0, 1000.0, 1000.0)
            shape = shape.transformGeometry(m)
        else:
            name = name[:-3]             # the suffix was a unit tag, not a name

        bb = tight_bbox(shape)
        linear, angular = mesh_params(bb.DiagonalLength)

        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=linear,
                                      AngularDeflection=angular, Relative=False)
        mesh.write(os.path.join(ASSY_OUT, name + ".stl"))

        # Print copy: same mesh, moved to sit on the origin with z=0. Measured
        # off the MESH bbox, which is exact by construction -- see tight_bbox.
        mb = mesh.BoundBox
        pm = mesh.copy()
        pm.translate(-mb.XMin, -mb.YMin, -mb.ZMin)
        pm.write(os.path.join(PRINT_OUT, name + ".stl"))
        bb = mb

        solid = mesh.isSolid()
        if not solid:
            leaky.append(name)
        if max(bb.XLength, bb.YLength, bb.ZLength) > BED_MM:
            oversize.append((name, max(bb.XLength, bb.YLength, bb.ZLength)))

        total_tris += mesh.CountFacets
        shapes.append(shape)
        print("%-24s %9d %8s %9.0f  %6.1f x %6.1f x %6.1f" %
              (name, mesh.CountFacets, "yes" if solid else "NO",
               shape.Volume, bb.XLength, bb.YLength, bb.ZLength))

    # --- whole aircraft, one file ------------------------------------------
    # Compound, not fuse: a boolean union of 19 lofted shells takes minutes and
    # can fail on a self-touching prop, and nothing downstream needs the parts
    # to actually be one solid.
    comp = Part.makeCompound(shapes)
    linear, angular = mesh_params(tight_bbox(comp).DiagonalLength)
    full = MeshPart.meshFromShape(Shape=comp, LinearDeflection=linear,
                                  AngularDeflection=angular, Relative=False)
    full.write(os.path.join(OUT, "tri_tiltrotor_full.stl"))
    bb = full.BoundBox

    print("-" * 86)
    print("%-24s %9d %8s %9s  %6.1f x %6.1f x %6.1f" %
          ("tri_tiltrotor_full", full.CountFacets, "-", "-",
           bb.XLength, bb.YLength, bb.ZLength))
    print("\n%d parts -> %s" % (len(files), OUT))
    print("   assembly/  flight positions, load together for the whole aircraft")
    print("   print/     each part on the origin, ready to slice")

    if leaky:
        print("\nNOT watertight (%d): %s" % (len(leaky), ", ".join(leaky)))
    if oversize:
        print("\nOver the %.0f mm bed -- these need splitting before printing:"
              % BED_MM)
        for name, d in sorted(oversize, key=lambda t: -t[1]):
            print("   %-22s %.0f mm" % (name, d))


main()
