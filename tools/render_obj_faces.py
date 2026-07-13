"""Render selected Weft OBJ face groups with nearby topology context.

Usage:
  blender --factory-startup --background --python tools/render_obj_faces.py -- \
      mesh.obj 632,1310 output.png
"""

import sys
from pathlib import Path

import bpy
from mathutils import Vector


def read_obj(path):
    verts = []
    faces = []
    face_id = 0
    with Path(path).open("r", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                _, x, y, z, *_ = line.split()
                verts.append(Vector((float(x), float(y), float(z))))
            elif line.startswith("g face_"):
                try:
                    face_id = int(line.split("_", 1)[1])
                except ValueError:
                    face_id = 0
            elif line.startswith("f "):
                indices = [int(p.split("/", 1)[0]) - 1 for p in line.split()[1:]]
                faces.append((indices, face_id))
    return verts, faces


def mesh_object(name, source_verts, source_faces, material):
    used = {}
    verts = []
    faces = []
    for poly in source_faces:
        local = []
        for index in poly:
            if index not in used:
                used[index] = len(verts)
                verts.append(tuple(source_verts[index]))
            local.append(used[index])
        faces.append(local)
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(material)
    # Keep Blender's hidden render tessellation out of the QA image.  With
    # show_all_edges enabled Blender draws every triangle it creates
    # internally for an n-gon, which looks like exported triangle soup even
    # though those diagonals are not present in the OBJ.  The dedicated
    # wireframe object below renders the actual polygon boundaries.
    obj.show_wire = False
    obj.show_all_edges = False
    return obj


def edge_object(name, source_verts, source_faces, material, thickness):
    """Render only exported polygon edges, never Blender's n-gon tessellation."""
    edges = set()
    for poly in source_faces:
        for index, a in enumerate(poly):
            b = poly[(index + 1) % len(poly)]
            if a != b:
                edges.add((min(a, b), max(a, b)))
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 1
    curve.bevel_depth = thickness
    curve.bevel_resolution = 0
    curve.materials.append(material)
    for a, b in edges:
        spline = curve.splines.new("POLY")
        spline.points.add(1)
        spline.points[0].co = (*source_verts[a], 1.0)
        spline.points[1].co = (*source_verts[b], 1.0)
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    return obj


def main():
    args = sys.argv[sys.argv.index("--") + 1 :]
    obj_path, face_csv, output = args[:3]
    selected_ids = (None if face_csv.lower() == "all" else
                    {int(value) for value in face_csv.split(",")})
    verts, records = read_obj(obj_path)
    selected = [poly for poly, fid in records
                if selected_ids is None or fid in selected_ids]
    if not selected:
        raise RuntimeError(f"no polygons for face ids {selected_ids}")
    selected_vertices = {index for poly in selected for index in poly}
    context = [] if selected_ids is None else [
        poly for poly, fid in records
        if fid not in selected_ids and any(index in selected_vertices
                                           for index in poly)
    ]

    selected_points = [verts[index] for poly in selected for index in poly]
    lo = Vector((min(p.x for p in selected_points),
                 min(p.y for p in selected_points),
                 min(p.z for p in selected_points)))
    hi = Vector((max(p.x for p in selected_points),
                 max(p.y for p in selected_points),
                 max(p.z for p in selected_points)))
    size = hi - lo
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    selected_mat = bpy.data.materials.new("selected")
    selected_mat.diffuse_color = (0.95, 0.48, 0.06, 1.0)
    selected_obj = mesh_object("selected", verts, selected, selected_mat)
    if context:
        context_mat = bpy.data.materials.new("context")
        context_mat.diffuse_color = (0.25, 0.28, 0.34, 1.0)
        mesh_object("context", verts, context, context_mat)

    center = (lo + hi) * 0.5
    extent = max(size.x, size.y, size.z, 1.0)
    if len(args) <= 4 or args[4].lower() != "solid":
        wire_mat = bpy.data.materials.new("wire")
        wire_mat.diffuse_color = (0.015, 0.015, 0.02, 1.0)
        edge_object("topology", verts, selected, wire_mat,
                    max(extent * (0.00016 if selected_ids is None else
                                  0.0008), 0.001))
        if context:
            context_wire_mat = bpy.data.materials.new("context-wire")
            context_wire_mat.diffuse_color = (0.055, 0.065, 0.08, 1.0)
            edge_object("context-topology", verts, context,
                        context_wire_mat, max(extent * 0.00045, 0.0008))
    direction = Vector(tuple(float(v) for v in args[3].split(","))).normalized() \
        if len(args) > 3 else Vector((1.0, -1.0, 0.72)).normalized()
    camera_data = bpy.data.cameras.new("camera")
    camera = bpy.data.objects.new("camera", camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = center + direction * extent * 3.0
    camera.rotation_euler = (-direction).to_track_quat("-Z", "Y").to_euler()
    camera_data.type = "ORTHO"
    camera_data.clip_start = max(extent * 1e-5, 0.001)
    camera_data.clip_end = extent * 10.0
    world_up = Vector((0.0, 0.0, 1.0))
    right = direction.cross(world_up)
    if right.length < 1e-6:
        right = Vector((1.0, 0.0, 0.0))
    right.normalize()
    up = right.cross(direction).normalized()
    projected_x = [p.dot(right) for p in selected_points]
    projected_y = [p.dot(up) for p in selected_points]
    span_x = max(projected_x) - min(projected_x)
    span_y = max(projected_y) - min(projected_y)
    camera_data.ortho_scale = max(span_y * 1.12, span_x * 900.0 / 1400.0 * 1.12,
                                  1.0)
    bpy.context.scene.camera = camera

    light_data = bpy.data.lights.new("key", "AREA")
    light_data.energy = 1300.0
    light_data.shape = "DISK"
    light_data.size = extent * 2.0
    light = bpy.data.objects.new("key", light_data)
    bpy.context.collection.objects.link(light)
    light.location = center + Vector((extent, -extent, extent * 2.0))

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_shadows = True
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = "WORLD"
    scene.display.shading.background_type = "WORLD"
    scene.render.resolution_x = 1400 if selected_ids is None else 900
    scene.render.resolution_y = 900 if selected_ids is None else 700
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = output
    scene.world.color = (0.025, 0.025, 0.035)
    scene.view_settings.look = "AgX - Medium High Contrast"
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    main()
