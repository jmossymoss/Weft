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
    obj.show_wire = True
    obj.show_all_edges = True
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

    center = (lo + hi) * 0.5
    extent = max(size.x, size.y, size.z, 1.0)
    if len(args) <= 4 or args[4].lower() != "solid":
        wire_mat = bpy.data.materials.new("wire")
        wire_mat.diffuse_color = (0.015, 0.015, 0.02, 1.0)
        wire = selected_obj.copy()
        wire.data = selected_obj.data.copy()
        wire.name = "topology"
        bpy.context.collection.objects.link(wire)
        wire.data.materials.clear()
        wire.data.materials.append(wire_mat)
        modifier = wire.modifiers.new("topology", "WIREFRAME")
        modifier.thickness = max(extent * (0.00055 if selected_ids is None else
                                           0.0025), 0.003)
        modifier.use_replace = True
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
    scene.render.engine = ("BLENDER_WORKBENCH" if selected_ids is None else
                           "BLENDER_EEVEE_NEXT")
    if selected_ids is None:
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
