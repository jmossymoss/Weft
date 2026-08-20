"""Weft live link: watch Weft's live OBJ export and mirror it into the scene.

Install this file as a Blender addon (Edit > Preferences > Add-ons >
Install...), then open the "Weft" tab in the 3D viewport sidebar (N) and
enable watching. In Weft, tick "live link (Blender)" — every edit you make
re-exports the mesh, and this addon swaps it into the "Weft" object in
place, so materials, transforms, and modifiers on the object survive.

CAD face identity survives too: each polygon carries its source B-rep face
id in the integer face attribute "weft_face" (usable in geometry nodes,
attribute-driven materials, or python selections).
"""

bl_info = {
    "name": "Weft Live Link",
    "author": "Weft",
    "version": (0, 1, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Weft",
    "description": "Auto-reimport Weft's live OBJ export, keeping CAD face "
                   "ids as the 'weft_face' face attribute",
    "category": "Import-Export",
}

import os

import bpy


def default_live_path():
    """Match the app's per-user data dir (userDataDir in app/main.cpp)."""
    if os.name == "nt":
        base = os.environ.get("LOCALAPPDATA", ".")
        return os.path.join(base, "Weft", "weft_live.obj")
    xdg = os.environ.get("XDG_STATE_HOME")
    base = xdg if xdg else os.path.join(os.path.expanduser("~"),
                                        ".local", "state")
    return os.path.join(base, "weft", "weft_live.obj")


def parse_obj(path):
    """Weft's exporter: plain v/f lines, 'g face_N' groups, and one
    'o object_N' block per CAD body (so bodies stay separate objects).
    Returns a list of (name, verts, faces, face_ids) per object, with
    vertex indices remapped to each object's own vertex list."""
    all_verts = []
    objects = []  # (name, [(global face tuple, gid), ...])
    current = ["Weft", []]
    gid = 0
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                all_verts.append((float(parts[1]), float(parts[2]),
                                  float(parts[3])))
            elif line.startswith("f "):
                idx = tuple(int(tok.split("/")[0]) - 1
                            for tok in line.split()[1:])
                current[1].append((idx, gid))
            elif line.startswith("g "):
                name = line.split(None, 1)[1].strip()
                tail = name.rsplit("_", 1)[-1]
                gid = int(tail) if tail.isdigit() else 0
            elif line.startswith("o "):
                if current[1]:
                    objects.append(current)
                current = ["Weft " + line.split(None, 1)[1].strip(), []]
    if current[1]:
        objects.append(current)

    result = []
    for name, polys in objects:
        remap = {}
        verts, faces, face_ids = [], [], []
        for idx, g in polys:
            mapped = []
            for i in idx:
                # A malformed/truncated OBJ can name vertices that do not
                # exist; skip the polygon instead of wrapping onto whatever
                # negative index addresses.
                if i < 0 or i >= len(all_verts):
                    mapped = []
                    break
                if i not in remap:
                    remap[i] = len(verts)
                    verts.append(all_verts[i])
                mapped.append(remap[i])
            if len(mapped) < 3:
                continue
            faces.append(tuple(mapped))
            face_ids.append(g)
        result.append((name, verts, faces, face_ids))
    return result


def apply_object(name, verts, faces, face_ids):
    mesh = bpy.data.meshes.new(name + " Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    attr = mesh.attributes.new("weft_face", "INT", "FACE")
    attr.data.foreach_set("value", face_ids)

    obj = bpy.data.objects.get(name)
    if obj is None or obj.type != "MESH":
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.scene.collection.objects.link(obj)
        return obj
    old = obj.data
    for mat in old.materials:  # keep material slots across swaps
        mesh.materials.append(mat)
    obj.data = mesh
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


def apply_objects(parsed):
    seen = set()
    for name, verts, faces, face_ids in parsed:
        apply_object(name, verts, faces, face_ids)
        seen.add(name)
    # Bodies deleted upstream disappear here too (only 'Weft ...' objects
    # this addon created are considered).
    for obj in list(bpy.data.objects):
        if obj.type != "MESH" or obj.name in seen:
            continue
        if obj.name == "Weft" or obj.name.startswith("Weft object_"):
            data = obj.data
            bpy.data.objects.remove(obj)
            if data is not None and data.users == 0:
                bpy.data.meshes.remove(data)


_state = {"mtime": 0.0}


def _poll():
    props = bpy.context.scene.weft_link
    if not props.enabled:
        return None  # unregisters the timer
    path = bpy.path.abspath(props.path)
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return props.interval
    if mtime != _state["mtime"]:
        for obj in bpy.data.objects:
            if obj.type == "MESH" and obj.name.startswith("Weft") and \
                    obj.mode != "OBJECT":
                return props.interval  # don't swap under edit mode
        try:
            # Weft renames the finished file into place, so a new mtime is
            # always a complete export.
            apply_objects(parse_obj(path))
            _state["mtime"] = mtime
        except Exception as exc:  # keep the timer alive on a bad read
            print("weft live link:", exc)
    return props.interval


class WeftLinkProps(bpy.types.PropertyGroup):
    path: bpy.props.StringProperty(
        name="Live OBJ",
        subtype="FILE_PATH",
        default=default_live_path(),
        description="The file Weft's 'live link (Blender)' toggle writes",
    )
    interval: bpy.props.FloatProperty(
        name="Poll (s)", default=0.5, min=0.1, max=5.0,
        description="How often to check the file for changes",
    )
    enabled: bpy.props.BoolProperty(name="Watching", default=False)


class WEFT_OT_toggle_watch(bpy.types.Operator):
    bl_idname = "weft.toggle_watch"
    bl_label = "Toggle Weft Watching"
    bl_description = "Start/stop watching the live OBJ for changes"

    def execute(self, context):
        props = context.scene.weft_link
        props.enabled = not props.enabled
        if props.enabled:
            _state["mtime"] = 0.0  # force an immediate import
            if not bpy.app.timers.is_registered(_poll):
                bpy.app.timers.register(_poll, first_interval=0.1)
        return {"FINISHED"}


class WEFT_PT_panel(bpy.types.Panel):
    bl_label = "Weft Live Link"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Weft"

    def draw(self, context):
        props = context.scene.weft_link
        col = self.layout.column()
        col.prop(props, "path")
        col.prop(props, "interval")
        col.operator("weft.toggle_watch",
                     text="Stop watching" if props.enabled else
                          "Start watching",
                     icon="PAUSE" if props.enabled else "PLAY")
        if props.enabled:
            col.label(text="Mirroring into object 'Weft'", icon="CHECKMARK")


classes = (WeftLinkProps, WEFT_OT_toggle_watch, WEFT_PT_panel)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.weft_link = bpy.props.PointerProperty(type=WeftLinkProps)


def unregister():
    if bpy.app.timers.is_registered(_poll):
        bpy.app.timers.unregister(_poll)
    del bpy.types.Scene.weft_link
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
