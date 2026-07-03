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
    """Weft's exporter writes plain v/f lines with 'g face_N' groups."""
    verts, faces, face_ids = [], [], []
    gid = 0
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                verts.append((float(parts[1]), float(parts[2]),
                              float(parts[3])))
            elif line.startswith("f "):
                idx = tuple(int(tok.split("/")[0]) - 1
                            for tok in line.split()[1:])
                faces.append(idx)
                face_ids.append(gid)
            elif line.startswith("g "):
                name = line.split(None, 1)[1].strip()
                tail = name.rsplit("_", 1)[-1]
                gid = int(tail) if tail.isdigit() else 0
    return verts, faces, face_ids


def apply_mesh(verts, faces, face_ids):
    mesh = bpy.data.meshes.new("WeftMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    attr = mesh.attributes.new("weft_face", "INT", "FACE")
    attr.data.foreach_set("value", face_ids)

    obj = bpy.data.objects.get("Weft")
    if obj is None or obj.type != "MESH":
        obj = bpy.data.objects.new("Weft", mesh)
        bpy.context.scene.collection.objects.link(obj)
        return obj
    old = obj.data
    for mat in old.materials:  # keep material slots across swaps
        mesh.materials.append(mat)
    obj.data = mesh
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


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
        obj = bpy.data.objects.get("Weft")
        if obj is not None and obj.mode != "OBJECT":
            return props.interval  # don't swap under an edit-mode session
        try:
            # Weft renames the finished file into place, so a new mtime is
            # always a complete export.
            verts, faces, face_ids = parse_obj(path)
            apply_mesh(verts, faces, face_ids)
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
