"""Open an FBX in Blender GUI (import_scene.fbx; CLI file open often fails on Flatpak)."""
import sys

import bpy


def _fbx_path() -> str:
    if "--" in sys.argv:
        rest = sys.argv[sys.argv.index("--") + 1 :]
        if rest:
            return rest[0]
    raise SystemExit("Usage: blender --python blender_open_fbx.py -- <file.fbx>")


def _frame_armature(arm_obj: bpy.types.Object) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    arm_obj.select_set(True)
    bpy.context.view_layer.objects.active = arm_obj

    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type != "VIEW_3D":
                continue
            region = next((r for r in area.regions if r.type == "WINDOW"), None)
            if region is None:
                continue
            space = area.spaces.active
            space.shading.type = "SOLID"
            with bpy.context.temp_override(window=window, screen=screen, area=area, region=region):
                bpy.ops.view3d.view_selected()


def _configure_armature(arm_obj: bpy.types.Object) -> None:
    arm_obj.data.display_type = "OCTAHEDRAL"
    arm_obj.show_in_front = True
    if arm_obj.animation_data and arm_obj.animation_data.action:
        action = arm_obj.animation_data.action
        start, end = [int(f) for f in action.frame_range]
        bpy.context.scene.frame_start = start
        bpy.context.scene.frame_end = end
        bpy.context.scene.frame_current = start
    # Add a floor reference so depth is obvious in an empty scene.
    bpy.ops.mesh.primitive_plane_add(size=4.0, location=(0.0, 0.0, 0.0))
    floor = bpy.context.active_object
    floor.name = "ground_ref"
    floor.display_type = "WIRE"


def main() -> None:
    path = _fbx_path()
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=path)

    arm_obj = next((o for o in bpy.data.objects if o.type == "ARMATURE"), None)
    if arm_obj is None:
        raise RuntimeError(f"No armature found in {path}")

    _configure_armature(arm_obj)
    _frame_armature(arm_obj)
    print(f"Imported: {path} ({len(arm_obj.data.bones)} bones)")


if __name__ == "__main__":
    main()
