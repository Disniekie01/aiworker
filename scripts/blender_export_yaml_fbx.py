#!/usr/bin/env python3
"""Blender script: URDF robot + YAML animation JSON -> FBX.

Run (do not execute with python3 directly):
  blender --background --python blender_export_yaml_fbx.py -- \\
    --animation /path/to/pick_place_animation.json \\
    --urdf /path/to/ffw_bg2_follower_nomesh.urdf \\
    --output /path/to/pick_place.fbx
"""

from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Euler, Matrix, Vector


def _parse_args() -> dict[str, str]:
    if "--" in sys.argv:
        argv = sys.argv[sys.argv.index("--") + 1 :]
    else:
        argv = []
    out: dict[str, str] = {}
    i = 0
    while i < len(argv):
        if argv[i].startswith("--"):
            key = argv[i][2:].replace("-", "_")
            if i + 1 < len(argv) and not argv[i + 1].startswith("--"):
                out[key] = argv[i + 1]
                i += 2
                continue
        i += 1
    return out


def _parse_xyz(text: str) -> Vector:
    parts = [float(x) for x in text.split()]
    return Vector((parts[0], parts[1], parts[2]))


def _parse_rpy(text: str) -> Euler:
    parts = [float(x) for x in text.split()]
    return Euler((parts[0], parts[1], parts[2]), "XYZ")


def _load_urdf(path: Path) -> tuple[dict, dict]:
    root = ET.parse(path).getroot()
    joints: dict[str, dict] = {}
    links: set[str] = set()
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        jtype = joint.attrib.get("type", "fixed")
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz = _parse_xyz(origin.attrib.get("xyz", "0 0 0")) if origin is not None else Vector()
        rpy = _parse_rpy(origin.attrib.get("rpy", "0 0 0")) if origin is not None else Euler()
        axis_el = joint.find("axis")
        axis = _parse_xyz(axis_el.attrib.get("xyz", "0 0 1")) if axis_el is not None else Vector((0, 0, 1))
        if axis.length > 1e-9:
            axis.normalize()
        joints[name] = {
            "type": jtype,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
        }
        links.add(parent)
        links.add(child)
    return joints, {ln: {} for ln in links}


def _build_armature(joints: dict, anim_joint_names: set[str]) -> tuple[bpy.types.Object, dict[str, str]]:
    """Create armature with bones for animated URDF joints + base root."""
    arm_data = bpy.data.armatures.new("FFW_Armature")
    arm_obj = bpy.data.objects.new("FFW_Robot", arm_data)
    bpy.context.collection.objects.link(arm_obj)
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="EDIT")

    edit_bones = arm_data.edit_bones
    base_bone = edit_bones.new("base_root")
    base_bone.head = Vector((0.0, 0.0, 0.0))
    base_bone.tail = Vector((0.0, 0.0, 0.1))

    link_bone: dict[str, bpy.types.EditBone] = {"base_link": base_bone}
    bone_for_joint: dict[str, str] = {}

    # Map parent link -> list of joint names
    children: dict[str, list[str]] = {}
    for jname, j in joints.items():
        children.setdefault(j["parent"], []).append(jname)

    queue = ["base_link"]
    visited_joints: set[str] = set()
    while queue:
        link = queue.pop(0)
        parent_bone = link_bone.get(link)
        if parent_bone is None:
            continue
        for jname in children.get(link, []):
            if jname in visited_joints:
                continue
            visited_joints.add(jname)
            j = joints[jname]
            child_link = j["child"]
            bone_name = jname
            if bone_name in edit_bones:
                bone_name = f"{jname}__{child_link}"

            bone = edit_bones.new(bone_name)
            local = Matrix.Translation(j["xyz"]) @ j["rpy"].to_matrix().to_4x4()
            world = parent_bone.matrix @ local
            bone.head = world.to_translation()
            tail_local = Vector((0.0, 0.05, 0.0))
            if j["type"] == "prismatic":
                tail_local = Vector(j["axis"]) * 0.08
            bone.tail = (world @ Matrix.Translation(tail_local)).to_translation()
            if (bone.tail - bone.head).length < 1e-4:
                bone.tail = bone.head + Vector((0.0, 0.05, 0.0))
            bone.parent = parent_bone
            bone.use_connect = False

            if j["type"] in ("revolute", "prismatic"):
                bone_for_joint[jname] = bone_name
            link_bone[child_link] = bone
            queue.append(child_link)

    bpy.ops.object.mode_set(mode="OBJECT")
    return arm_obj, bone_for_joint


def _apply_frame(
    arm_obj: bpy.types.Object,
    joints: dict,
    bone_for_joint: dict[str, str],
    frame: dict,
    fps: float,
    frame_idx: int,
) -> None:
    scene = bpy.context.scene
    scene.frame_set(frame_idx)
    t = frame_idx / fps

    arm_obj.location = (
        float(frame.get("isaac_base_x", 0.0)),
        float(frame.get("isaac_base_y", 0.0)),
        float(frame.get("isaac_base_z", 0.0)),
    )
    arm_obj.rotation_euler = (0.0, 0.0, float(frame.get("isaac_base_yaw", 0.0)))
    arm_obj.keyframe_insert(data_path="location", frame=frame_idx)
    arm_obj.keyframe_insert(data_path="rotation_euler", frame=frame_idx)

    pose_bones = arm_obj.pose.bones
    for jname, value in frame.items():
        if jname not in bone_for_joint:
            continue
        bone = pose_bones.get(bone_for_joint[jname])
        if bone is None:
            continue
        j = joints.get(jname)
        if j is None:
            continue
        jtype = j["type"]
        axis = Vector(j["axis"])
        if jtype == "revolute":
            bone.rotation_mode = "AXIS_ANGLE"
            bone.rotation_axis_angle = (float(value), axis.x, axis.y, axis.z)
            bone.keyframe_insert(data_path="rotation_axis_angle", frame=frame_idx)
        elif jtype == "prismatic":
            bone.location = axis * float(value)
            bone.keyframe_insert(data_path="location", frame=frame_idx)


def _add_stick_meshes(arm_obj: bpy.types.Object) -> None:
    """Visible proxy geometry so the skeleton is easy to see in the viewport."""
    mat = bpy.data.materials.new(name="FFW_Stick")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (0.2, 0.65, 1.0, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 0.4

    bpy.context.view_layer.objects.active = arm_obj
    for bone in arm_obj.data.bones:
        length = (bone.tail_local - bone.head_local).length
        if length < 1e-5:
            continue
        radius = max(0.012, min(0.03, length * 0.08))
        bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=length)
        cyl = bpy.context.active_object
        cyl.name = f"stick_{bone.name}"
        cyl.data.materials.append(mat)
        cyl.parent = arm_obj
        cyl.parent_type = "BONE"
        cyl.parent_bone = bone.name
        cyl.location = (0.0, 0.0, length * 0.5)


def _export_fbx(path: Path) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in bpy.context.scene.objects:
        if obj.type in {"ARMATURE", "MESH"}:
            obj.select_set(True)
    bpy.context.view_layer.objects.active = next(
        o for o in bpy.context.scene.objects if o.type == "ARMATURE"
    )
    bpy.ops.export_scene.fbx(
        filepath=str(path),
        use_selection=True,
        object_types={"ARMATURE", "MESH"},
        bake_anim=True,
        bake_anim_use_all_actions=False,
        bake_anim_step=1.0,
        bake_anim_simplify_factor=0.0,
        add_leaf_bones=False,
        apply_scale_options="FBX_SCALE_ALL",
    )


def main() -> None:
    args = _parse_args()
    anim_path = Path(args.get("animation", "")).expanduser()
    urdf_path = Path(args.get("urdf", "")).expanduser()
    out_path = Path(args.get("output", "")).expanduser()
    if not anim_path.is_file():
        raise FileNotFoundError(f"Animation JSON not found: {anim_path}")
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    if not out_path:
        raise ValueError("--output FBX path required")

    anim = json.loads(anim_path.read_text(encoding="utf-8"))
    fps = float(anim.get("fps", 30.0))
    frames = anim["frames"]
    joint_names = set(anim.get("joint_names", []))

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.fps = int(round(fps))
    scene.frame_start = 0
    scene.frame_end = max(0, len(frames) - 1)

    joints, _ = _load_urdf(urdf_path)
    anim_joints = {n for n in joint_names if n in joints}
    arm_obj, bone_for_joint = _build_armature(joints, anim_joints)

    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="POSE")
    for i, frame in enumerate(frames):
        _apply_frame(arm_obj, joints, bone_for_joint, frame, fps, i)
    bpy.ops.object.mode_set(mode="OBJECT")

    _add_stick_meshes(arm_obj)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    _export_fbx(out_path)
    print(f"Exported FBX: {out_path} ({len(frames)} frames @ {fps} Hz)")


if __name__ == "__main__":
    main()
