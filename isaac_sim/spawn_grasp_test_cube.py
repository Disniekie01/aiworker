# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Spawn a flat rigid cube for gripper weld testing (replaces shirt pile)."""

from __future__ import annotations

from pxr import Gf, Usd, UsdGeom

from configure_scene_physics import GRASP_TEST_CUBE_ROOT, SHIRT_ROOTS

GRASP_TEST_CUBE_MESH = f"{GRASP_TEST_CUBE_ROOT}/Cube"
GRASP_TEST_CUBE_GRIP = f"{GRASP_TEST_CUBE_ROOT}/Grippoint"

# Flat slab ~shirt footprint in the crate (meters).
DEFAULT_CUBE_SIZE = Gf.Vec3f(0.18, 0.12, 0.012)
DEFAULT_CUBE_COLOR = Gf.Vec3f(0.85, 0.35, 0.15)
# User-tuned rest pose in the crate (root center; mesh is centered on origin).
DEFAULT_CUBE_TRANSLATE = Gf.Vec3d(
    0.12917177379131317,
    -0.6569662094116211,
    1.0815482330322266,
)


def _define_box_mesh(stage: Usd.Stage, mesh_path: str, size: Gf.Vec3f) -> UsdGeom.Mesh:
    """Box mesh centered at origin (no xformOp:scale on the prim)."""
    hx, hy, hz = float(size[0]) * 0.5, float(size[1]) * 0.5, float(size[2]) * 0.5
    points = [
        Gf.Vec3f(-hx, -hy, -hz),
        Gf.Vec3f(hx, -hy, -hz),
        Gf.Vec3f(hx, hy, -hz),
        Gf.Vec3f(-hx, hy, -hz),
        Gf.Vec3f(-hx, -hy, hz),
        Gf.Vec3f(hx, -hy, hz),
        Gf.Vec3f(hx, hy, hz),
        Gf.Vec3f(-hx, hy, hz),
    ]
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr(
        [0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
    )
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.GetDisplayColorAttr().Set([DEFAULT_CUBE_COLOR])
    return mesh


def _default_cube_pose(_stage: Usd.Stage, _half_height: float) -> Gf.Vec3d:
    return Gf.Vec3d(DEFAULT_CUBE_TRANSLATE)


def _reset_cube_mesh_local_xform(stage: Usd.Stage) -> None:
    mesh_prim = stage.GetPrimAtPath(GRASP_TEST_CUBE_MESH)
    if not mesh_prim or not mesh_prim.IsValid():
        return
    UsdGeom.XformCommonAPI(mesh_prim).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    UsdGeom.XformCommonAPI(mesh_prim).SetRotate(Gf.Vec3f(0.0, 0.0, 0.0))
    UsdGeom.XformCommonAPI(mesh_prim).SetScale(Gf.Vec3f(1.0, 1.0, 1.0))


def _set_cube_root_translate(stage: Usd.Stage, pos: Gf.Vec3d) -> None:
    root_prim = stage.GetPrimAtPath(GRASP_TEST_CUBE_ROOT)
    if not root_prim or not root_prim.IsValid():
        return
    UsdGeom.XformCommonAPI(root_prim).SetTranslate(pos)


def hide_shirt_piles(stage: Usd.Stage, roots: tuple[str, ...] = SHIRT_ROOTS) -> int:
    hidden = 0
    for root in roots:
        prim = stage.GetPrimAtPath(root)
        if prim and prim.IsValid() and prim.IsActive():
            prim.SetActive(False)
            hidden += 1
            print(f"[grasp_test_cube] Hidden shirt pile: {root}")
    return hidden


def spawn_grasp_test_cube(
    stage: Usd.Stage,
    translate: Gf.Vec3d | None = None,
    size: Gf.Vec3f = DEFAULT_CUBE_SIZE,
    hide_shirts: bool = True,
) -> str:
    """Create /World/GraspTestCube with mesh + Grippoint; return mesh path."""
    if hide_shirts:
        hide_shirt_piles(stage)

    root_prim = stage.GetPrimAtPath(GRASP_TEST_CUBE_ROOT)
    pos = translate if translate is not None else _default_cube_pose(stage, float(size[2]) * 0.5)
    if root_prim and root_prim.IsValid():
        _set_cube_root_translate(stage, pos)
        _reset_cube_mesh_local_xform(stage)
        print(
            f"[grasp_test_cube] Repositioned existing cube to "
            f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
        )
        return GRASP_TEST_CUBE_MESH

    root = UsdGeom.Xform.Define(stage, GRASP_TEST_CUBE_ROOT)
    UsdGeom.XformCommonAPI(root).SetTranslate(pos)

    cube = _define_box_mesh(stage, GRASP_TEST_CUBE_MESH, size)
    # Keep mesh at local origin — YAML shirt poses must not offset a physics collider child.
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    UsdGeom.XformCommonAPI(cube).SetRotate(Gf.Vec3f(0.0, 0.0, 0.0))
    UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(1.0, 1.0, 1.0))

    grip = UsdGeom.Xform.Define(stage, GRASP_TEST_CUBE_GRIP)
    UsdGeom.XformCommonAPI(grip).SetTranslate(Gf.Vec3d(0.0, 0.0, float(size[2]) * 0.5))

    print(
        f"[grasp_test_cube] Spawned flat cube at ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
        f"size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) m "
        f"(physics applied in configure_scene_physics)"
    )
    print(f"[grasp_test_cube] Mesh={GRASP_TEST_CUBE_MESH} Grippoint={GRASP_TEST_CUBE_GRIP}")
    return GRASP_TEST_CUBE_MESH
