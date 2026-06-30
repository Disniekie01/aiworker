# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Configure crate (kinematic) and t-shirt meshes (rigid or deformable) in Isaac."""

from __future__ import annotations

from typing import Iterable, List

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


CRATE_PRIM = "/World/KB3D_CTS_Crate_A"
CRATE_COLLISION_ROOT = "/World/CrateInteriorCollision"
CRATE_INTERIOR_INSET_M = 0.008
# User-tuned floor pose in world space (Scene_clean / Isaac viewport).
CRATE_FLOOR_TRANSLATE = Gf.Vec3d(
    0.15968298966507785,
    -0.6910173958402114,
    1.030548253804919,
)
CRATE_FLOOR_SCALE = Gf.Vec3f(-0.3034169, 0.19236615, 0.01)
# PhysX rejects negative/thin scales — collision uses positive extents (min 2.5 cm thick).
CRATE_FLOOR_COLLISION_MIN_Z = 0.025
SHIRT_ABOVE_FLOOR_M = 0.012
# Physics grasp: small kinematic rest pad (shirt sits on top; grippers reach under fold).
PHYSICS_GRASP_REST_PAD_TRANSLATE = Gf.Vec3d(
    0.15867,
    -0.69102,
    1.04679,
)
PHYSICS_GRASP_REST_PAD_SCALE = Gf.Vec3f(0.05, 0.05, 0.025)
# PhysX contact tuning for friction grasp (shirt + gripper fingers).
GRASP_PHYSICS_MATERIAL_PATH = "/World/PhysicsMaterials/GraspHighFriction"
GRASP_STATIC_FRICTION = 2.0
GRASP_DYNAMIC_FRICTION = 2.0
GRASP_RESTITUTION = 0.0
PHYSICS_GRASP_SHIRT_MASS_KG = 0.10
GRIPPER_GRASP_SUBTREES = (
    "/World/Robot/ffw_sg2_follower/right_gripper",
)
SHIRT_ROOTS = (
    "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01",
)
GRASP_TEST_CUBE_ROOT = "/World/GraspTestCube"
ROBOT_COLLISION_FILTER_ROOTS = (
    "/World/Robot/ffw_sg2_follower",
    "/World/Robot",
)
TSHIRT_MATERIAL_PATH = "/World/PhysicsMaterials/TShirtDeformable"

# Soft folded-fabric feel (lower Young's modulus = drapier).
TSHIRT_YOUNGS_MODULUS = 400.0
TSHIRT_POISSONS_RATIO = 0.49
TSHIRT_DENSITY = 45.0
TSHIRT_ELASTICITY_DAMPING = 0.15
TSHIRT_DAMPING_SCALE = 1.0
TSHIRT_DYNAMIC_FRICTION = 0.75
TSHIRT_VERTEX_VELOCITY_DAMPING = 0.45
TSHIRT_SOLVER_POSITION_ITERATIONS = 32


def _find_active_mesh_paths(stage: Usd.Stage, root_path: str) -> List[str]:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return []
    paths: List[str] = []
    for prim in Usd.PrimRange(root):
        if not prim.IsActive():
            continue
        if prim.GetTypeName() == "Mesh":
            paths.append(prim.GetPath().pathString)
    return paths


def _remove_rigid_body(prim: Usd.Prim) -> None:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)


def _disable_descendant_mesh_collisions(root: Usd.Prim) -> int:
    count = 0
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        count += 1
    return count


def _world_to_local(parent: Usd.Prim, world_pt: Gf.Vec3d) -> Gf.Vec3d:
    xform = UsdGeom.Xformable(parent)
    parent_world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return parent_world.GetInverse().Transform(world_pt)


def _world_translate(prim: Usd.Prim) -> Gf.Vec3d:
    xform = UsdGeom.Xformable(prim)
    return Gf.Vec3d(xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())


def _prim_mesh_world_bounds(root: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Vec3d] | None:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    mn: Gf.Vec3d | None = None
    mx: Gf.Vec3d | None = None
    for prim in Usd.PrimRange(root):
        if not prim.IsActive():
            continue
        if prim.GetTypeName() != "Mesh":
            continue
        bound = cache.ComputeWorldBound(prim)
        if bound.GetRange().IsEmpty():
            continue
        bmn = Gf.Vec3d(bound.GetRange().GetMin())
        bmx = Gf.Vec3d(bound.GetRange().GetMax())
        if mn is None:
            mn, mx = bmn, bmx
        else:
            mn = Gf.Vec3d(min(mn[i], bmn[i]) for i in range(3))
            mx = Gf.Vec3d(max(mx[i], bmx[i]) for i in range(3))
    if mn is None or mx is None:
        return None
    return mn, mx


def _nudge_prim_world_z(prim: Usd.Prim, delta_world_z: float) -> None:
    if abs(delta_world_z) < 1e-6:
        return
    xformable = UsdGeom.Xformable(prim)
    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        local_delta = parent_world.GetInverse().Transform(Gf.Vec3d(0.0, 0.0, delta_world_z))
    else:
        local_delta = Gf.Vec3d(0.0, 0.0, delta_world_z)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
            continue
        cur = op.Get()
        op.Set(
            Gf.Vec3d(
                float(cur[0]) + local_delta[0],
                float(cur[1]) + local_delta[1],
                float(cur[2]) + local_delta[2],
            )
        )
        return


def _add_box_collider(
    stage: Usd.Stage,
    path: str,
    center_world: Gf.Vec3d,
    half_extents: Gf.Vec3d,
    parent: Usd.Prim,
    *,
    visible: bool = False,
    high_friction: bool = False,
    world_coords: bool = False,
) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(2.0)
    prim = cube.GetPrim()
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    if world_coords or parent.GetPath() == Sdf.Path("/World"):
        xformable.AddTranslateOp().Set(center_world)
    else:
        xformable.AddTranslateOp().Set(_world_to_local(parent, center_world))
    xformable.AddScaleOp().Set(Gf.Vec3f(half_extents))
    if visible:
        cube.CreateDisplayColorAttr([(Gf.Vec3f(0.42, 0.39, 0.34))])
        cube.CreatePurposeAttr(UsdGeom.Tokens.default_)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True)
    if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx_col = PhysxSchema.PhysxCollisionAPI(prim)
    physx_col.CreateContactOffsetAttr(0.01 if high_friction else 0.002)
    physx_col.CreateRestOffsetAttr(0.0)
    if high_friction:
        if not prim.HasAPI(UsdPhysics.MaterialAPI):
            UsdPhysics.MaterialAPI.Apply(prim)
        usd_mat = UsdPhysics.MaterialAPI(prim)
        usd_mat.CreateStaticFrictionAttr(0.95)
        usd_mat.CreateDynamicFrictionAttr(0.9)


def _crate_floor_collision_scale() -> Gf.Vec3f:
    return Gf.Vec3f(
        abs(float(CRATE_FLOOR_SCALE[0])),
        abs(float(CRATE_FLOOR_SCALE[1])),
        max(abs(float(CRATE_FLOOR_SCALE[2])), CRATE_FLOOR_COLLISION_MIN_Z),
    )


def _shirt_rest_pad_pose(*, physics_grasp: bool = False) -> tuple[Gf.Vec3d, Gf.Vec3f]:
    if physics_grasp:
        return PHYSICS_GRASP_REST_PAD_TRANSLATE, PHYSICS_GRASP_REST_PAD_SCALE
    return CRATE_FLOOR_TRANSLATE, _crate_floor_collision_scale()


def _rest_pad_top_z(translate: Gf.Vec3d, scale: Gf.Vec3f) -> float:
    # UsdGeom.Cube size=2 → half-extent 1; scale Z is world half-height.
    return float(translate[2]) + float(scale[2])


def _crate_floor_top_z(*, physics_grasp: bool = False) -> float:
    translate, scale = _shirt_rest_pad_pose(physics_grasp=physics_grasp)
    return _rest_pad_top_z(translate, scale)


def _add_shirt_rest_floor(
    stage: Usd.Stage,
    path: str,
    translate: Gf.Vec3d,
    scale: Gf.Vec3f,
) -> None:
    """Shirt rest pad at tuned translate; PhysX-safe positive collision scale."""
    col_scale = Gf.Vec3f(abs(scale[0]), abs(scale[1]), max(abs(scale[2]), CRATE_FLOOR_COLLISION_MIN_Z))
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(2.0)
    prim = cube.GetPrim()
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(translate)
    xformable.AddScaleOp().Set(col_scale)
    cube.CreateDisplayColorAttr([(Gf.Vec3f(0.42, 0.39, 0.34))])
    cube.CreatePurposeAttr(UsdGeom.Tokens.default_)

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb = UsdPhysics.RigidBodyAPI(prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(True)

    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True)
    if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        UsdPhysics.MeshCollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr("boundingCube")

    if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx_col = PhysxSchema.PhysxCollisionAPI(prim)
    physx_col.CreateContactOffsetAttr(0.015)
    physx_col.CreateRestOffsetAttr(0.0)

    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(prim)
    usd_mat = UsdPhysics.MaterialAPI(prim)
    usd_mat.CreateStaticFrictionAttr(0.95)
    usd_mat.CreateDynamicFrictionAttr(0.9)


def _crate_mesh_world_bounds(crate_prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Vec3d] | None:
    """Bounds from crate visual meshes only (ignore skel payloads and our collider cubes)."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    mn: Gf.Vec3d | None = None
    mx: Gf.Vec3d | None = None
    for prim in Usd.PrimRange(crate_prim):
        if not prim.IsActive():
            continue
        path = prim.GetPath().pathString
        if "/CollisionWalls" in path:
            continue
        if "skel" in path.lower():
            continue
        if prim.GetTypeName() != "Mesh":
            continue
        bound = cache.ComputeWorldBound(prim)
        if bound.GetRange().IsEmpty():
            continue
        bmn = Gf.Vec3d(bound.GetRange().GetMin())
        bmx = Gf.Vec3d(bound.GetRange().GetMax())
        if mn is None:
            mn, mx = bmn, bmx
        else:
            mn = Gf.Vec3d(min(mn[i], bmn[i]) for i in range(3))
            mx = Gf.Vec3d(max(mx[i], bmx[i]) for i in range(3))
    if mn is None or mx is None:
        return None
    return mn, mx


def _crate_interior_bounds(crate_prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Vec3d, float, float, float, float, float, float] | None:
    mesh_bounds = _crate_mesh_world_bounds(crate_prim)
    if mesh_bounds is None:
        return None

    mn, mx = mesh_bounds
    inset = CRATE_INTERIOR_INSET_M

    mn = Gf.Vec3d(mn[0] + inset, mn[1] + inset, mn[2])
    mx = Gf.Vec3d(mx[0] - inset, mx[1] - inset, mx[2])

    if mx[0] <= mn[0] or mx[1] <= mn[1] or mx[2] <= mn[2]:
        return None

    cx = 0.5 * (mn[0] + mx[0])
    cy = 0.5 * (mn[1] + mx[1])
    cz = 0.5 * (mn[2] + mx[2])
    sx = mx[0] - mn[0]
    sy = mx[1] - mn[1]
    sz = mx[2] - mn[2]
    return mn, mx, cx, cy, cz, sx, sy, sz


def crate_floor_top_z() -> float:
    return _crate_floor_top_z(physics_grasp=False)


def crate_object_center_z(half_height_m: float = 0.0) -> float:
    """World Z for the center of a resting object on the crate floor pad."""
    return _crate_floor_top_z(physics_grasp=False) + SHIRT_ABOVE_FLOOR_M + float(half_height_m)


def crate_spawn_xy(stage: Usd.Stage, crate_path: str = CRATE_PRIM) -> tuple[float, float]:
    crate_prim = stage.GetPrimAtPath(crate_path)
    if crate_prim and crate_prim.IsValid():
        bounds = _crate_interior_bounds(crate_prim)
        if bounds is not None:
            return float(bounds[3]), float(bounds[4])
    return 0.15, -0.65


def _ensure_crate_interior_walls(stage: Usd.Stage, crate_prim: Usd.Prim, collision_root: str) -> None:
    """Thin kinematic walls so dynamic props do not fall through crate sides."""
    bounds = _crate_interior_bounds(crate_prim)
    if bounds is None:
        print("[scene_physics] Crate interior walls skipped (no mesh bounds)")
        return

    mn, mx, cx, cy, _cz, sx, sy, sz = bounds
    parent = stage.GetPrimAtPath(collision_root)
    if not parent or not parent.IsValid():
        return

    wall_t = 0.012
    wall_h = max(float(sz) * 0.7, 0.14)
    wall_z = float(mn[2]) + wall_h * 0.5
    half_h = wall_h * 0.5
    half_tx = wall_t * 0.5
    half_sy = float(sy) * 0.5 + wall_t
    half_sx = float(sx) * 0.5 + wall_t

    walls = (
        ("wall_x_min", Gf.Vec3d(float(mn[0]) + half_tx, cy, wall_z), Gf.Vec3d(half_tx, half_sy, half_h)),
        ("wall_x_max", Gf.Vec3d(float(mx[0]) - half_tx, cy, wall_z), Gf.Vec3d(half_tx, half_sy, half_h)),
        ("wall_y_min", Gf.Vec3d(cx, float(mn[1]) + half_tx, wall_z), Gf.Vec3d(half_sx, half_tx, half_h)),
        ("wall_y_max", Gf.Vec3d(cx, float(mx[1]) - half_tx, wall_z), Gf.Vec3d(half_sx, half_tx, half_h)),
    )
    for name, center, half_extents in walls:
        path = f"{collision_root}/{name}"
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(Sdf.Path(path))
        _add_box_collider(stage, path, center, half_extents, parent, world_coords=True, high_friction=True)

    print(
        f"[scene_physics] Crate interior walls at {collision_root} "
        f"(height={wall_h:.3f}m, thickness={wall_t:.3f}m)"
    )


def _ensure_shirt_rest_bottom(
    stage: Usd.Stage,
    collision_root: str,
    *,
    physics_grasp: bool = False,
) -> bool:
    bottom_path = f"{collision_root}/shirt_rest_bottom"
    if stage.GetPrimAtPath(bottom_path).IsValid():
        stage.RemovePrim(Sdf.Path(bottom_path))

    translate, scale = _shirt_rest_pad_pose(physics_grasp=physics_grasp)
    _add_shirt_rest_floor(stage, bottom_path, translate, scale)

    legacy_floor = f"{collision_root}/floor"
    if stage.GetPrimAtPath(legacy_floor).IsValid():
        stage.RemovePrim(Sdf.Path(legacy_floor))

    floor_top_z = _rest_pad_top_z(translate, scale)
    col_scale = Gf.Vec3f(
        abs(scale[0]), abs(scale[1]), max(abs(scale[2]), CRATE_FLOOR_COLLISION_MIN_Z)
    )
    mode = "physics grasp pad" if physics_grasp else "default"
    print(
        f"[scene_physics] Shirt rest bottom ({mode}) translate="
        f"({translate[0]:.5f}, {translate[1]:.5f}, {translate[2]:.5f}) "
        f"collision_scale=({col_scale[0]:.4f}, {col_scale[1]:.4f}, {col_scale[2]:.4f}) "
        f"top z={floor_top_z:.5f} kinematic ({bottom_path})"
    )
    return True


def _configure_crate_floor_colliders(
    stage: Usd.Stage, crate_prim: Usd.Prim, *, physics_grasp: bool = False
) -> bool:
    """Disable crate mesh collision and add the tuned world-space shirt rest floor."""
    collision_root_path = Sdf.Path(CRATE_COLLISION_ROOT)
    collision_root = CRATE_COLLISION_ROOT

    if stage.GetPrimAtPath(collision_root_path).IsValid():
        stage.RemovePrim(collision_root_path)
    legacy_walls = crate_prim.GetPath().AppendChild("CollisionWalls")
    if stage.GetPrimAtPath(legacy_walls).IsValid():
        stage.RemovePrim(legacy_walls)

    disabled = _disable_descendant_mesh_collisions(crate_prim)
    UsdGeom.Xform.Define(stage, collision_root_path)

    _ensure_shirt_rest_bottom(stage, collision_root, physics_grasp=physics_grasp)
    _ensure_crate_interior_walls(stage, crate_prim, collision_root)
    print(
        f"[scene_physics] Crate floor only at {collision_root} "
        f"(disabled {disabled} crate mesh colliders)"
    )
    return True


def _set_kinematic_crate(stage: Usd.Stage, crate_path: str = CRATE_PRIM, *, physics_grasp: bool = False) -> bool:
    prim = stage.GetPrimAtPath(crate_path)
    if not prim or not prim.IsValid():
        print(f"[scene_physics] Crate prim not found: {crate_path}")
        return False

    for child in Usd.PrimRange(prim):
        if child == prim:
            continue
        _remove_rigid_body(child)

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

    rb = UsdPhysics.RigidBodyAPI(prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(True)

    physx_rb = PhysxSchema.PhysxRigidBodyAPI(prim)
    physx_rb.CreateLockedPosAxisAttr(0)
    physx_rb.CreateLockedRotAxisAttr(0)

    _configure_crate_floor_colliders(stage, prim, physics_grasp=physics_grasp)
    print(f"[scene_physics] Crate set kinematic: {crate_path}")
    return True


def _ensure_tshirt_material(stage: Usd.Stage) -> None:
    from omni.physx.scripts import deformableUtils

    deformableUtils.add_deformable_body_material(
        stage,
        TSHIRT_MATERIAL_PATH,
        youngs_modulus=TSHIRT_YOUNGS_MODULUS,
        poissons_ratio=TSHIRT_POISSONS_RATIO,
        density=TSHIRT_DENSITY,
        elasticity_damping=TSHIRT_ELASTICITY_DAMPING,
        damping_scale=TSHIRT_DAMPING_SCALE,
        dynamic_friction=TSHIRT_DYNAMIC_FRICTION,
    )


def _tune_deformable_mesh(mesh_prim: Usd.Prim) -> None:
    if not mesh_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
        return
    deformable_api = PhysxSchema.PhysxDeformableAPI(PhysxSchema.PhysxDeformableBodyAPI(mesh_prim))
    deformable_api.CreateVertexVelocityDampingAttr(TSHIRT_VERTEX_VELOCITY_DAMPING)
    deformable_api.CreateSolverPositionIterationCountAttr(TSHIRT_SOLVER_POSITION_ITERATIONS)


def _make_mesh_deformable(
    stage: Usd.Stage,
    mesh_path: str,
    simulation_hexahedral_resolution: int = 8,
) -> bool:
    from omni.physx.scripts import deformableUtils, physicsUtils

    mesh_prim = stage.GetPrimAtPath(mesh_path)
    if not mesh_prim or not mesh_prim.IsValid():
        print(f"[scene_physics] Mesh prim not found: {mesh_path}")
        return False

    if mesh_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
        print(f"[scene_physics] Shirt already deformable, retuning: {mesh_path}")
        _tune_deformable_mesh(mesh_prim)
        _ensure_tshirt_material(stage)
        physicsUtils.add_physics_material_to_prim(stage, mesh_prim, TSHIRT_MATERIAL_PATH)
        return True

    _remove_rigid_body(mesh_prim)

    ok = deformableUtils.add_physx_deformable_body(
        stage,
        Sdf.Path(mesh_path),
        collision_simplification=True,
        collision_simplification_remeshing=True,
        simulation_hexahedral_resolution=int(simulation_hexahedral_resolution),
        self_collision=True,
        vertex_velocity_damping=TSHIRT_VERTEX_VELOCITY_DAMPING,
        solver_position_iteration_count=TSHIRT_SOLVER_POSITION_ITERATIONS,
    )
    if not ok:
        print(f"[scene_physics] add_physx_deformable_body failed: {mesh_path}")
        return False

    _ensure_tshirt_material(stage)
    physicsUtils.add_physics_material_to_prim(stage, mesh_prim, TSHIRT_MATERIAL_PATH)
    _tune_deformable_mesh(mesh_prim)
    print(f"[scene_physics] Shirt deformable body: {mesh_path}")
    return True


def _shirt_inner_prim_path(shirt_root: str) -> str:
    root_name = shirt_root.rstrip("/").rsplit("/", 1)[-1]
    if "_" in root_name:
        base = root_name.rsplit("_", 1)[0]
        return f"{shirt_root.rstrip('/')}/{base}"
    return f"{shirt_root.rstrip('/')}/{root_name}"


def _make_shirt_deformable(stage: Usd.Stage, shirt_root: str, voxel_resolution: int = 8) -> bool:
    inner_path = _shirt_inner_prim_path(shirt_root)
    inner = stage.GetPrimAtPath(inner_path)
    if inner and inner.IsValid():
        _remove_rigid_body(inner)

    mesh_paths = _find_active_mesh_paths(stage, shirt_root)
    if not mesh_paths:
        print(f"[scene_physics] No active meshes under shirt root: {shirt_root}")
        return False

    ok_any = False
    for mesh_path in mesh_paths:
        ok_any = _make_mesh_deformable(stage, mesh_path, simulation_hexahedral_resolution=voxel_resolution) or ok_any
    return ok_any


def _make_shirt_rigid(stage: Usd.Stage, shirt_root: str) -> bool:
    """Convex-hull collision on shirt meshes; pose comes from USD xform until grasp weld."""
    inner_path = _shirt_inner_prim_path(shirt_root)
    inner = stage.GetPrimAtPath(inner_path)
    if inner and inner.IsValid():
        _remove_rigid_body(inner)

    mesh_paths = _find_active_mesh_paths(stage, shirt_root)
    if not mesh_paths:
        print(f"[scene_physics] No active meshes under shirt root: {shirt_root}")
        return False

    for mesh_path in mesh_paths:
        mesh_prim = stage.GetPrimAtPath(mesh_path)
        if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        UsdPhysics.CollisionAPI(mesh_prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI(mesh_prim).CreateApproximationAttr("convexHull")

    print(f"[scene_physics] Shirt collision meshes (no dynamic rigid body): {shirt_root}")
    return True


def _apply_high_friction_material(mesh_prim: Usd.Prim) -> None:
    _apply_super_grippy_contact_material(mesh_prim)


def _apply_super_grippy_contact_material(prim: Usd.Prim) -> None:
    """Max friction, zero bounce — for shirt mesh and gripper finger collisions."""
    if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx_col = PhysxSchema.PhysxCollisionAPI(prim)
    physx_col.CreateContactOffsetAttr(0.008)
    physx_col.CreateRestOffsetAttr(0.0)

    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(prim)
    usd_mat = UsdPhysics.MaterialAPI(prim)
    usd_mat.CreateStaticFrictionAttr(GRASP_STATIC_FRICTION)
    usd_mat.CreateDynamicFrictionAttr(GRASP_DYNAMIC_FRICTION)
    usd_mat.CreateRestitutionAttr(GRASP_RESTITUTION)

    if not prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
        PhysxSchema.PhysxMaterialAPI.Apply(prim)
    px_mat = PhysxSchema.PhysxMaterialAPI(prim)
    px_mat.CreateFrictionCombineModeAttr("max")
    px_mat.CreateRestitutionCombineModeAttr("min")


def _ensure_grasp_physics_material_prim(stage: Usd.Stage) -> str:
    """Shared high-friction material (also applied per-prim for reliability)."""
    path = GRASP_PHYSICS_MATERIAL_PATH
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        prim = stage.DefinePrim(path, "Material")
    if not prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(prim)
    usd_mat = UsdPhysics.MaterialAPI(prim)
    usd_mat.CreateStaticFrictionAttr(GRASP_STATIC_FRICTION)
    usd_mat.CreateDynamicFrictionAttr(GRASP_DYNAMIC_FRICTION)
    usd_mat.CreateRestitutionAttr(GRASP_RESTITUTION)
    if not prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
        PhysxSchema.PhysxMaterialAPI.Apply(prim)
    px_mat = PhysxSchema.PhysxMaterialAPI(prim)
    px_mat.CreateFrictionCombineModeAttr("max")
    px_mat.CreateRestitutionCombineModeAttr("min")
    return path


def _apply_gripper_grasp_friction(stage: Usd.Stage, robot_root: str = "") -> int:
    """Super-grippy material on right gripper collision meshes."""
    roots: list[str] = []
    for path in GRIPPER_GRASP_SUBTREES:
        if stage.GetPrimAtPath(path).IsValid():
            roots.append(path)
    if robot_root:
        candidate = f"{robot_root.rstrip('/')}/right_gripper"
        if stage.GetPrimAtPath(candidate).IsValid() and candidate not in roots:
            roots.append(candidate)

    tuned = 0
    for root_path in roots:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            continue
        root_tuned = 0
        for prim in Usd.PrimRange(root_prim):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
            if enabled.IsValid() and enabled.Get() is False:
                continue
            _apply_super_grippy_contact_material(prim)
            root_tuned += 1

        if root_tuned == 0:
            for prim in Usd.PrimRange(root_prim):
                if prim.GetTypeName() != "Mesh":
                    continue
                path = prim.GetPath().pathString
                if "gripper_r_rh_p12_rn" not in path:
                    continue
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(prim)
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True)
                if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    UsdPhysics.MeshCollisionAPI.Apply(prim)
                    UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr("convexHull")
                _apply_super_grippy_contact_material(prim)
                root_tuned += 1
        tuned += root_tuned
    return tuned


def _make_shirt_dynamic_rigid(stage: Usd.Stage, shirt_root: str, *, physics_grasp: bool = False) -> bool:
    """Dynamic rigid body on pile root + convex-hull mesh collision."""
    root_prim = stage.GetPrimAtPath(shirt_root)
    if not root_prim or not root_prim.IsValid():
        print(f"[scene_physics] Shirt root not found: {shirt_root}")
        return False

    inner_path = _shirt_inner_prim_path(shirt_root)
    inner = stage.GetPrimAtPath(inner_path)
    if inner and inner.IsValid():
        _remove_rigid_body(inner)

    mesh_paths = _find_active_mesh_paths(stage, shirt_root)
    if not mesh_paths:
        print(f"[scene_physics] No active meshes under shirt root: {shirt_root}")
        return False

    for mesh_path in mesh_paths:
        mesh_prim = stage.GetPrimAtPath(mesh_path)
        if mesh_prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
            _remove_rigid_body(mesh_prim)
        if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        UsdPhysics.CollisionAPI(mesh_prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI(mesh_prim).CreateApproximationAttr("convexHull")
        if physics_grasp:
            _apply_super_grippy_contact_material(mesh_prim)

    if not root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(root_prim)
    if not root_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
    rb = UsdPhysics.RigidBodyAPI(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(False)

    if not root_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(root_prim)
    mass_kg = PHYSICS_GRASP_SHIRT_MASS_KG if physics_grasp else 0.25
    UsdPhysics.MassAPI(root_prim).CreateMassAttr(float(mass_kg))

    if physics_grasp and root_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        px_rb = PhysxSchema.PhysxRigidBodyAPI(root_prim)
    elif physics_grasp:
        px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
    else:
        px_rb = None
    if px_rb is not None:
        if physics_grasp:
            px_rb.CreateLinearDampingAttr(0.8)
            px_rb.CreateAngularDampingAttr(4.0)
        else:
            px_rb.CreateLinearDampingAttr(0.2)
            px_rb.CreateAngularDampingAttr(0.4)

    print(
        f"[scene_physics] Shirt dynamic rigid body"
        f" ({'physics grasp' if physics_grasp else 'constraint grasp'}): {shirt_root}"
        + (f" mass={mass_kg:.2f}kg friction={GRASP_STATIC_FRICTION}" if physics_grasp else "")
    )
    return True


def _make_grasp_test_cube_rigid(stage: Usd.Stage, cube_root: str) -> bool:
    """Dynamic rigid body + box collision for the flat grasp-test cube."""
    root_prim = stage.GetPrimAtPath(cube_root)
    if not root_prim or not root_prim.IsValid():
        print(f"[scene_physics] Grasp test cube root not found: {cube_root}")
        return False

    mesh_paths = _find_active_mesh_paths(stage, cube_root)
    if not mesh_paths:
        print(f"[scene_physics] No meshes under grasp test cube: {cube_root}")
        return False

    for mesh_path in mesh_paths:
        mesh_prim = stage.GetPrimAtPath(mesh_path)
        if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
        if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        UsdPhysics.CollisionAPI(mesh_prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI(mesh_prim).CreateApproximationAttr("boundingCube")
        physx_col = PhysxSchema.PhysxCollisionAPI(mesh_prim)
        physx_col.CreateContactOffsetAttr(0.004)
        physx_col.CreateRestOffsetAttr(0.0)

    if not root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(root_prim)
    if not root_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
    rb = UsdPhysics.RigidBodyAPI(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(False)

    if not root_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(root_prim)
    UsdPhysics.MassAPI(root_prim).CreateMassAttr(0.12)

    print(f"[scene_physics] Grasp test cube rigid body: {cube_root}")
    return True


def _make_object_physics(
    stage: Usd.Stage,
    object_root: str,
    *,
    constraint_grasp: bool = False,
    physics_grasp: bool = False,
) -> bool:
    if object_root.rstrip("/") == GRASP_TEST_CUBE_ROOT:
        return _make_grasp_test_cube_rigid(stage, object_root)
    if constraint_grasp or physics_grasp:
        return _make_shirt_dynamic_rigid(stage, object_root, physics_grasp=physics_grasp)
    return _make_shirt_rigid(stage, object_root)


def _resolve_robot_filter_root(stage: Usd.Stage, preferred: str = "") -> str:
    if preferred:
        path = preferred.rstrip("/")
        if stage.GetPrimAtPath(path).IsValid():
            return path
    for path in ROBOT_COLLISION_FILTER_ROOTS:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    return ""


def _filter_shirt_robot_collisions(
    stage: Usd.Stage,
    shirt_roots: Iterable[str],
    robot_root: str = "",
) -> None:
    """Ignore contacts between shirt pile(s) and the robot articulation subtree."""
    robot_path = _resolve_robot_filter_root(stage, robot_root)
    if not robot_path:
        print("[scene_physics] Robot filter root not found; shirt may collide with robot")
        return

    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim or not robot_prim.IsValid():
        return

    filtered = 0
    for shirt_root in shirt_roots:
        shirt_prim = stage.GetPrimAtPath(shirt_root)
        if not shirt_prim or not shirt_prim.IsValid():
            continue
        shirt_path = Sdf.Path(shirt_root.rstrip("/"))
        robot_path_sdf = Sdf.Path(robot_path)

        shirt_fp = UsdPhysics.FilteredPairsAPI.Apply(shirt_prim)
        rel = shirt_fp.GetFilteredPairsRel()
        if not rel or not rel.IsValid():
            rel = shirt_fp.CreateFilteredPairsRel()
        rel.AddTarget(robot_path_sdf)

        robot_fp = UsdPhysics.FilteredPairsAPI.Apply(robot_prim)
        rel_r = robot_fp.GetFilteredPairsRel()
        if not rel_r or not rel_r.IsValid():
            rel_r = robot_fp.CreateFilteredPairsRel()
        rel_r.AddTarget(shirt_path)
        filtered += 1

    if filtered:
        print(
            f"[scene_physics] Shirt-robot collision filtered "
            f"({filtered} shirt root(s) vs {robot_path})"
        )


def enable_gpu_physics_for_deformables(world=None) -> None:
    """Turn on GPU broadphase/dynamics required by PhysX deformables (CPU sim device is fine)."""
    try:
        from isaacsim.core.api.world import World
        from isaacsim.core.simulation_manager import SimulationManager

        sim_world = world if world is not None else World.instance()
        if sim_world is not None:
            ctx = sim_world.get_physics_context()
            ctx.set_broadphase_type("GPU")
            ctx.enable_gpu_dynamics(flag=True)
            ctx.enable_fabric(True)
            ctx.enable_ccd(flag=False)
        else:
            SimulationManager.enable_gpu_dynamics(flag=True)
        print("[scene_physics] GPU dynamics enabled for deformables")
    except Exception as exc:
        print(f"[scene_physics] Could not enable GPU dynamics: {exc!r}")


def _align_shirts_to_crate_floor(
    stage: Usd.Stage,
    crate_path: str,
    shirt_roots: Iterable[str],
    *,
    physics_grasp: bool = False,
) -> None:
    floor_top_z = _crate_floor_top_z(physics_grasp=physics_grasp)
    clearance = SHIRT_ABOVE_FLOOR_M
    for shirt_root in shirt_roots:
        if shirt_root.rstrip("/") == GRASP_TEST_CUBE_ROOT:
            continue
        shirt_prim = stage.GetPrimAtPath(shirt_root)
        if not shirt_prim or not shirt_prim.IsValid():
            continue
        bounds = _prim_mesh_world_bounds(shirt_prim)
        if bounds is None:
            continue
        shirt_mn, _ = bounds
        target_shirt_bottom = floor_top_z + clearance
        delta_z = target_shirt_bottom - shirt_mn[2]
        if abs(delta_z) > 0.001:
            _nudge_prim_world_z(shirt_prim, delta_z)
            print(
                f"[scene_physics] Aligned shirt {shirt_root} by dz={delta_z:+.3f}m "
                f"(bottom target z={target_shirt_bottom:.3f}, floor z={floor_top_z:.3f})"
            )


def configure_scene_physics(
    stage: Usd.Stage | None = None,
    shirt_roots: Iterable[str] = SHIRT_ROOTS,
    crate_path: str = CRATE_PRIM,
    shirt_voxel_resolution: int = 8,
    soft_shirts: bool = False,
    constraint_grasp: bool = False,
    physics_grasp: bool = False,
    world=None,
    robot_filter_root: str = "",
) -> None:
    """Apply kinematic crate and shirt physics. Safe to call once per stage load."""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[scene_physics] No USD stage available")
        return

    try:
        _set_kinematic_crate(stage, crate_path, physics_grasp=physics_grasp)
    except Exception as exc:
        print(f"[scene_physics] Crate setup failed: {exc!r}")

    if soft_shirts:
        try:
            enable_gpu_physics_for_deformables(world=world)
            for root in shirt_roots:
                if not _make_shirt_deformable(stage, root, voxel_resolution=shirt_voxel_resolution):
                    print(f"[scene_physics] Failed to make shirt deformable: {root}")
        except Exception as exc:
            print(f"[scene_physics] Soft shirt setup failed: {exc!r}")
    else:
        try:
            for root in shirt_roots:
                if not _make_object_physics(
                    stage,
                    root,
                    constraint_grasp=constraint_grasp,
                    physics_grasp=physics_grasp,
                ):
                    print(f"[scene_physics] Failed to configure object physics: {root}")
        except Exception as exc:
            print(f"[scene_physics] Rigid object setup failed: {exc!r}")

    if physics_grasp:
        try:
            _ensure_grasp_physics_material_prim(stage)
            gripper_tuned = _apply_gripper_grasp_friction(stage, robot_filter_root)
            print(
                f"[scene_physics] Grasp contact material "
                f"(mu={GRASP_STATIC_FRICTION}/{GRASP_DYNAMIC_FRICTION}, combine=max) "
                f"on shirt meshes + {gripper_tuned} gripper collision prim(s)"
            )
        except Exception as exc:
            print(f"[scene_physics] Grasp friction tune failed: {exc!r}")

    try:
        _align_shirts_to_crate_floor(
            stage, crate_path, shirt_roots, physics_grasp=physics_grasp
        )
        if physics_grasp:
            top_z = _crate_floor_top_z(physics_grasp=True)
            print(
                f"[scene_physics] Physics grasp: shirt on small rest pad "
                f"(top z={top_z:.5f}, grippers can reach under fold)"
            )
    except Exception as exc:
        print(f"[scene_physics] Floor alignment failed: {exc!r}")

    if not physics_grasp:
        try:
            _filter_shirt_robot_collisions(stage, shirt_roots, robot_root=robot_filter_root)
        except Exception as exc:
            print(f"[scene_physics] Shirt-robot collision filter failed: {exc!r}")
    else:
        print("[scene_physics] Physics grasp: shirt-gripper contacts enabled (no robot collision filter)")
