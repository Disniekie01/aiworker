"""
Run once in Omniverse: Window > Script Editor > Run

Clears stale session-layer overrides, uninstances prims, and binds rubber
materials on the root layer so drag-and-drop errors stop persisting.
"""
import omni.usd
from pxr import Sdf, Usd, UsdGeom, UsdShade

MATERIAL_PATH = "/World/Looks/Rubber_Local"
FALLBACK_MATERIAL_PATH = "/World/Looks/Rubber_Textured"
TARGETS = [
    "/World/Item_00",
    "/World/Item_00/Plane_02",
]


def _get_material(stage):
    for path in (MATERIAL_PATH, FALLBACK_MATERIAL_PATH):
        mat = UsdShade.Material(stage.GetPrimAtPath(path))
        if mat:
            return mat
    raise RuntimeError("No rubber material found under /World/Looks")


def _bind(prim, material):
    if not prim or not prim.IsValid():
        return

    if prim.IsInstance():
        prim.SetInstanceable(False)

    target = prim
    if prim.IsA(UsdGeom.Xform):
        meshes = [c for c in prim.GetChildren() if c.IsA(UsdGeom.Mesh)]
        if len(meshes) == 1:
            target = meshes[0]

    if target.IsInstanceProxy():
        print(f"Still an instance proxy, skipping: {target.GetPath()}")
        return

    binding = UsdShade.MaterialBindingAPI.Apply(target)
    binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    print(f"Bound {material.GetPath()} -> {target.GetPath()}")


ctx = omni.usd.get_context()
stage = ctx.get_stage()
root = stage.GetRootLayer()
session = stage.GetSessionLayer()

if session and not session.empty:
    print(f"Clearing session layer: {session.identifier}")
    session.Clear()

if not stage.GetPrimAtPath(MATERIAL_PATH):
    print(f"Warning: {MATERIAL_PATH} missing. Reload Scene.usd from disk first.")

material = _get_material(stage)

for path in TARGETS:
    _bind(stage.GetPrimAtPath(path), material)

# Also fix instanced CAD products if selected.
for path in ctx.get_selection().get_selected_prim_paths():
    _bind(stage.GetPrimAtPath(path), material)

root.Save()
print("Saved root layer:", root.identifier)
