"""
Run in Omniverse Script Editor (Window > Script Editor).
Select one or more prims, set MATERIAL_PATH, then run.

Handles instanced CAD PRODUCT prims by uninstancing and binding on the Mesh child.
"""
import omni.usd
from pxr import Usd, UsdGeom, UsdShade, Sdf

MATERIAL_PATH = "/World/Looks/Rubber_Textured"


def bind_material_to_prim(prim, material):
    if not prim or not prim.IsValid():
        return

    if prim.IsInstance():
        prim.SetInstanceable(False)

    target = prim
    if prim.IsA(UsdGeom.Xform) or prim.GetTypeName() == "Xform":
        mesh_children = [c for c in prim.GetChildren() if c.IsA(UsdGeom.Mesh)]
        if len(mesh_children) == 1:
            target = mesh_children[0]

    if target.IsInstanceProxy():
        print(f"Skip instance proxy: {target.GetPath()}")
        return

    binding = UsdShade.MaterialBindingAPI.Apply(target)
    binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    print(f"Bound {material.GetPath()} -> {target.GetPath()}")



stage = omni.usd.get_context().get_stage()
material = UsdShade.Material(stage.GetPrimAtPath(MATERIAL_PATH))
if not material:
    raise RuntimeError(f"Material not found: {MATERIAL_PATH}")

for path in omni.usd.get_context().get_selection().get_selected_prim_paths():
    bind_material_to_prim(stage.GetPrimAtPath(path), material)
