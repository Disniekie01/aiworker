"""Inspect lift_joint properties from a USD file."""
import sys

USD_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/sungmin/Downloads/Thermo_SUNGMIN/Collected_Thermo_Fisher_TeleOp_Lab_v01/Thermo_Fisher_Lab_Scene_Flattened.usd"

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
from pxr import UsdPhysics, PhysxSchema

omni.usd.get_context().open_stage(USD_PATH)
app.update()

stage = omni.usd.get_context().get_stage()
OUT = "/tmp/lift_joint.txt"
lines = []

for prim in stage.Traverse():
    if not prim.IsValid() or prim.GetName() != "lift_joint":
        continue
    lines.append(f"Prim : {prim.GetPath()}")
    lines.append(f"Type : {prim.GetTypeName()}")
    lines.append(f"APIs : {prim.GetAppliedSchemas()}")
    lines.append("")
    for attr in prim.GetAttributes():
        val = attr.Get()
        if val is not None:
            lines.append(f"  {attr.GetName():50s} = {val}")

with open(OUT, "w") as f:
    f.write("\n".join(lines) + "\n")

app.close()
