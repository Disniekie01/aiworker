"""Print PhysxMimicJointAPI gearing for all gripper joints in a USD file."""
import sys

USD_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/sungmin/Downloads/Thermo_SUNGMIN/Collected_Thermo_Fisher_TeleOp_Lab_v01/Thermo_Fisher_Lab_Scene_Flattened.usd"

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.usd
from pxr import PhysxSchema, UsdPhysics

omni.usd.get_context().open_stage(USD_PATH)
app.update()

stage = omni.usd.get_context().get_stage()

OUT = "/tmp/gripper_joints.txt"
lines = []
lines.append(f"\n{'Joint name':30s}  {'drive?':>6}  {'mimic axis':>12}  {'gearing':>10}  {'offset':>8}")
lines.append("-" * 80)

for prim in stage.Traverse():
    if not prim.IsValid() or "gripper" not in prim.GetName().lower():
        continue

    has_drive = bool(UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear"))

    # PhysxMimicJointAPI.Get(prim, instanceName) — axis is the instance name
    mimic_rotX  = PhysxSchema.PhysxMimicJointAPI.Get(prim, "rotX")
    mimic_transX = PhysxSchema.PhysxMimicJointAPI.Get(prim, "transX")
    mimic = mimic_rotX or mimic_transX
    axis  = "rotX" if mimic_rotX else ("transX" if mimic_transX else "-")

    if mimic:
        gearing = mimic.GetGearingAttr().Get()
        offset  = mimic.GetOffsetAttr().Get()
    else:
        gearing = offset = "-"

    lines.append(f"{prim.GetName():30s}  {str(has_drive):>6}  {axis:>12}  {str(gearing):>10}  {str(offset):>8}")

with open(OUT, "w") as f:
    f.write("\n".join(lines) + "\n")

app.close()
