# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""Bake YAML animation JSON into animated USD (headless Isaac Sim).

Run:
  cd ~/isaacsim && ./python.sh /path/to/robotis_vr_isaac/isaac_sim/export_yaml_animation_usd.py \\
    --animation /path/to/pick_place_animation.json \\
    --scene /path/to/Scene_clean.usda \\
    --output /path/to/pick_place_anim.usda
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
from pathlib import Path

_BAD_SITE_HINT = "/.venv/lib/python3.12/site-packages"
if _BAD_SITE_HINT in os.environ.get("PYTHONPATH", ""):
    py_entries = [p for p in os.environ["PYTHONPATH"].split(":") if _BAD_SITE_HINT not in p]
    os.environ["PYTHONPATH"] = ":".join(py_entries)
sys.path = [p for p in sys.path if _BAD_SITE_HINT not in p]

with contextlib.suppress(ModuleNotFoundError):
    import isaacsim  # noqa: F401

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

_ROOT = Path(__file__).resolve().parents[1]
_SCRIPTS = _ROOT / "scripts"
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from yaml_animation_to_usd import export_animation_usd  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YAML-sampled animation to animated USD")
    parser.add_argument("--animation", required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--joints-root", default="/World/Robot/ffw_sg2_follower/joints")
    parser.add_argument("--base-prim", default="/World/Robot")
    parser.add_argument(
        "--shirt-mesh",
        default=(
            "/World/Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell_01/"
            "Folded_TShirt_V_Neck_10_Pile_White_VRay_StemCell/TShirts_Hanging_V_neck_04"
        ),
    )
    parser.add_argument("--flatten", action="store_true")
    parser.add_argument(
        "--skip-base",
        action="store_true",
        help="Do not animate /World/Robot translate/yaw",
    )
    args = parser.parse_args()

    anim_path = Path(args.animation).expanduser()
    anim = json.loads(anim_path.read_text(encoding="utf-8"))
    export_animation_usd(
        anim,
        Path(args.scene).expanduser(),
        Path(args.output).expanduser(),
        joints_root=args.joints_root,
        base_prim=args.base_prim,
        shirt_mesh=args.shirt_mesh,
        animate_base=not args.skip_base,
        flatten=args.flatten,
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
