# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
"""PyBullet IK helper for FFW BG2 arms (nomesh URDF)."""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import pybullet as p
except ImportError:
    p = None


class FfwBg2PyBulletIk:
    """Load stripped URDF and solve position+orientation IK for left/right wrist."""

    ARM_L_JOINT_NAMES = [f"arm_l_joint{i}" for i in range(1, 8)]
    ARM_R_JOINT_NAMES = [f"arm_r_joint{i}" for i in range(1, 8)]

    def __init__(self, urdf_path: str) -> None:
        if p is None:
            raise RuntimeError("pybullet is required for pose IK. pip install pybullet")
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(urdf_path)
        self._cid = p.connect(p.DIRECT)
        self._rid = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_MAINTAIN_LINK_ORDER)
        self._name_to_idx: Dict[str, int] = {}
        self._idx_to_name: Dict[int, str] = {}
        for i in range(p.getNumJoints(self._rid)):
            info = p.getJointInfo(self._rid, i)
            name = info[1].decode("utf-8")
            self._name_to_idx[name] = i
            self._idx_to_name[i] = name
        self._ee_l = self._name_to_idx.get("arm_l_joint7")
        self._ee_r = self._name_to_idx.get("arm_r_joint7")
        if self._ee_l is None or self._ee_r is None:
            p.disconnect(self._cid)
            raise RuntimeError("URDF must contain arm_l_joint7 and arm_r_joint7")
        # IK solution vector is ordered by movable DOFs, not by joint index.
        self._ik_slot: Dict[str, int] = {}
        slot = 0
        for i in range(p.getNumJoints(self._rid)):
            info = p.getJointInfo(self._rid, i)
            if info[2] == p.JOINT_FIXED:
                continue
            name = info[1].decode("utf-8")
            self._ik_slot[name] = slot
            slot += 1

    def close(self) -> None:
        if p is not None and self._cid is not None:
            try:
                p.disconnect(self._cid)
            except Exception:
                pass
            self._cid = None

    def _get_positions_map(self, joint_values: Dict[str, float]) -> None:
        for name, pos in joint_values.items():
            if name in self._name_to_idx:
                idx = self._name_to_idx[name]
                p.resetJointState(self._rid, idx, float(pos))

    def solve(
        self,
        side: str,
        pos_base: np.ndarray,
        quat_xyzw_base: np.ndarray,
        seed: Dict[str, float],
        *,
        max_iter: int = 80,
    ) -> Optional[Dict[str, float]]:
        """IK in base (URDF world) frame. quat_xyzw_base is geometry_msgs order x,y,z,w."""
        self._get_positions_map(seed)
        ee = self._ee_l if side == "left" else self._ee_r
        pos = pos_base.astype(np.float64).tolist()
        orn = quat_xyzw_base.astype(np.float64).tolist()
        try:
            sol = p.calculateInverseKinematics(
                self._rid,
                ee,
                pos,
                targetOrientation=orn,
                maxNumIterations=int(max_iter),
                residualThreshold=1e-4,
            )
        except Exception:
            return None
        names = self.ARM_L_JOINT_NAMES if side == "left" else self.ARM_R_JOINT_NAMES
        out: Dict[str, float] = {}
        for jn in names:
            si = self._ik_slot[jn]
            out[jn] = float(sol[si])
        return out
