#!/usr/bin/env python3
"""Summarize physics grasp JSONL logs for debugging fly-off / latch issues."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


KEY_EVENTS = {
    "session_start",
    "config",
    "grip_close_start",
    "grip_settle",
    "overlap_check",
    "attach_pending",
    "attach_snapshot",
    "attach_created",
    "attach_failed",
    "attach_skipped",
    "attach_released",
    "collision_filter_applied",
    "shirt_anomaly",
    "shirt_velocity_zeroed",
}


def _load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as fp:
        for line_no, line in enumerate(fp, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"WARN: {path}:{line_no}: invalid JSON: {exc}", file=sys.stderr)
    return records


def _fmt_step(rec: dict[str, Any]) -> str:
    step = rec.get("sim_step", "?")
    motion = rec.get("motion_step")
    motion_name = rec.get("motion_step_name")
    if motion_name:
        return f"sim={step} motion={motion}({motion_name})"
    if motion is not None:
        return f"sim={step} motion={motion}"
    return f"sim={step}"


def summarize(records: list[dict[str, Any]]) -> None:
    if not records:
        print("No records.")
        return

    print(f"Records: {len(records)}")
    anomalies = [r for r in records if r.get("event") == "shirt_anomaly"]
    print(f"Anomalies: {len(anomalies)}")
    print()

    print("=== Timeline (key events) ===")
    for rec in records:
        event = rec.get("event")
        if event not in KEY_EVENTS:
            continue
        line = f"[{_fmt_step(rec)}] {event}"
        if event == "overlap_check":
            line += f" overlaps={rec.get('overlaps')} center_sep_m={rec.get('center_sep_m')}"
        elif event == "attach_created":
            line += (
                f" sustain={rec.get('sustain_mode', rec.get('latch_mode'))}"
                f" frame_err_m={rec.get('frame_err_m')} body1={rec.get('body1_path')}"
            )
        elif event == "attach_snapshot":
            line += (
                f" constraint_residual_m={rec.get('constraint_residual_m')}"
                f" frame_err_m={rec.get('frame_err_m')}"
            )
            body1 = rec.get("body1_fabric", {}).get("pos")
            if body1:
                line += f" shirt_pos={body1}"
        elif event == "attach_failed":
            line += f" reason={rec.get('reason')} error={rec.get('error')}"
        elif event == "shirt_anomaly":
            line += (
                f" speed={rec.get('linear_speed_m_s'):.3f}m/s"
                f" jump={rec.get('step_jump_m'):.3f}m"
                f" attached={rec.get('attached')}"
                f" pending={rec.get('attach_pending')}"
            )
        elif event == "grip_close_start":
            line += f" grip={rec.get('grip_used')}"
        print(line)

    if anomalies:
        print()
        print("=== First anomaly detail ===")
        first = anomalies[0]
        print(json.dumps(first, indent=2, default=str))

    attach = next((r for r in records if r.get("event") == "attach_snapshot"), None)
    if attach:
        print()
        print("=== Attach snapshot checks ===")
        print(f"frame_err_m: {attach.get('frame_err_m')}")
        print(f"constraint_residual_m: {attach.get('constraint_residual_m')}")
        for key in ("attach_prim", "shirt_root", "shirt_mesh"):
            snap = attach.get(key) or {}
            err = snap.get("fabric_usd_pos_err_m")
            if err is not None:
                print(f"{key} fabric/usd pos err: {err:.4f} m")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log",
        nargs="?",
        default="",
        help="JSONL log path (default: logs/physics_grasp/latest.jsonl under project root)",
    )
    args = parser.parse_args()

    if args.log:
        path = Path(args.log)
    else:
        root = Path(__file__).resolve().parent.parent
        path = root / "logs" / "physics_grasp" / "latest.jsonl"

    if not path.is_file():
        print(f"Log not found: {path}", file=sys.stderr)
        return 1

    summarize(_load_records(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
