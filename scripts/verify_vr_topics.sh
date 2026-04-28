#!/usr/bin/env bash
# Phase 1: confirm Vuer / vr_publisher topics (run after: ros2 launch robotis_vuer vr.launch.py model:=sg2)
set -euo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "== ros2 topic list (filtered) =="
ros2 topic list | grep -E 'goal_pose|leader/joint|cmd_vel|reactivate' || true
echo "== Echo /l_goal_pose (Ctrl+C after squeezing both grips on Quest) =="
timeout 5 ros2 topic echo /l_goal_pose --once || true
