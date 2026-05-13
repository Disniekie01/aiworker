# How To Run And Test Each Component

This guide runs each component separately so you can isolate failures quickly.

## Quick setup for any machine

```bash
cd /path/to/robotis_vr_isaac
export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
```

Then use `${ROBOTIS_VR_ROOT}` in commands below instead of hardcoded user paths.

### Cyclone DDS + `robotis_dds_python`

Before any relay or `run_stack.sh` step that launches the relay:

- Build **Cyclone DDS C** into `${ROBOTIS_VR_ROOT}/deps/cyclonedds-install` (or install dev packages and set `CYCLONEDDS_HOME`) — see `INSTALL.txt`. The `deps/` tree is not in git.
- Provide **`robotis_dds_python`** on disk: clone into `${ROBOTIS_VR_ROOT}/third_party/robotis_dds_python` (see `third_party/README.md`) **or** use `${ROBOTIS_VR_ROOT}/../robotis_lab/third_party/robotis_dds_python`.
- Create `${ROBOTIS_VR_ROOT}/.venv` (Python 3.12), install bindings per `INSTALL.txt`, then in every terminal that runs the relay:

```bash
source "${ROBOTIS_VR_ROOT}/env_local.bash"
```

If `libddsc` or `robotis_dds_python` is missing, `env_local.bash` warns on stderr; fix paths before debugging ROS.

## 0) Stop old processes (optional but recommended)

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" stop
pkill -f "standalone_ffw_joint_teleop.py" || true
pkill -f "joint_targets_udp_bridge.py" || true
pkill -f "loop_joint_test.py" || true
```

---

## 1) Terminal A - Relay (ROS -> DDS + /ffw_isaac/joint_targets)

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
source "${ROBOTIS_VR_ROOT}/env_local.bash"
export ROS_DOMAIN_ID=0
ros2 launch ffw_vuer_dds_relay relay.launch.py
```

Expected:
- Node starts without traceback.
- Logs mention `Vuer DDS relay running`.

---

## 2) Terminal B - UDP bridge (ROS JointState -> UDP JSON)

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 "${ROBOTIS_VR_ROOT}/scripts/joint_targets_udp_bridge.py" \
  --udp_host 127.0.0.1 \
  --udp_port 15000
```

Expected:
- `Forwarding /ffw_isaac/joint_targets -> udp://127.0.0.1:15000`

---

## 3) Terminal C - Isaac script (UDP input mode)

```bash
cd "${ISAAC_ROOT}" && ./python.sh "${ROBOTIS_VR_ROOT}/isaac_sim/standalone_ffw_joint_teleop.py" \
  --usd_path "${ROBOTIS_VR_ROOT}/scenes/Scene_clean.usda" \
  --spawn_prim /World \
  --articulation_prim /World/Robot/ffw_sg2_follower \
  --input_mode udp \
  --udp_host 127.0.0.1 \
  --udp_port 15000
```

Expected:
- Prints articulation and joint names.
- Repeated lines like:
  - `applying ... joints (lift_cmd=..., lift_meas=...)`

---

## 4) Terminal D - Continuous test motion publisher

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 "${ROBOTIS_VR_ROOT}/scripts/loop_joint_test.py" --hz 100
```

Expected:
- Isaac logs continue showing `lift_cmd/lift_meas` updates.

Stop with `Ctrl-C`.

---

## 5) Single-command spot checks (without loop test)

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
```

### 5.1 Lift within valid range
(`Scene.usda` has lift limits around `[-0.5, 0.0]`)

```bash
ros2 topic pub --once /leader/joystick_controller_right/joint_trajectory trajectory_msgs/msg/JointTrajectory \
'{joint_names: ["lift_joint"], points: [{positions: [-0.3]}]}'
```

### 5.2 Big visible arm motion

```bash
ros2 topic pub --once /leader/joint_trajectory_command_broadcaster_left/joint_trajectory trajectory_msgs/msg/JointTrajectory \
'{joint_names: ["arm_l_joint2","arm_l_joint3","arm_l_joint4"], points: [{positions: [1.2, -1.0, 0.8]}]}'
```

---

## 6) Verify topic flow

### Relay output check
```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 topic echo /ffw_isaac/joint_targets --once
```

You should see `lift_joint` and arm joint values updating.

---

## 7) Common failure patterns

- `rclpy._rclpy_pybind11` import error:
  - You used conda python (3.13). Use `/usr/bin/python3.12` for ROS scripts.
- Isaac runs but no visible movement:
  - Check Isaac log `lift_cmd/lift_meas`. If they change, control is working; likely a viewport/prim visibility issue.
- No data in Isaac:
  - Ensure Terminal A, B, C are all running.
  - Confirm `ROS_DOMAIN_ID=0` in all ROS terminals.

---

## 8) Quick teardown

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" stop
pkill -f "standalone_ffw_joint_teleop.py" || true
pkill -f "joint_targets_udp_bridge.py" || true
pkill -f "loop_joint_test.py" || true
```

---

## 9) Run arm-forward pose from YAML

This uses a preset file at `robotis_vr_isaac/config/arm_forward_pose.yaml` and
publishes trajectory steps to the same arm topics used by the relay.

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 "${ROBOTIS_VR_ROOT}/scripts/publish_pose_from_yaml.py" \
  --config "${ROBOTIS_VR_ROOT}/config/arm_forward_pose.yaml"
```

Notes:
- Values in YAML are radians.
- Keep value count aligned with `joint_names`.
- You can add more steps by extending `step_names` and adding matching arrays.

---

## 10) Hardware teleoperation option (Leader + Follower)

If your team has the physical hardware, use the hardware mode in `run_stack.sh`.

All-in-one (recommended first):

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" start-hardware --hw-model sg2 --hw-all-in-one
```

Separate launches (follower + leader in background):

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" start-hardware --hw-model sg2
```

For BG2:

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" start-hardware --hw-model bg2
```

Check status / stop:

```bash
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" status
"${ROBOTIS_VR_ROOT}/scripts/run_stack.sh" stop
```

Reference guides:
- Hardware teleoperation: https://ai.robotis.com/ai_worker/operation_teleoperation_ai_worker.html
- VR teleoperation: https://ai.robotis.com/ai_worker/operation_vr_teleoperation_ai_worker.html

Scene files included in this repo:
- `scenes/Scene_clean.usda`
- `scenes/Scene.usd`

---

## 11) Common failures (Cyclone / DDS Python)

- **`Cyclone DDS not found` or `libddsc` errors:** `deps/cyclonedds-install` was never built or `CYCLONEDDS_HOME` points at the wrong prefix. Follow `INSTALL.txt` and confirm `ls "${ROBOTIS_VR_ROOT}/deps/cyclonedds-install/lib"/libddsc.so*`.
- **`No module named 'robotis_dds'` (or similar):** clone `robotis_dds_python` into `third_party/` or use `robotis_lab` sibling layout (`third_party/README.md`), then `source env_local.bash` in the same shell as `ros2 launch`.
- **Warnings when sourcing `env_local.bash`:** fix the missing path it names before chasing ROS launch errors.
