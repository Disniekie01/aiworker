# How To Run And Test Each Component

This guide runs each component separately so you can isolate failures quickly.

## Quick setup for any machine

```bash
cd /path/to/robotis_vr_isaac
./scripts/setup_repo.sh
export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
```

Then use `${ROBOTIS_VR_ROOT}` in commands below instead of hardcoded user paths.

## 0) Stop old processes (optional but recommended)

```bash
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh stop
pkill -f "standalone_ffw_joint_teleop.py" || true
pkill -f "joint_targets_udp_bridge.py" || true
pkill -f "loop_joint_test.py" || true
```

---

## 1) Terminal A - Relay (ROS -> DDS + /ffw_isaac/joint_targets)

```bash
source /opt/ros/jazzy/setup.bash
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
source /home/disniekie/Robotis/robotis_vr_isaac/env_local.bash
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
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 /home/disniekie/Robotis/robotis_vr_isaac/scripts/joint_targets_udp_bridge.py \
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
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 /home/disniekie/Robotis/robotis_vr_isaac/scripts/loop_joint_test.py --hz 100
```

Expected:
- Isaac logs continue showing `lift_cmd/lift_meas` updates.

Stop with `Ctrl-C`.

---

## 5) Single-command spot checks (without loop test)

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
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
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
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
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh stop
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
source /home/disniekie/Robotis/robotis_vr_isaac/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 /home/disniekie/Robotis/robotis_vr_isaac/scripts/publish_pose_from_yaml.py \
  --config /home/disniekie/Robotis/robotis_vr_isaac/config/arm_forward_pose.yaml
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
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh start-hardware --hw-model sg2 --hw-all-in-one
```

Separate launches (follower + leader in background):

```bash
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh start-hardware --hw-model sg2
```

For BG2:

```bash
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh start-hardware --hw-model bg2
```

Check status / stop:

```bash
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh status
/home/disniekie/Robotis/robotis_vr_isaac/scripts/run_stack.sh stop
```

Reference guides:
- Hardware teleoperation: https://ai.robotis.com/ai_worker/operation_teleoperation_ai_worker.html
- VR teleoperation: https://ai.robotis.com/ai_worker/operation_vr_teleoperation_ai_worker.html

Scene files included in this repo:
- `scenes/Scene_clean.usda`
- `scenes/Scene.usd`

