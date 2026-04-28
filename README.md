# robotis_vr_isaac

VR teleoperation bridge for ROBOTIS SG2/BG2 workflows:

- `robotis_vuer` (Quest/Vuer input)
- ROS 2 relay (`ffw_vuer_dds_relay`) for IK + `/ffw_isaac/joint_targets`
- UDP bridge for Isaac Sim Python compatibility
- Optional hardware launch mode for teams with leader/follower hardware

## What is included

- `scripts/run_stack.sh`: one-command process launcher
- `scripts/setup_repo.sh`: initializes required submodules
- `scripts/publish_pose_from_yaml.py`: apply preset arm poses
- `config/arm_forward_pose.yaml`: sample ready pose
- `scenes/Scene_clean.usda` + `scenes/Scene.usd`: default Isaac scene files

---

## 1) Clone and bootstrap

```bash
git clone https://github.com/Disniekie01/aiworker.git
cd aiworker
./scripts/setup_repo.sh
```

---

## 2) Prerequisites

- Ubuntu + ROS 2 Jazzy installed (`/opt/ros/jazzy`)
- Python 3.12 available for ROS scripts
- Isaac Sim installed (default assumed at `$HOME/isaacsim`)
- Quest and host machine on same network

For non-sudo CycloneDDS/python binding setup, see `INSTALL.txt`.

---

## 3) Environment setup

```bash
cd /path/to/aiworker
export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
```

If you use local CycloneDDS + python bindings:

```bash
source "${ROBOTIS_VR_ROOT}/env_local.bash"
```

---

## 4) Build ROS workspace

```bash
source /opt/ros/jazzy/setup.bash
cd "${ROBOTIS_VR_ROOT}/ros2_ws"
colcon build --packages-select ffw_vuer_dds_relay
source install/setup.bash
```

---

## 5) Run simulation stack (Vuer + Isaac)

```bash
cd "${ROBOTIS_VR_ROOT}"
./scripts/run_stack.sh start --with-vuer --with-isaac
./scripts/run_stack.sh status
```

Notes:
- If `--usd-path` is omitted, launcher uses `scenes/Scene_clean.usda`.
- To stop all managed processes:

```bash
./scripts/run_stack.sh stop
```

---

## 6) Quest connect flow

Open on Quest browser:

`https://vuer.ai?ws=wss://<HOST_IP>:8012`

Then:
- Click **Enter VR**
- Confirm controller axes visible
- Hold both squeeze buttons (normal safety gate)
- Align arms and activate as required by your controller flow

---

## 7) Hardware mode (for teams with real hardware)

All-in-one:

```bash
./scripts/run_stack.sh start-hardware --hw-model sg2 --hw-all-in-one
```

Separate follower + leader:

```bash
./scripts/run_stack.sh start-hardware --hw-model sg2
```

BG2:

```bash
./scripts/run_stack.sh start-hardware --hw-model bg2
```

---

## 8) Apply preset arm pose

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 "${ROBOTIS_VR_ROOT}/scripts/publish_pose_from_yaml.py" \
  --config "${ROBOTIS_VR_ROOT}/config/arm_forward_pose.yaml"
```

---

## 9) Troubleshooting quick checks

- Vuer not tracking: check `./scripts/run_stack.sh status` and ensure port `8012` is not occupied by stale processes.
- No motion in Isaac: verify `/ffw_isaac/joint_targets` is publishing.
- Hardware mode fails with `ffw_bringup` not found: source/build correct hardware workspace.

For deep debugging and component-by-component commands, see `HOW_TO_RUN.md`.
