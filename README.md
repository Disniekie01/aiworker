# robotis_vr_isaac

VR teleoperation bridge for ROBOTIS SG2/BG2 workflows:

- `robotis_vuer` (Quest/Vuer input)
- ROS 2 relay (`ffw_vuer_dds_relay`) for IK + `/ffw_isaac/joint_targets`
- UDP bridge for Isaac Sim Python compatibility
- Optional hardware launch mode for teams with leader/follower hardware

## What is included

- `scripts/run_stack.sh`: one-command process launcher
- `scripts/publish_pose_from_yaml.py`: apply preset arm poses
- `config/arm_forward_pose.yaml`: sample ready pose
- `scenes/Scene_clean.usda` + `scenes/Scene.usd`: default Isaac scene files

---

## 1) Clone and bootstrap

```bash
git clone https://github.com/Disniekie01/aiworker.git
cd aiworker
```

---

## 2) Prerequisites

- Ubuntu + ROS 2 Jazzy installed (`/opt/ros/jazzy`)
- Python 3.12 available for ROS scripts
- Isaac Sim installed (default assumed at `$HOME/isaacsim`)
- Quest and host machine on same network

For step-by-step Cyclone DDS C build and `pip install -e` for `robotis_dds_python`, see `INSTALL.txt`.

### Cyclone DDS and `robotis_dds_python` (relay)

The ROS relay (`ffw_vuer_dds_relay`) loads **Cyclone DDS C** (`libddsc`) and **ROBOTIS** [`robotis_dds_python`](https://github.com/ROBOTIS-GIT/robotis_dds_python). A fresh clone of this repo does **not** include either:

1. **`deps/cyclonedds-install`** — the directory `deps/` is **gitignored**. You must build Cyclone DDS C into that prefix (or install system `cyclonedds-dev` and point `CYCLONEDDS_HOME` accordingly). See `INSTALL.txt`.
2. **`robotis_dds_python`** — not vendored here. Either:
   - clone it under **`third_party/robotis_dds_python`** (see `third_party/README.md`), or  
   - keep a sibling checkout **`../robotis_lab/third_party/robotis_dds_python`** (original layout).

Create a **Python 3.12** venv at the repo root (`.venv`), install `cyclonedds` and `robotis_dds_python` per `INSTALL.txt`, then **always** `source env_local.bash` before `ros2 launch ... relay.launch.py` or `./scripts/run_stack.sh` so `LD_LIBRARY_PATH`, `CYCLONEDDS_HOME`, and `PYTHONPATH` are set. If something is missing, `env_local.bash` prints a short warning to stderr.

If you run everything inside a **ROBOTIS** Docker image that already ships Cyclone and `robotis_dds_python`, you may not need a local `deps/` build; match your image’s layout and `ROS_DOMAIN_ID` with the rest of the stack.

---

## 3) Environment setup

```bash
cd /path/to/aiworker
export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
```

For the relay and any process that imports `robotis_dds_python` / Cyclone from this repo’s venv:

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
- **Cyclone DDS not found** / import errors for `robotis_dds_python`: build Cyclone C under `deps/cyclonedds-install` (or set `CYCLONEDDS_HOME` to a valid prefix), clone `robotis_dds_python` per `third_party/README.md`, activate `.venv`, then `source env_local.bash` and read warnings on stderr.

For deep debugging and component-by-component commands, see `HOW_TO_RUN.md`.

---

## 10) Robot state mirror (Isaac Sim visualization)

Mirror real robot joint states into Isaac Sim only (assuming robot operation was executed separately with ROBOTIS' official repository).
See **[robot_state_mirror.md](robot_state_mirror.md)** for full details.

```bash
./scripts/launch_isaac_mirror.sh
```

This feature is **independent of the installation steps in sections 3–8**. The following are not required:

- Cyclone DDS build (`deps/cyclonedds-install`)
- `robotis_dds_python` clone
- Python 3.12 venv (`.venv`) / `env_local.bash`
- `ros2_ws` build (`ffw_vuer_dds_relay`)
- ROBOTIS Docker container
- VR / Quest hardware

Only the following are needed:

- ROS 2 Jazzy installed at `/opt/ros/jazzy` (FastRTPS libraries are loaded from there at runtime)
- Isaac Sim 5.x at `/isaac-sim`
- Real robot publishing `/joint_states` on `ROS_DOMAIN_ID=30`
