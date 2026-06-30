# aiworker — shirt pick/place demo (Isaac Sim)

Deterministic **point-to-point** demo: an FFW SG2 robot picks a rigid shirt from a crate and places it back, driven by scripted poses in Isaac Sim (no VR).

You define **waypoints** (joints, base, crate) in a web tuner. Those points are saved in YAML. Playback **lerps** smoothly between each pair of points at a fixed rate.

All paths in this repo are **relative to the clone** (`$ROBOTIS_VR_ROOT`). Nothing depends on `~/Downloads` or a fixed home directory.

---

## How it works

```
  Web tuner (8765)          config/pick_place.yaml          Dashboard (8760)
  ─────────────────         ───────────────────────         ─────────────────
  Move robot live    →      Named steps: Start, GRIPP,  →   Start Isaac
  per step/point            Drop, … (each a full pose)       Run YAML playback
  Write YAML                duration + easing per step       Start/stop tuner
```

| Piece | Role |
|-------|------|
| **Points (steps)** | Named poses in `pick_place.yaml` — e.g. `Start`, `GRIPP`, `Drop`. Each stores all arm joints, grippers, lift, head, mobile base, and crate pose. |
| **Tuner** | Live UI to drag the robot into position for one step, then **Write YAML** to save that point. |
| **YAML playback** | Reads step A → step B and **linearly interpolates (lerp)** between them at 30 Hz with smooth easing. Not teleop — the path is fixed once YAML is written. |
| **Physics grasp** | At `GRIPP` the shirt latches to the gripper; at `Drop` it releases. |
| **Dashboard** | `run_launcher.sh` — start Isaac, run the sequence, open the tuner. |

**Do not run YAML playback and tuner live-publish together** — both use `/ffw_isaac/joint_targets`. Tune → Write YAML → **Live publish OFF** → run YAML.

---

## Dependencies

Everything below is installed on the **new machine** — not copied from a developer laptop path.

| Dependency | Pick/place demo | VR teleop (optional) |
|------------|-----------------|----------------------|
| Ubuntu 22.04+ | ✓ | ✓ |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) | ✓ | ✓ |
| Python **3.12** (`/usr/bin/python3.12`) | ✓ | ✓ |
| [git-lfs](https://git-lfs.com/) | ✓ | ✓ |
| [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) (`$ISAAC_ROOT`) | ✓ | ✓ |
| `colcon` + `ros2_ws` build | ✓ | ✓ |
| Cyclone DDS C + `robotis_dds_python` | — | ✓ (relay) |
| Quest + Vuer | — | ✓ |

Scene meshes (~1 GB) are **in this repository via Git LFS** under `scenes/newscene/`.

---

## Install (new machine)

### 1. System packages

```bash
sudo apt update
sudo apt install -y git git-lfs curl
# ROS 2 Jazzy — follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install -y ros-jazzy-desktop python3.12 python3.12-venv
```

Install **Isaac Sim** per NVIDIA docs and note the install path (usually `~/isaacsim`).

### 2. Clone repository + scene assets (LFS)

```bash
git lfs install
git clone https://github.com/Disniekie01/aiworker.git
cd aiworker

export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"

git lfs pull
./scenes/newscene/verify_assets.sh
```

Expected scene entry (all paths inside the repo):

```text
${ROBOTIS_VR_ROOT}/scenes/newscene/newscene.usda
```

If `verify_assets.sh` fails, install `git-lfs` and run `git lfs pull` again.

### 3. Build ROS workspace

Pick/place uses the UDP bridge from this workspace (no VR relay required):

```bash
source /opt/ros/jazzy/setup.bash
cd "${ROBOTIS_VR_ROOT}/ros2_ws"
colcon build --packages-select ffw_vuer_dds_relay
source install/setup.bash
```

Add to every new shell (or put in `~/.bashrc`):

```bash
export ROBOTIS_VR_ROOT=/path/to/aiworker
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
```

### 4. Optional — VR relay dependencies

Only needed for Quest teleop (`--with-vuer`), **not** for the shirt dashboard demo:

1. Build Cyclone DDS C into `deps/cyclonedds-install` (or install `cyclonedds-dev`)
2. Clone `robotis_dds_python` into `third_party/` — see `third_party/README.md`
3. Python 3.12 venv + `pip install` per **`INSTALL.txt`**
4. `source "${ROBOTIS_VR_ROOT}/env_local.bash"` before the relay

Full steps: **`INSTALL.txt`**.

### 5. Quick sanity check

```bash
test -f "${ROBOTIS_VR_ROOT}/scenes/newscene/newscene.usda"
test -f "${ISAAC_ROOT}/python.sh"
/usr/bin/python3.12 -c "import rclpy; print('rclpy ok')"
./scenes/newscene/verify_assets.sh
```

---

## Run with the dashboard

```bash
cd "${ROBOTIS_VR_ROOT}"
./scripts/run_launcher.sh
```

Open **http://127.0.0.1:8760**

| Button | Action |
|--------|--------|
| **Start Isaac** | UDP bridge + Isaac Sim (`scenes/newscene/newscene.usda`) |
| **Start tuner** | Pose editor → http://127.0.0.1:8765 |
| **Run sequence** | Plays `config/pick_place.yaml` (~40–50 s) |
| **Stop** | Stops each service |

### Typical session

1. **Start Isaac** — wait for robot, crate, and shirt in the viewport (1–3 min first launch).
2. **Start tuner** → adjust poses if needed ([Tuning waypoints](#tuning-waypoints)).
3. **Live publish OFF** in the tuner.
4. **Run sequence** on the dashboard.

```bash
./scripts/run_stack.sh stop   # when finished
```

---

## Tuning waypoints

```bash
cd "${ROBOTIS_VR_ROOT}"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 scripts/joint_pose_web_tuner.py \
  --config config/pick_place.yaml --physics-grasp
```

http://127.0.0.1:8765

1. Select a **step** (`GRIPP`, `Drop`, …).
2. Adjust arm / gripper / base sliders.
3. Adjust **crate pose** per step if needed.
4. At **GRIPP**, tune **grasp attach offset** (ATTACH frame).
5. **Write YAML** → saves to `config/pick_place.yaml`.
6. **Live publish OFF** → run sequence from dashboard.

Playback lerps between consecutive steps (`duration`, `arm_duration`, `playback_speed`, `easing: smooth` in YAML).

---

## Manual run (no dashboard)

```bash
cd "${ROBOTIS_VR_ROOT}"
./scripts/run_stack.sh start --pick-place --with-isaac --physics-grasp
```

Second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 scripts/publish_pose_from_yaml.py --config config/pick_place.yaml
```

Isaac loads `scenes/newscene/newscene.usda` automatically when LFS assets are present.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Missing meshes / `verify_assets` fails | `git lfs install && git lfs pull` |
| Isaac not found | Set `ISAAC_ROOT` to your Isaac Sim install |
| `rclpy` errors | Use `/usr/bin/python3.12`, source ROS + `ros2_ws/install/setup.bash` |
| Robot jitters | Turn tuner **Live publish OFF** before YAML |
| Port 8760 / 8765 busy | `pkill -f pick_place_launcher` or `joint_pose_web_tuner` |
| Wrong scene / old paths | Use `scenes/newscene/newscene.usda`, not files outside the repo |

---

## Other modes

| Mode | Doc |
|------|-----|
| VR teleop (Quest) | `HOW_TO_RUN.md` |
| Hardware | `./scripts/run_stack.sh start-hardware` |
| Cyclone + relay | `INSTALL.txt` |

---

## License

ROBOTIS FFW assets and `robotis_dds_python` follow their upstream licenses. Third-party warehouse, crate, and shirt meshes are included via Git LFS for demo use only.
