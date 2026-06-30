# aiworker — shirt pick/place demo (Isaac Sim)

Deterministic **point-to-point** demo: an FFW SG2 robot picks a rigid shirt from a crate and places it back, driven by scripted poses in Isaac Sim (no VR).

You define **waypoints** (joints, base, crate) in a web tuner. Those points are saved in YAML. Playback **lerps** smoothly between each pair of points at a fixed rate.

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
| **Physics grasp** | At `GRIPP` the shirt latches to the gripper; at `Drop` it releases. Grip is friction + kinematic follow, not a scripted shirt path. |
| **Dashboard** | `run_launcher.sh` — one page to start Isaac, run the sequence, and open the tuner. |

This is a **repeatable demo**, not adaptive manipulation: same points and timings every run unless you change YAML.

**Do not run YAML playback and tuner live-publish at the same time** — both send to `/ffw_isaac/joint_targets` and will fight each other. Tune → Write YAML → turn **Live publish OFF** → run YAML.

---

## Requirements

- Ubuntu with **ROS 2 Jazzy** (`/opt/ros/jazzy`)
- **Python 3.12** (`/usr/bin/python3.12`)
- **Isaac Sim** (default `$HOME/isaacsim`)
- **Scene assets** ~1 GB (robot USD, warehouse, crate, shirt) — not in git; see [Scene assets](#scene-assets)
- Pick/place stack does **not** need the VR relay or Quest

Optional (VR / hardware): see `INSTALL.txt` and `HOW_TO_RUN.md`.

---

## Install

### 1. Clone

```bash
git clone https://github.com/Disniekie01/aiworker.git
cd aiworker
export ROBOTIS_VR_ROOT="$(pwd)"
export ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
```

### 2. Scene assets (included via Git LFS)

This repo ships the shirt scene binaries under `scenes/newscene/` using **Git LFS** (~1 GB).

```bash
# One-time on each machine
git lfs install

# If you already cloned without LFS files:
cd aiworker
git lfs pull
./scenes/newscene/verify_assets.sh
```

Fresh clone (LFS downloads automatically if `git-lfs` is installed):

```bash
git clone https://github.com/Disniekie01/aiworker.git
cd aiworker
git lfs pull   # safe to run again; ensures all LFS objects are present
./scenes/newscene/verify_assets.sh
```

**Without Git LFS** (not recommended): use the tarball flow in `scenes/newscene/README.md` (`package_assets.sh` / `install_assets.sh`).

Scene entry: `scenes/newscene/newscene.usda`

### 3. Python + ROS workspace

Pick/place only needs the UDP bridge (no Cyclone relay). Minimal setup:

```bash
source /opt/ros/jazzy/setup.bash
cd "${ROBOTIS_VR_ROOT}/ros2_ws"
colcon build --packages-select ffw_vuer_dds_relay
source install/setup.bash
```

If you also use VR teleop, build Cyclone DDS + `robotis_dds_python` per **`INSTALL.txt`** and `source env_local.bash` before the relay.

### 4. Check scene

```bash
"${ROBOTIS_VR_ROOT}/scenes/newscene/verify_assets.sh"
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
| **Start Isaac** | UDP bridge + Isaac Sim with physics-grasp shirt scene |
| **Start tuner** | Pose editor at http://127.0.0.1:8765 |
| **Run sequence** | Plays `config/pick_place.yaml` (~40–50 s at 70% speed) |
| **Stop** | Stops each service |

### Typical session

1. **Start Isaac** — wait until robot, crate, and shirt appear in the viewport (1–3 min first launch).
2. **Start tuner** → **Open tuner** — adjust poses if needed (see [Tuning waypoints](#tuning-waypoints)).
3. Turn **Live publish OFF** in the tuner.
4. **Run sequence** on the dashboard — watch pick → carry → drop.

Stop everything:

```bash
./scripts/run_stack.sh stop
```

---

## Tuning waypoints

Open the tuner (from dashboard or manually):

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 scripts/joint_pose_web_tuner.py \
  --config config/pick_place.yaml --physics-grasp
```

http://127.0.0.1:8765

1. Pick a **step** from the dropdown (`GRIPP`, `Drop`, …).
2. Move **arm / gripper / base** sliders until the pose looks right in Isaac.
3. Adjust **crate pose** sliders if the crate should move per step.
4. At **GRIPP**, tune **grasp attach offset** while the shirt is latched (ATTACH frame, not mesh inspector).
5. Click **Write YAML** — saves points to `config/pick_place.yaml`.
6. Turn **Live publish OFF**, then run the sequence from the dashboard.

### What gets lerped

`publish_pose_from_yaml.py` walks `step_names` in order. For each transition it lerps:

- All arm joints and grippers (`arm_duration`, default 1 s)
- Lift, head, base (`duration`, default 2 s — base can take longer on steps like `Turn`)
- Crate pose (same timing as arms)

Settings in YAML: `duration`, `arm_duration`, `playback_speed`, `publish_hz`, `easing: smooth`.

Example steps: `Start → Ready → … → GRIPP → Lift → Turn → Drop → Rest`.

---

## Manual run (without dashboard)

```bash
cd "${ROBOTIS_VR_ROOT}"
./scripts/run_stack.sh start --pick-place --with-isaac --physics-grasp
```

Second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source "${ROBOTIS_VR_ROOT}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/bin/python3.12 scripts/publish_pose_from_yaml.py --config config/pick_place.yaml
```

---

## Scene assets

Shipped in **`scenes/newscene/`** via **Git LFS** (~1 GB). After `git clone`, run `git lfs pull` and `./verify_assets.sh`.

Fallback (no LFS): `package_assets.sh` / `install_assets.sh` — see below.

**Package on a machine that has them:**

```bash
cd scenes/newscene && ./package_assets.sh
# → dist/newscene-assets-YYYYMMDD.tar.gz
```

**Install on a new machine:**

```bash
cd scenes/newscene && ./install_assets.sh dist/newscene-assets-YYYYMMDD.tar.gz
```

Scene file used by the stack:

```text
scenes/newscene/newscene.usda
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Blank Isaac / missing robot | `./scenes/newscene/verify_assets.sh` |
| Robot jitters during playback | Tuner **Live publish** must be OFF |
| Port 8760 or 8765 in use | `pkill -f pick_place_launcher` or `joint_pose_web_tuner` |
| Commands ignored | Run from repo root; check `./scripts/run_stack.sh status` |
| Shirt doesn’t stick | Tune grasp attach at `GRIPP` in tuner, Write YAML |

---

## Other modes

| Mode | Doc |
|------|-----|
| VR teleop (Quest) | `HOW_TO_RUN.md` — `./scripts/run_stack.sh start --with-vuer --with-isaac` |
| Hardware bringup | `README` hardware section / `run_stack.sh start-hardware` |
| Full Cyclone + relay install | `INSTALL.txt` |

---

## License

ROBOTIS FFW assets and `robotis_dds_python` follow their upstream licenses. Third-party warehouse, crate, and shirt assets are not redistributed in this repository — use `package_assets.sh` or your own licensed copies.
