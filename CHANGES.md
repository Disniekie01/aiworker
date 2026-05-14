# Stack Launch Fixes: Vuer (Docker) + Isaac Sim Path

## Summary

Fixed two broken launch paths in `run_stack.sh --with-vuer --with-isaac`, and
improved the stop/status lifecycle for the vuer Docker container.

---

## 1. Vuer: switched from native ROS 2 launch to Docker container

**Problem:** `--with-vuer` failed immediately with `ModuleNotFoundError: No module
named 'nest_asyncio'`. The `robotis_vuer` node requires `nest_asyncio` and `vuer`
(ROBOTIS fork v0.1.6), which are only available inside the pre-built
`robotis/robotis-applications:0.0.4` Docker image.

**Fix (`scripts/run_stack.sh`):**
```
# before
start_proc vuer "${base_env} && source .../install/setup.bash && ros2 launch robotis_vuer vr.launch.py model:=sg2"

# after
start_proc vuer "docker exec -e ROS_DOMAIN_ID=${DOMAIN_ID} robotis-applications bash -ic 'ros2 launch robotis_vuer vr.launch.py model:=sg2'"
```

Key details:
- `bash -ic` (interactive mode) is required to source `.bashrc`, which sources
  `/opt/ros/jazzy/setup.bash` inside the container. Without `-i`, `.bashrc` exits
  early at the `case $-` interactive check and `ros2` is not found.
- `ROS_DOMAIN_ID` is passed via `-e` to override the container default (30) with
  the stack's configured domain (default 0).
- The `robotis-applications` container must be started separately with
  `robotis_applications/docker/container.sh start` before running the stack.

**Prerequisite — build `robotis_applications` workspace:**
```bash
# Clone the missing dependency (has its own .git, not tracked in this repo)
cd robotis_applications
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_interfaces.git

# Build
source /opt/ros/jazzy/setup.bash
colcon build --install-base install
```

---

## 2. Isaac Sim: fixed wrong default path

**Problem:** `ISAAC_ROOT` defaulted to `$HOME/isaacsim` but Isaac Sim is installed
at `/isaac-sim` on this machine. The script used `${ISAAC_ROOT:-default}`, so if
`ISAAC_ROOT` was already exported in the shell (from a stale session), the wrong
path was used regardless of the default.

**Fix:**
- `scripts/run_stack.sh`: changed fallback default from `$HOME/isaacsim` to
  `/isaac-sim`.
- `env_local.bash`: added `export ISAAC_ROOT=/isaac-sim` as an unconditional
  assignment, so sourcing `env_local.bash` always sets the correct path for this
  machine. The `--isaac-root` flag on `run_stack.sh` still allows per-invocation
  override.

---

## 3. Python dependency installer (`install_python_deps.sh`)

New helper script that installs `cyclonedds==0.10.2` and `robotis_dds_python` into
the project `.venv`, using `CYCLONEDDS_HOME` from `deps/cyclonedds-install`.
Run once after cloning:
```bash
bash install_python_deps.sh
```

---

## 4. Improved `stop` lifecycle for vuer container processes

**Problem:** `run_stack.sh stop` only killed the host-side `docker exec` client
process (tracked by PID file). The container-internal `ros2 launch` and
`vr_publisher_sg2` processes were left running as orphans.

**Root cause of orphan:** When the docker exec client dies, the container's bash
session receives SIGHUP. `ros2 launch` may exit on SIGHUP, but its child process
(`vr_publisher_sg2`) is adopted by the container's init (PID 1) and survives.

**Fix (`cmd_stop`):** Added direct `docker exec pkill` calls before `stop_proc vuer`:
1. `pkill -f "[r]os2 launch"` — terminates ros2 launch, which cascades SIGINT to
   its children.
2. `sleep 1` — allows graceful shutdown.
3. `pkill -f "[r]obotis_vuer"` — cleans up any surviving orphan node processes.
4. `sleep 1` + `pkill -9 -f "[r]obotis_vuer"` — force-kills any remaining stragglers.

The `[r]` regex trick (`[r]obotis_vuer` instead of `robotis_vuer`) prevents `pkill`
from matching the `bash -c 'pkill -f "robotis_vuer"'` wrapper process itself.

---

## 5. Improved `status` output for vuer

`run_stack.sh status` now reports three separate lines for vuer:

```
[up]   vuer/docker_exec  (pid 12345)               # host-side docker exec client
[up]   vuer/launch       (container: ros2 launch)   # ros2 launch inside container
[up]   vuer/node         (container: vr_publisher_sg2)  # actual ROS 2 node
```

This makes zombie states immediately visible:
```
[down] vuer/docker_exec                             # docker exec client gone
[down] vuer/launch       (container: ros2 launch)   # launch also gone
[up]   vuer/node         (container: vr_publisher_sg2)  # but node still alive ← zombie
```

`status_proc` was extended with an optional display label parameter so the PID
file key (`vuer`) and the displayed name (`vuer/docker_exec`) can differ.
