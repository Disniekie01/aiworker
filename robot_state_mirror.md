# robot_state_mirror

Visualization tool that mirrors real robot joint states into Isaac Sim in real time.

## Overview

Subscribes to the `/joint_states` ROS2 topic published by a real robot and drives the corresponding Isaac Sim articulation to match. The purpose is **pure visualization** — not teleoperation or control.

- Physics simulation is kept minimal: gravity is not explicitly disabled, but `set_joint_positions()` overrides joint positions every frame so gravity has no visible effect
- ROS2 reception is handled by `SingleThreadedExecutor` + `spin_once()` inside the simulation loop — no separate spin thread
- Uses Isaac Sim's bundled ROS2 bridge extension (`isaacsim.ros2.bridge`) to avoid Python version conflicts with system ROS2

## Data Flow

```
Real robot
  │  sensor_msgs/JointState
  │  (name[], position[])
  ▼
ROS2 topic (/joint_states, ROS_DOMAIN_ID=30)
  │
  ▼
JointStateSub (rclpy Node)
  │  spin_once() — non-blocking poll every simulation step
  │  lock-protected buffer
  ▼
_map_joints()
  │  matches topic joint names → Isaac Sim joint indices
  │  unmatched joints stay at 0.0
  ▼
robot.set_joint_positions()
  │  direct position write, bypasses PD controller
  ▼
Isaac Sim Articulation (3D visualization)
```

### Key Design Decisions

| Item | Choice | Reason |
|---|---|---|
| ROS2 executor | `SingleThreadedExecutor` + `spin_once(timeout_sec=0)` | `MultiThreadedExecutor` with a background spin thread starves the GPU rendering scheduler |
| Joint control | `set_joint_positions()` | `set_joint_position_targets()` fights the internal PD controller and causes jitter |
| ROS2 runtime | Isaac-bundled rclpy (Python 3.11) | System ROS2 Jazzy ships Python 3.12-compiled libraries, incompatible with Isaac Sim's Python 3.11 |

## Prerequisites

**1. Confirm the real robot is publishing `/joint_states`**
```bash
ROS_DOMAIN_ID=30 ros2 topic echo /joint_states --once
```
`ROS_DOMAIN_ID=30` is the default value of AI WORKER.

See [ROBOOTIS' AI WORKER Documentation](https://ai.robotis.com/ai_worker/operation_teleoperation_ai_worker.html) for bringing up the physical robot.


**2. Confirm Isaac Sim is installed**
```bash
ls /isaac-sim/python.sh
```

**3. Confirm the USD scene exists**
```bash
ls scenes/Scene_clean.usda
```

**4. Confirm ROS_DOMAIN_ID**

All nodes (real robot and Isaac Sim) must share the same domain ID. Default is `30`.

## Usage

### Basic
```bash
./scripts/launch_isaac_mirror.sh
```

### Options

```
--usd-path <path>           USD scene file (default: scenes/Scene_clean.usda)
--articulation-prim <path>  Articulation root prim (default: /World/Robot/ffw_sg2_follower)
--spawn-prim <path>         Prim path to load the USD into (default: /World)
--topic <topic>             JointState topic to subscribe (default: /joint_states)
--headless                  Run without GUI
--isaac-root <path>         Isaac Sim install path (default: /isaac-sim)
```

### Stopping

Close the Isaac Sim window or press `Ctrl+C` in the terminal.
