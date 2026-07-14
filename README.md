# ROS 2 Gazebo Obstacle Avoidance

A ROS 2 project implementing a custom differential-drive robot — modeled from scratch in URDF/Xacro with a front-facing RGB camera and a 360° LiDAR — that navigates a hand-built Gazebo obstacle course using **reactive, map-free LiDAR avoidance**, with everything visualized live in RViz2.

## Demo

![Obstacle Avoidance demo](demo.gif)

*(full run, screen-captured at 8fps — Gazebo world on the left, RViz displays/camera feed/robot TF and a terminal on the right)*

## Project Overview

The robot drives straight through the world at full speed until its LiDAR detects an obstacle inside a forward cone. It then commits to a turn direction — toward whichever side has more clearance — and rotates in place until the path ahead is clear again, at which point it resumes driving straight. This loop repeats indefinitely, letting the robot navigate the entire obstacle course with no map and no planner, purely off the live scan.

Key behaviors demonstrated:
- Reactive (map-free) obstacle avoidance driven entirely by live LiDAR data
- Direction-locking: the turn side is chosen once per avoidance episode and held, so the robot can't oscillate between two obstacles
- Hysteresis between the stop and resume thresholds to prevent flickering right at the trigger boundary
- A robot modeled entirely from scratch in URDF/Xacro, with a simulated RGB camera and GPU LiDAR bridged from Gazebo into ROS 2
- A hand-authored Gazebo world (SDF) built as an obstacle course, with live camera + LiDAR/TF visualization in RViz2

## System Architecture

```
test_world.sdf (Gazebo) ──spawn── my_robot (diff-drive base + camera + LiDAR)
                                          │
                              ros_gz_bridge (parameter_bridge)
                                          │
                ┌─────────────────────────┼─────────────────────────┐
                ▼                         ▼                         ▼
        /camera/image_raw             /scan                 /joint_states, /tf
                │                         │                         │
                ▼                         ▼                         ▼
              RViz2                obstacle_avoider                RViz2
         (visualization)          (my_robot_autonomy)          (robot model)
                                          │
                                      /cmd_vel
                                          │
                                          ▼
                              ros_gz_bridge ──► gz-sim-diff-drive-system
```

## Packages

| Package | Build type | Purpose |
|---|---|---|
| [`my_robot_description`](src/my_robot_description) | `ament_cmake` | Robot geometry (base, wheels, camera, LiDAR) in URDF/Xacro, RViz config, standalone display launch |
| [`my_robot_bringup`](src/my_robot_bringup) | `ament_cmake` | Main Gazebo + RViz launch file, `ros_gz_bridge` topic config, custom obstacle-course world |
| [`my_robot_autonomy`](src/my_robot_autonomy) | `ament_python` | `obstacle_avoider` reactive avoidance node |

`my_robot_description` and `my_robot_bringup` are `ament_cmake` packages used purely for their `install(DIRECTORY ...)` mechanism — there is no C++ in this project. Every executable behavior lives in the Python `obstacle_avoider` node; the CMake packages just install URDF/Xacro, launch files, RViz config, world files, and the bridge config into the share directory so `ros2 launch` can find them.

---

## How It Was Built

### Robot Description (URDF/Xacro)

The robot is composed from five Xacro files, assembled by a top-level entry point:

- **`common_properties.xacro`** — shared `blue`/`grey` materials and three reusable inertia macros (`box_inertia`, `cylinder_inertia`, `sphere_inertia`) that compute the inertia tensor from mass and dimensions, so every link below just calls a macro instead of hand-deriving inertia values.

- **`mobile_base.xacro`** — defines `base_footprint` (a massless reference frame at ground level) and `base_link` (a `0.6 × 0.4 × 0.2 m`, 5 kg box). A `wheel_link` macro is instantiated once per side (`left`/`right`) to produce two `0.1 m`-radius drive wheels on **continuous** joints (unbounded rotation, required for driving), plus a passive `caster_wheel_link` (a sphere, both visually and for collision, so it slides freely in any direction) mounted toward the front.

- **`mobile_base_gazebo.xacro`** — wires the base to Gazebo:
  - `gz-sim-diff-drive-system` plugin drives `base_left_wheel_joint`/`base_right_wheel_joint` from `/cmd_vel`, with a `0.45 m` wheel separation matching the URDF geometry, and publishes odometry as `odom → base_footprint`.
  - `gz-sim-joint-state-publisher-system` publishes both wheel joint states so `robot_state_publisher` can complete the TF tree.
  - The caster's friction (`mu1`/`mu2 = 0.1`) is deliberately low so it doesn't resist the robot turning in place.

- **`camera.xacro`** — a small box `camera_link` mounted at the front of the base, plus a second, non-visual `camera_link_optical` frame rotated `(-π/2, 0, -π/2)` relative to it. This second frame exists because URDF/ROS convention is X-forward/Z-up, while camera optical convention (and the image data itself) is Z-forward/Y-down (REP 103) — the Gazebo camera sensor is told to publish using `camera_link_optical` as its `optical_frame_id` so the image and its TF agree. The sensor itself is `640×480`, ~80° horizontal FOV, Gaussian pixel noise, 20 Hz.

- **`lidar.xacro`** — a cylinder `lidar_link` on top of the base carrying a `gpu_lidar` sensor: 360 samples over a full 2π sweep (1° resolution), `0.12–8.0 m` range, Gaussian range noise, 10 Hz, publishing to `/scan`.

- **`my_robot.urdf.xacro`** — the top-level file: just five `xacro:include`s in order (properties → base → base's Gazebo plugins → camera → LiDAR). This is the file actually pointed to by both launch files.

### RViz Configuration

`urdf_config.rviz` is a saved, pre-built RViz layout (RobotModel, Camera, LaserScan, TF, Grid, fixed frame `base_footprint`) so both launch files can open RViz straight into a useful view with `-d urdf_config.rviz` instead of everyone re-adding displays by hand every run.

### Standalone Visualization Launch

`my_robot_description` ships its own `display.launch.py` / `display.launch.xml` (Python and XML versions of the same thing): `robot_state_publisher` + `joint_state_publisher_gui` (slider GUI for manually driving each joint) + `rviz2` — no Gazebo involved. This is for iterating on the URDF/Xacro geometry quickly without booting the simulator each time.

### Custom Gazebo World

`test_world.sdf` (515 lines) is a hand-authored SDF world: a ground plane and directional sun light, ten statically-placed colored obstacles (boxes and cylinders at explicit poses forming a course to weave through), one free (non-static) box, and a four-wall rectangular `boundary_fence` (~12.4 × 9.4 m) so the robot can't drive out of the arena. It enables the `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, and `Sensors` (with the `ogre2` render engine, required for the camera/LiDAR sensors to actually produce data) system plugins.

### ROS 2 ↔ Gazebo Bridge

`gazebo_bridge.yaml` configures `ros_gz_bridge`'s `parameter_bridge` with seven topic mappings between ROS 2 and Gazebo Transport: `/clock`, `/joint_states`, and `/tf` flow Gazebo → ROS; `/cmd_vel` flows ROS → Gazebo into the diff-drive plugin; `/camera/camera_info`, `/camera/image_raw`, and `/scan` flow Gazebo → ROS from the simulated sensors.

### Bringup Launch File

`my_robot_gazebo.launch.xml` ties everything together: it xacro-processes the URDF into a `robot_description` param for `robot_state_publisher`, includes `ros_gz_sim`'s `gz_sim.launch.py` pointed at `test_world.sdf` (`-r` to start running immediately rather than paused), spawns the robot into that running world via `ros_gz_sim create -topic robot_description` (no separate file path needed — it reads the same param), starts the `parameter_bridge` with the YAML config above, and launches `rviz2` with the saved config. The `obstacle_avoider` node is included last, gated behind an `autonomous` launch argument (default `false`) — so the world can be explored/teleoperated manually before turning autonomy on.

### Autonomy Package

`my_robot_autonomy` is a standard `ament_python` package: `package.xml` + `setup.py`/`setup.cfg`, a `resource/` marker file, and a console-script entry point (`obstacle_avoider = my_robot_autonomy.obstacle_avoider:main`). `test/` holds the standard ament lint scaffolding (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`) generated by the package template, runnable via `colcon test`.

The node itself subscribes to `/scan` with `qos_profile_sensor_data` (best-effort) rather than the default reliable QoS — the bridged `LaserScan` is published best-effort on the Gazebo side, and a reliable subscriber would simply never receive it. See **Control Algorithm** below for the avoidance logic.

---

## Control Algorithm

`obstacle_avoider` implements a simple reactive state machine over the raw `/scan` data — no costmap, no planner:

1. Split the scan into a forward cone (`±front_half_angle_deg`) and its left/right halves.
2. While not avoiding: drive straight at `linear_speed`. If the closest point in the forward cone drops below `stop_distance`, enter avoidance mode and lock in a turn direction — toward whichever side (left/right half of the cone) currently has more clearance.
3. While avoiding: rotate in place at `angular_speed` in the locked direction. Once the forward cone clears past `resume_distance`, drop out of avoidance mode and resume driving straight.

The turn direction is picked once per episode rather than recomputed every scan, so it can't chatter between two nearby obstacles. `resume_distance > stop_distance` gives hysteresis so the robot doesn't flicker in and out of avoidance right at the trigger boundary.

### Parameters (`obstacle_avoider`)

| Parameter | Default | Description |
|---|---|---|
| `linear_speed` | `0.7` | Forward speed (m/s) while no obstacle is in range |
| `angular_speed` | `2.0` | Rotation speed (rad/s) while avoiding |
| `front_half_angle_deg` | `45.0` | Half-angle of the forward detection cone, in degrees |
| `stop_distance` | `1.0` | Distance (m) to the closest obstacle in the cone that triggers avoidance |
| `resume_distance` | `1.4` | Distance (m) the cone must clear to resume straight driving |

## Nodes

| Node (executable) | Package | Role |
|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF from the xacro-processed URDF |
| `create` | `ros_gz_sim` | Spawns `my_robot` into the running Gazebo world from `/robot_description` |
| `parameter_bridge` | `ros_gz_bridge` | Bridges topics between ROS 2 and Gazebo Transport |
| `rviz2` | `rviz2` | Visualizes the camera feed, LiDAR-driven TF, and robot model |
| `obstacle_avoider` | `my_robot_autonomy` | Reactive LiDAR-based avoidance — only launched when `autonomous:=true` |

## Robot & Sensors

| Component | Details |
|---|---|
| Base | `0.6 × 0.4 × 0.2 m` box, two continuous-joint drive wheels + a passive caster |
| Drive | `gz-sim-diff-drive-system` plugin, `0.45 m` wheel separation, `0.1 m` wheel radius |
| Camera | RGB, `640×480`, ~80° horizontal FOV, 20 Hz, optical-frame convention joint |
| LiDAR | `gpu_lidar`, 360 samples/revolution (1° resolution), `0.12–8.0 m` range, 10 Hz |

## Languages & Tooling

| Language / format | Where | Approx. lines |
|---|---|---|
| SDF | Gazebo world (`test_world.sdf`) | ~515 |
| XML (Xacro/URDF + launch) | Robot description, launch files | ~350 |
| RViz config (YAML-based) | Saved display layout | ~245 |
| Python | `obstacle_avoider` node + package scaffolding | ~230 |
| YAML | `ros_gz_bridge` topic config | ~40 |
| CMake | `ament_cmake` build glue (no compiled code) | ~30 |

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`ros-jazzy-ros-gz`)

## Installation

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/frankNumfor/ros2-gazebo-obstacle-avoidance.git .

colcon build
source install/setup.bash
```

## Usage

```bash
# Launch Gazebo, spawn the robot, start the ros_gz bridge and RViz2
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

# Same, with autonomous obstacle avoidance running
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml autonomous:=true
```

```bash
# View the robot model alone in RViz2 (no Gazebo), with a joint_state_publisher_gui
ros2 launch my_robot_description display.launch.py
```

### Drive manually / tune avoidance at runtime

```bash
# Manual teleop (when not running autonomous:=true)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Run the avoidance node standalone with custom parameters
ros2 run my_robot_autonomy obstacle_avoider --ros-args -p stop_distance:=1.5 -p angular_speed:=1.0
```

## What This Project Demonstrates

- Modeling a mobile robot entirely from scratch in URDF/Xacro — links, continuous drive joints, inertia macros, and a camera optical-frame convention joint
- Reactive, sensor-driven autonomous control with no map or planner, made stable under noisy continuous sensor input via hysteresis and direction-locking
- Gazebo Harmonic simulation integration: `gz-sim` system/sensor plugins, `ros_gz_sim` spawning, and `ros_gz_bridge` topic bridging with explicit per-topic direction
- Hand-authoring a Gazebo world in SDF, including a bounded obstacle course
- Multi-sensor robot (camera + LiDAR) with live visualization in RViz2, driven by a saved, shareable display config
- ROS 2 XML launch files with conditional node inclusion via launch arguments
- Structuring a workspace across both `ament_cmake` (asset-only) and `ament_python` (executable) package types

## Skills & Technologies

`ROS 2 Jazzy` `Gazebo Harmonic` `URDF/Xacro` `SDF` `ros_gz_bridge` `Python` `RViz2` `LiDAR Simulation` `Reactive Control` `YAML` `CMake` `Ubuntu 24.04`
