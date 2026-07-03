# SemanticAwareGo2

A ROS 2 (Jazzy) workspace for the Unitree Go2 quadruped, simulated in Gazebo Harmonic with the [CHAMP](https://github.com/chvmp/champ) gait controller. The robot model includes IMU, 2D/3D LiDAR, a mono camera, and an Intel RealSense D435i depth camera. This is the base perception/locomotion stack that future semantic-aware navigation work will build on top of.

> **Status:** simulation and sensor bring-up only. Semantic perception (segmentation, scene understanding, etc.) is not implemented yet.

## Repository layout

```
.
├── Dockerfile               # ROS 2 Jazzy Desktop + Gazebo Harmonic + Nav2 + RealSense deps
├── docker-compose.yml       # dev container (X11 + GPU passthrough, bind-mounts the repo)
└── src/
    └── unitree_go2_ros2_jazzy/
        ├── champ/                    # CHAMP core control library (vendored)
        ├── champ_base/               # CHAMP ROS 2 driver nodes (vendored)
        ├── champ_msgs/                # CHAMP message definitions (vendored)
        ├── unitree_go2_description/   # URDF/xacro, meshes, Gazebo worlds
        └── unitree_go2_sim/           # Gazebo bring-up: launch files + gait/joint/link config
```

`champ*` and the Go2 description/sim packages originate from [chvmp/champ](https://github.com/chvmp/champ) and [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy) — see [`src/unitree_go2_ros2_jazzy/README.md`](src/unitree_go2_ros2_jazzy/README.md) for the full upstream feature list, sensor visualizations, and gait-tuning reference table.

## Prerequisites

- Docker + Docker Compose
- Linux host with X11 (for RViz/Gazebo GUI passthrough)
- `xhost +local:root` run once per session on the host, so the container can open GUI windows

## Quick start (Docker)

```bash
# 1. Build and enter the dev container
xhost +local:root
docker compose run --rm unitree_go2_dev bash

# 2. Inside the container: build the workspace
cd /home/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

To re-enter a container that's already running:

```bash
docker compose exec unitree_go2_dev bash
```

Stop everything with `docker compose down`.

## Running the simulation

Default world:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

TI building world:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch_TI.py
```

Add `rviz:=true` (default) or `rviz:=false` to toggle RViz, and `gui:=false` to run Gazebo headless.

### Teleoperation

In a second terminal (`docker compose exec unitree_go2_dev bash`, then `source install/setup.bash`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Sensor topics

| Sensor | Topics |
|---|---|
| D435i RGB-D camera | `/d435i/color/image_raw`, `/d435i/color/camera_info`, `/d435i/depth/image_raw`, `/d435i/depth/points` |
| IMU | `/imu/data` |
| Velodyne 3D LiDAR | `/velodyne_points/points` |
| 4D LiDAR L1 | `/unitree_lidar/points` |
| Mono camera | `/rgb_image` |

> Note: the D435i topic bridge (`d435i_bridge.yaml`) is currently only wired into `unitree_go2_launch.py`, not `unitree_go2_launch_TI.py`.

### Tuning the gait

Gait parameters (knee orientation, walking speed/height, stance duration, etc.) live in `src/unitree_go2_ros2_jazzy/unitree_go2_sim/config/gait/gait.yaml` — see the table in the [upstream README](src/unitree_go2_ros2_jazzy/README.md#tuning-gait-parameters) for what each field does.

## Resetting a stuck simulation

```bash
pkill -f gz
pkill -f gazebo
cd /home/ros2_ws
rm -rf build/unitree_go2_sim install/unitree_go2_sim
colcon build --packages-select unitree_go2_sim
source install/setup.bash
```

## Acknowledgements

Built on top of [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros), [CHAMP](https://github.com/chvmp/champ), [CHAMP Robots](https://github.com/chvmp/robots), and [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy).
