# SemanticAwareGo2

A ROS 2 (Jazzy) workspace for the Unitree Go2 quadruped, simulated in Gazebo Harmonic with the [CHAMP](https://github.com/chvmp/champ) gait controller. The robot model includes IMU, 2D/3D LiDAR, a mono camera, and an Intel RealSense D435i depth camera. This is the base perception/locomotion/navigation stack that future semantic-aware exploration work will build on top of.

> **Status:** simulation, sensor bring-up, 2D SLAM, Nav2 navigation, and frontier exploration are working end-to-end. 3D volumetric mapping (OctoMap) is in progress. Semantic perception (ArUco detection, NBV planning, etc.) is not implemented yet.

## Repository layout

```
.
├── Dockerfile               # ROS 2 Jazzy Desktop + Gazebo Harmonic + Nav2 + SLAM Toolbox + RealSense deps
├── docker-compose.yml       # dev container (X11 + GPU passthrough, bind-mounts the repo)
└── src/
    └── unitree_go2_ros2_jazzy/
        ├── champ/                    # CHAMP core control library (vendored)
        ├── champ_base/               # CHAMP ROS 2 driver nodes (vendored)
        ├── champ_msgs/                # CHAMP message definitions (vendored)
        ├── unitree_go2_description/   # URDF/xacro, meshes, Gazebo worlds
        └── unitree_go2_sim/           # Gazebo bring-up: launch files + gait/joint/link/SLAM/Nav2/explore config
    └── m-explore-ros2/
        └── explore_lite/              # Frontier-based autonomous exploration (vendored)
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
| 2D scan (from Velodyne) | `/scan` |

> Note: the D435i topic bridge (`d435i_bridge.yaml`) is currently only wired into `unitree_go2_launch.py`, not `unitree_go2_launch_TI.py`.

### Tuning the gait

Gait parameters (knee orientation, walking speed/height, stance duration, etc.) live in `src/unitree_go2_ros2_jazzy/unitree_go2_sim/config/gait/gait.yaml` — see the table in the [upstream README](src/unitree_go2_ros2_jazzy/README.md#tuning-gait-parameters) for what each field does.

## Mapping (SLAM Toolbox)

`unitree_go2_launch.py` brings up `slam_toolbox` in mapping mode by default, converting the Velodyne point cloud to a 2D `LaserScan` (`pointcloud_to_laserscan`) and building a live occupancy grid.

| Topic | Description |
|---|---|
| `/map` | Live occupancy grid (`nav_msgs/OccupancyGrid`), transient-local QoS |
| `/scan` | 2D laser scan derived from `/velodyne_points/points` |

Config: `unitree_go2_sim/config/slam_toolbox_params.yaml`. Save a snapshot of the current map:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/ros2_ws/src/go2_map --ros-args -p save_map_timeout:=5.0
```

## Navigation (Nav2)

Nav2's `controller_server`, `planner_server`, `bt_navigator`, `behavior_server`, `smoother_server`, `waypoint_follower`, and `velocity_smoother` are launched as individual nodes (not via `nav2_bringup`'s `navigation_launch.py`, which pulls in extra servers — `collision_monitor`, `docking_server`, `route_server` — that this setup doesn't use and that require additional config to activate cleanly). No `amcl`/`map_server` — localization and the map both come from `slam_toolbox` above.

Config: `unitree_go2_sim/config/nav2_params.yaml`. Send a goal via RViz's **2D Goal Pose** tool (set **Fixed Frame** to `map` first), or:

```bash
ros2 lifecycle get /controller_server   # sanity check: should read "active"
```

## Autonomous exploration (explore_lite)

Frontier-based exploration ([`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)) drives the robot to unexplored areas automatically via Nav2, using `/map` as an interim 2D baseline (OctoMap-based 3D exploration is planned — see Status above).

Run it in a second terminal, after the main sim/SLAM/Nav2 stack is up:

```bash
ros2 launch unitree_go2_sim explore_launch.py
```

Config: `unitree_go2_sim/config/explore_params.yaml`. Frontier candidates are visualized on `/explore/frontiers` (add a `MarkerArray` display in RViz). `Ctrl+C` this launch to hand control back to teleop/manual goals without killing the rest of the stack.

## Known caveats / design notes

- **CHAMP's leg-odometry path is unavailable in this Gazebo Harmonic port** — no contact-sensor plugin publishes `/foot_contacts`, so `state_estimation_node`'s pose output is unusable. Odometry instead comes from Gazebo's own `gz-sim-odometry-publisher-system` plugin, bridged in as `/odom/gz` and fused by `footprint_to_odom_ekf` as an absolute pose (not integrated velocity) to avoid drift. `base_footprint → base_link` is a static identity transform (not EKF-estimated) for the same reason.
- **`pointcloud_to_laserscan`'s height slice matters a lot** (`min_height`/`max_height` in `unitree_go2_launch.py`): too thin and most azimuths miss the Velodyne's sparse vertical channels entirely (walls vanish from `/scan`); too low and it clips the floor (phantom ring obstacle around the robot). Current values: `-0.05` to `1.5`.

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