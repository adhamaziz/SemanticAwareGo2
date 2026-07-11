# SemanticAwareGo2

A ROS 2 (Jazzy) workspace for the Unitree Go2 quadruped, simulated in Gazebo Harmonic with the [CHAMP](https://github.com/chvmp/champ) gait controller. The robot model includes IMU, 2D/3D LiDAR, a mono camera, and an Intel RealSense D435i depth camera. This is the base perception/locomotion/navigation stack that future semantic-aware exploration work will build on top of.

> **Status:** simulation, sensor bring-up, 2D SLAM, Nav2 navigation, frontier exploration, 3D volumetric mapping (OctoMap), and ArUco marker detection are all working end-to-end. Next-Best-View (NBV) planning and the SWAP behaviour state machine are not implemented yet.

## Repository layout

```
.
├── Dockerfile               # ROS 2 Jazzy Desktop + Gazebo Harmonic + Nav2 + SLAM Toolbox + OctoMap + RealSense deps
├── docker-compose.yml       # dev container (X11 + GPU passthrough, bind-mounts the repo)
└── src/
    ├── unitree_go2_ros2_jazzy/
    │   ├── champ/                    # CHAMP core control library (vendored)
    │   ├── champ_base/               # CHAMP ROS 2 driver nodes (vendored)
    │   ├── champ_msgs/                # CHAMP message definitions (vendored)
    │   ├── unitree_go2_description/   # URDF/xacro, meshes, Gazebo worlds, ArUco marker textures
    │   └── unitree_go2_sim/           # Gazebo bring-up: launch files + gait/joint/link/SLAM/Nav2/explore/octomap/aruco config
    ├── m-explore-ros2/
    │   └── explore_lite/              # Frontier-based autonomous exploration (vendored, MIT license)
    └── ros_aruco_opencv/
        ├── aruco_opencv/               # ArUco detection + pose estimation node (vendored, MIT license)
        └── aruco_opencv_msgs/          # ArUco message definitions (vendored, MIT license)
```

`champ*` and the Go2 description/sim packages originate from [chvmp/champ](https://github.com/chvmp/champ) and [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy) — see [`src/unitree_go2_ros2_jazzy/README.md`](src/unitree_go2_ros2_jazzy/README.md) for the full upstream feature list, sensor visualizations, and gait-tuning reference table.

`m-explore-ros2` and `ros_aruco_opencv` are vendored (git history removed, source kept, original `LICENSE` file preserved in each folder) rather than added as git submodules, for consistency with how `champ*` is already handled above.

## Prerequisites

- Docker + Docker Compose
- Linux host with X11 (for RViz/Gazebo GUI passthrough)
- `xhost +local:root` run once per session on the host, so the container can open GUI windows

## Quick start (Docker)

```bash
# 1. Build and enter the dev container
xhost +local:root
docker compose run --rm --remove-orphans unitree_go2_dev bash

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

Default world (three-room layout with connecting doorways — see below):

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

TI building world:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch_TI.py
```

Add `rviz:=true` (default) or `rviz:=false` to toggle RViz, and `gui:=false` to run Gazebo headless.

This single launch file brings up: Gazebo + robot spawn, CHAMP gait control, EKF localization, `slam_toolbox` (mapping), the full Nav2 stack (individual nodes, not `nav2_bringup`), and `pointcloud_to_laserscan`. **OctoMap, frontier exploration, and ArUco detection are separate, standalone launch files** — start them in additional terminals once the main stack is up (see their sections below).

### Teleoperation

In a second terminal (`docker compose exec unitree_go2_dev bash`, then `source install/setup.bash`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Gazebo World

This launch the gazebo world with default.sdf without any rviz, launches.

```bash
gz sim /home/ros2_ws/src/unitree_go2_ros2_jazzy/unitree_go2_description/worlds/default.sdf
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

## The world: three connected rooms

`default.sdf` is a 20m x 15m enclosed area, split into **three equal rooms along the long (X) axis** by two interior dividing walls, each with a 1.8m doorway centered on it. The robot spawns in the middle room. This layout exists specifically to exercise `explore_lite`'s ability to find and pass through doorways into unexplored rooms, rather than just wander a single open space.

```
        room 1              room 2 (spawn)         room 3
   x: -10 .. -3.33      x: -3.33 .. 3.33        x: 3.33 .. 10
+------------------+----------------------+------------------+
|                  |                      |                  |
|   marker_1       |       (robot)        |   marker_0       |
|    (0.8m)        [door             door] (   (0.5m)         |
|                  |                      |       marker_2   |
|                  |                      |        (1.0m)    |
+------------------+----------------------+------------------+
```

Existing obstacles (`box1`-`box3`, `cylinder1`-`cylinder2`) are unchanged from the original single-room layout.

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

Nav2's `controller_server`, `planner_server`, `bt_navigator`, `behavior_server`, `smoother_server`, `waypoint_follower`, and `velocity_smoother` are launched as individual nodes (not via `nav2_bringup`'s `navigation_launch.py`, which pulls in extra servers -- `collision_monitor`, `docking_server`, `route_server` -- that this setup doesn't use and that require additional config to activate cleanly). No `amcl`/`map_server` -- localization and the map both come from `slam_toolbox` above.

Config: `unitree_go2_sim/config/nav2_params.yaml`. Send a goal via RViz's **2D Goal Pose** tool (set **Fixed Frame** to `map` first), or:

```bash
ros2 lifecycle get /controller_server   # sanity check: should read "active"
```

## 3D volumetric mapping (OctoMap)

```bash
ros2 launch unitree_go2_sim octomap_launch.py
```

Builds a live 3D voxel occupancy map directly from the raw Velodyne cloud (not the flattened 2D `/scan`). Also publishes `/projected_map`, a 2D `OccupancyGrid` slice derived from the 3D voxels within a configured height band -- this is what `explore_lite` now consumes (see below), rather than `slam_toolbox`'s flat 2D map.

| Topic | Description |
|---|---|
| `/octomap_full` | Full 3D voxel map (`octomap_msgs/Octomap`) |
| `/projected_map` | 2D `OccupancyGrid` slice, derived from the 3D map |
| `/occupied_cells_vis_array` | Occupied voxels as `visualization_msgs/MarkerArray` -- works with RViz's plain `MarkerArray` display, no special plugin needed |

Config: `unitree_go2_sim/config/octomap_params.yaml`. Z-height filtering matters a lot here: `point_cloud_min_z`/`max_z` exclude points before insertion (currently `0.3`-`2.0` -- high enough to filter out both the floor *and* the CHAMP legs' own sweep during the gait cycle, which otherwise get detected as phantom obstacles tracing the robot's path). `occupancy_min_z`/`max_z` control the height band used for the `/projected_map` slice.

**RViz visualization note:** `ros-jazzy-octomap-rviz-plugins`' `OccupancyGrid` display type has a known upstream packaging bug (`undefined symbol: _ZTIN7octomap13OcTreeStampedE` -- missing `find_package(octomap REQUIRED)` in its own CMakeLists, reported across multiple ROS distros). Current workaround: `LD_PRELOAD=/opt/ros/jazzy/lib/liboctomap.so` set as `additional_env` on the `rviz2` node in the launch file. The `/occupied_cells_vis_array` `MarkerArray` topic works without any of this and is a reliable fallback.

## Autonomous exploration (explore_lite)

```bash
ros2 launch unitree_go2_sim explore_launch.py
```

Frontier-based exploration ([`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)) drives the robot to unexplored areas automatically via Nav2. Requires `octomap_launch.py` to already be running, since it now sources `/projected_map` (see above) rather than `slam_toolbox`'s `/map`.

Config: `unitree_go2_sim/config/explore_params.yaml`. Frontier candidates are visualized on `/explore/frontiers` (add a `MarkerArray` display in RViz). `Ctrl+C` this launch to hand control back to teleop/manual goals without killing the rest of the stack.

With the three-room world above, this is a reasonable test of exploration through doorways: starting from the middle room, `explore_lite` should identify each doorway as a frontier boundary, drive through, fully explore the adjacent room, and then return to explore whichever room (or remaining space) is left.

## Autonomous exploration trial experiment

```bash
ros2 launch unitree_go2_sim pilot_trial_launch.py trial_name:=trial_x
```

To experiment the time taken for complete exploration with comparison of space coverage and time taken. Change the trial_X with trial number: 1,2,3.. etc

For current experiments, the explore_params.yaml is edited with tuning the potential scale and the gain scale, to achieve the sweet spot.

## ArUco marker detection

```bash
ros2 launch unitree_go2_sim aruco_launch.py
```

Uses [`fictionlab/ros_aruco_opencv`](https://github.com/fictionlab/ros_aruco_opencv) (`jazzy` branch) rather than the POLIMI package cited in the original thesis roadmap, which only states support for ROS 2 Humble/Iron -- this package has confirmed Jazzy support. Subscribes to the D435i's `/d435i/color/image_raw` feed.

| Topic | Description |
|---|---|
| `/aruco_detections` | Detected marker IDs + 6-DoF poses (`aruco_opencv_msgs/ArucoDetection`) |
| `/aruco_tracker/debug` | Annotated debug image (detection outlines + pose axes drawn on the camera feed) |
| TF frames `marker_<id>` | Published per detected marker, parented to the camera's own frame (Gazebo-assigned name, currently `go2/base_link/d435i_rgbd` rather than the URDF's intended `d435i_color_optical_frame` -- see caveats below) |

Config: set directly as launch node parameters in `aruco_launch.py` (`marker_dict: "6X6_250"`, `marker_size: 0.15`). Three markers (IDs 0-2) are placed in the default world at varying heights (0.5m, 0.8m, 1.0m) across the three rooms, generated via OpenCV's `cv2.aruco.generateImageMarker` with dictionary `DICT_6X6_250` and stored as PNGs in `unitree_go2_description/worlds/textures/`.

Marker material note: use `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>` alongside the `<pbr><metal><albedo_map>` block -- an unspecified (default black) diffuse color multiplicatively tints the texture to solid black regardless of the image content.

## Known caveats / design notes

- **CHAMP's leg-odometry path is unavailable in this Gazebo Harmonic port** -- no contact-sensor plugin publishes `/foot_contacts`, so `state_estimation_node`'s pose output is unusable. Odometry instead comes from Gazebo's own `gz-sim-odometry-publisher-system` plugin, bridged in as `/odom/gz` and fused by `footprint_to_odom_ekf` as an absolute pose (not integrated velocity) to avoid drift. `base_footprint -> base_link` is a static identity transform (not EKF-estimated) for the same reason.
- **`pointcloud_to_laserscan`'s height slice matters a lot** (`min_height`/`max_height` in `unitree_go2_launch.py`): too thin and most azimuths miss the Velodyne's sparse vertical channels entirely (walls vanish from `/scan`); too low and it clips the floor (phantom ring obstacle around the robot). Current values: `-0.05` to `1.5`.
- **Gazebo doesn't always honor custom `gz_frame_id`/`frame_id` values set in the URDF/xacro** for sensors -- it silently falls back to its own scoped `<model>/<link>/<sensor>` naming (e.g. `go2/base_link/d435i_rgbd` instead of the intended `d435i_color_optical_frame`). Check actual frame names via `ros2 topic echo <topic> --once` (look at the `frame_id` field) or `ros2 run tf2_tools view_frames` rather than assuming the URDF's stated name is what's actually live.
- **`$(find package_name)` xacro substitutions don't work in plain `.sdf` world files** -- only in `.xacro`/URDF files that go through the `xacro` preprocessor. Texture/media paths in world files need to be either relative to the world file's own directory, or absolute paths.

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

Built on top of [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros), [CHAMP](https://github.com/chvmp/champ), [CHAMP Robots](https://github.com/chvmp/robots), [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy), [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) (MIT license), and [fictionlab/ros_aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv) (MIT license).
