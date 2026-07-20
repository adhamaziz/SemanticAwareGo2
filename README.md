# SemanticAwareGo2

A ROS 2 (Jazzy) workspace for the Unitree Go2 quadruped, simulated in Gazebo Harmonic with the [CHAMP](https://github.com/chvmp/champ) gait controller. The robot model includes IMU, 2D/3D LiDAR, a mono camera, and an Intel RealSense D435i depth camera. This is the base perception/locomotion/navigation stack for a semantics-aware exploration and inspection project.

> **Status:** simulation, sensor bring-up, 2D SLAM, Nav2 navigation, frontier exploration, 3D volumetric mapping (OctoMap), ArUco marker detection, YOLO-based object detection, and a first working end-to-end SWAP behaviour loop (explore -> detect -> approach -> search -> inspect -> resume) are all working. This is "Phase A": YOLO uses pretrained COCO weights (no custom training) and a proxy object (fire extinguisher meshes, detected via visually-similar COCO classes) rather than a fine-tuned model on the real target classes. Full NBV viewpoint scoring (C3) is still a simplified rotate-and-search stand-in, not the real Q(v,m) viewpoint optimization.

## Repository layout

```
.
├── Dockerfile               # ROS 2 Jazzy Desktop + Gazebo Harmonic + Nav2 + SLAM Toolbox + OctoMap + RealSense + PyTorch/YOLO deps
├── docker-compose.yml       # dev container (X11 + GPU passthrough, bind-mounts the repo)
└── src/
    ├── unitree_go2_ros2_jazzy/
    │   ├── champ/                    # CHAMP core control library (vendored)
    │   ├── champ_base/               # CHAMP ROS 2 driver nodes (vendored)
    │   ├── champ_msgs/                # CHAMP message definitions (vendored)
    │   ├── unitree_go2_description/   # URDF/xacro, meshes, Gazebo worlds, ArUco marker textures
    │   └── unitree_go2_sim/           # Gazebo bring-up: launch files + gait/joint/link/SLAM/Nav2/explore/octomap/aruco/yolo/SWAP config
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
- No discrete GPU required — YOLO inference runs CPU-only (see the YOLO section below). A discrete NVIDIA GPU would speed up inference significantly if available, but isn't assumed anywhere in this setup.

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

Load a different world file:
```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py world:=/absolute/path/to/some_world.sdf
```

TI building world:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch_TI.py
```

Add `rviz:=true` (default) or `rviz:=false` to toggle RViz, and `gui:=false` to run Gazebo headless.

This single launch file brings up: Gazebo + robot spawn, CHAMP gait control, EKF localization, `slam_toolbox` (mapping), the full Nav2 stack (individual nodes, not `nav2_bringup`), and `pointcloud_to_laserscan`. **OctoMap, frontier exploration, ArUco detection, YOLO detection, and the SWAP state machine are separate, standalone scripts/launch files** — start them in additional terminals once the main stack is up (see their sections below).

### Teleoperation

In a second terminal (`docker compose exec unitree_go2_dev bash`, then `source install/setup.bash`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Gazebo world only (no ROS2, no robot)

Fast way to check a world file's geometry/textures/lighting without the ~20-30s overhead of the full ROS2 stack:

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

## The worlds

### `default.sdf` — three connected rooms

20m x 15m enclosed area, split into **three equal rooms along the long (X) axis** by two interior dividing walls, each with a 1.8m doorway centered on it. The robot spawns in the middle room. Built to exercise `explore_lite`'s ability to find and pass through doorways rather than just wander a single open space.

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

Also contains three `fire_extinguisher_N` meshes (the YOLO proxy objects — see below), each with a co-located ArUco marker.

### `test_room_small.sdf` — isolated 5m x 5m debug room

A small, simple room for isolating and debugging the full detect -> approach -> search -> inspect loop without the complexity/runtime of the full three-room world. Two fire extinguishers: one (`fire_extinguisher_A`) has a correctly-oriented ArUco marker (yaw computed to face the room's open center, not left at the default which points along world X/-X regardless of where the object sits); the other (`fire_extinguisher_B`) has none, deliberately, to test that the NBV search times out and resumes exploring gracefully rather than getting stuck. Robot spawns at the room's center.

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py world:=/home/ros2_ws/install/unitree_go2_description/share/unitree_go2_description/worlds/test_room_small.sdf
```

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

## 3D volumetric mapping (OctoMap)

```bash
ros2 launch unitree_go2_sim octomap_launch.py
```

Builds a live 3D voxel occupancy map directly from the raw Velodyne cloud (not the flattened 2D `/scan`). Also publishes `/projected_map`, a 2D `OccupancyGrid` slice derived from the 3D voxels within a configured height band — this is what `explore_lite` consumes by default (see below), rather than `slam_toolbox`'s flat 2D map.

| Topic | Description |
|---|---|
| `/octomap_full` | Full 3D voxel map (`octomap_msgs/Octomap`) |
| `/projected_map` | 2D `OccupancyGrid` slice, derived from the 3D map |
| `/occupied_cells_vis_array` | Occupied voxels as `visualization_msgs/MarkerArray` — works with RViz's plain `MarkerArray` display, no special plugin needed |

Config: `unitree_go2_sim/config/octomap_params.yaml`. Z-height filtering matters a lot here: `point_cloud_min_z`/`max_z` exclude points before insertion (currently `0.3`–`2.0` — high enough to filter out both the floor *and* the CHAMP legs' own sweep during the gait cycle, which otherwise get detected as phantom obstacles tracing the robot's path). One tradeoff worth knowing: this also means short objects (like the fire extinguishers, ~0.6m tall) only get voxelized from `0.3m` up — their lower third is invisible to OctoMap. `occupancy_min_z`/`max_z` control the height band used for the `/projected_map` slice.

**RViz visualization note:** `ros-jazzy-octomap-rviz-plugins`' `OccupancyGrid` display type has a known upstream packaging bug (`undefined symbol: _ZTIN7octomap13OcTreeStampedE` — missing `find_package(octomap REQUIRED)` in its own CMakeLists, reported across multiple ROS distros). Current workaround: `LD_PRELOAD=/opt/ros/jazzy/lib/liboctomap.so` set as `additional_env` on the `rviz2` node in the launch file. The `/occupied_cells_vis_array` `MarkerArray` topic works without any of this and is a reliable fallback.

## Autonomous exploration (explore_lite)

```bash
ros2 launch unitree_go2_sim explore_launch.py
```

Frontier-based exploration ([`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)) drives the robot to unexplored areas automatically via Nav2. By default, sources `/projected_map` (OctoMap-derived — see above), which requires `octomap_launch.py` to already be running.

**B1 baseline variant** (2D-only, per the thesis evaluation framework — no OctoMap in the loop, sourced from `slam_toolbox`'s `/map` instead):
```bash
ros2 launch unitree_go2_sim explore_launch.py params_file:=/home/ros2_ws/install/unitree_go2_sim/share/unitree_go2_sim/config/explore_params_b1.yaml
```
Both configs share the same tuned exploration weights (`potential_scale`, `gain_scale`, `min_frontier_size` — see below); only the map source differs, so any coverage/thoroughness difference between them isolates the 3D-vs-2D variable rather than confounding it with different tuning.

Config: `unitree_go2_sim/config/explore_params.yaml`. Frontier candidates are visualized on `/explore/frontiers` (add a `MarkerArray` display in RViz). `Ctrl+C` this launch to hand control back to teleop/manual goals without killing the rest of the stack.

### Exploration trial experiments

```bash
ros2 launch unitree_go2_sim pilot_trial_launch.py trial_name:=trial_1
```

Runs `explore_lite` and a coverage logger (`coverage_logger.py`, subscribes to the map, writes elapsed-time-vs-percent-explored to CSV) together, auto-shutting down once `explore_lite` reports no frontiers remaining (or after a 5-minute cap). Increment `trial_name` per run (`trial_1`, `trial_2`, ...) to keep separate CSVs.

Current tuning (`explore_params.yaml`), converged via a 10-trial sweep:
```yaml
potential_scale: 3.2
gain_scale: 1.2
min_frontier_size: 0.20   # down from the default 0.75 -- this was the main lever
                          # for getting exploration to actually enter small
                          # nooks/crooks instead of only chasing large open frontiers
```

## ArUco marker detection

```bash
ros2 launch unitree_go2_sim aruco_launch.py
```

Uses [`fictionlab/ros_aruco_opencv`](https://github.com/fictionlab/ros_aruco_opencv) (`jazzy` branch) rather than the POLIMI package cited in the original thesis roadmap, which only states support for ROS 2 Humble/Iron — this package has confirmed Jazzy support. Subscribes to the D435i's `/d435i/color/image_raw` feed.

| Topic | Description |
|---|---|
| `/aruco_detections` | Detected marker IDs + 6-DoF poses (`aruco_opencv_msgs/ArucoDetection`) |
| `/aruco_tracker/debug` | Annotated debug image (detection outlines + pose axes drawn on the camera feed) |
| TF frames `marker_<id>` | Published per detected marker |

Config: set directly as launch node parameters in `aruco_launch.py` (`marker_dict: "6X6_250"`, `marker_size: 0.15`). Markers are placed in the world files at varying heights, generated via OpenCV's `cv2.aruco.generateImageMarker` with dictionary `DICT_6X6_250`, stored as PNGs in `unitree_go2_description/worlds/textures/`.

**Marker material note:** use `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>` alongside the `<pbr><metal><albedo_map>` block — an unspecified (default black) diffuse color multiplicatively tints the texture to solid black regardless of the image content.

**Marker orientation note:** a flat marker plaque with `rpy="0 0 0"` faces along world **+X/-X** by default, regardless of which direction the object it's mounted on actually needs to be viewed from. When placing a marker offset from its object in the **Y** direction (as opposed to X), it will *not* automatically face the object/room correctly — compute an explicit `yaw` toward the intended viewing direction (see `test_room_small.sdf`'s `aruco_marker_0` for a worked example: `yaw = atan2(dy, dx)` where `(dx, dy)` is the direction from the marker toward wherever it should be visible from).

## YOLO object detection (Phase A)

```bash
python3 unitree_go2_sim/launch/yolo_detector.py
```

Pretrained YOLOv8n (`ultralytics`, COCO classes, **no custom training**) running CPU-only, throttled to 2Hz on 416px images to stay usable alongside the rest of the stack on a laptop without a discrete GPU. Watches the D435i color feed; on a match against `target_classes` (default: `fire hydrant`, `street light`, `traffic light` — plausible COCO classes for the fire-extinguisher-mesh proxy object; see Phase A note below), back-projects the detection center through the depth image + camera intrinsics into a 3D point, transforms to `map` frame via TF, publishes as a pose.

| Topic | Description |
|---|---|
| `/yolo/detections` | All raw detections, any class (`vision_msgs/Detection2DArray`) |
| `/yolo/target_pose` | Estimated `map`-frame position of the best-confidence match against `target_classes` |
| `/yolo/debug_image` | Annotated feed with bounding boxes + class labels + confidence, drawn via `ultralytics`' own `result.plot()` |

**Phase A vs Phase B:** this is deliberately using pretrained-only weights and accepting classification ambiguity against visually-similar real-world COCO classes (a red cylindrical mesh plausibly reads as "fire hydrant" *or* "street light" to a model that's never seen it) rather than fine-tuning a model on the actual target classes named in the thesis (fire extinguishers, panels, valves). `target_classes` is a set, not a single string, specifically to accommodate this ambiguity — any match in the set is treated as a valid detection. Fine-tuning on synthetic Gazebo-rendered data (free ground-truth labels from known object poses) is the planned Phase B upgrade, swapped in without needing to change anything else in the pipeline.

## SWAP state machine (C1) + inspection logger (C4)

```bash
python3 unitree_go2_sim/launch/swap_state_machine.py
python3 unitree_go2_sim/launch/inspection_logger.py --ros-args -p output_json:=/home/ros2_ws/src/inspection_report.json
```

The orchestrator tying everything above together, plus a structured JSON output. State flow (published on `/current_behaviour`, `std_msgs/String`):

```
B1_EXPLORE
  -> (YOLO target_pose received) -> B2_APPROACH (Nav2 goal, stop short of the raw YOLO position)
  -> B2_NBV_SEARCH (rotate in place scanning for an ArUco tag; timeout -> back to B1_EXPLORE)
  -> (ArUco found) -> B2_PRECISE_INSPECT (Nav2 goal: standoff distance + facing along the marker's own normal)
  -> B3_DWELL (hold briefly for clean detections)
  -> back to B1_EXPLORE (resumes explore_lite via /explore/resume)
```

`B2_NBV_SEARCH` is a **simplified stand-in** for the real NBV viewpoint-scoring (C3, the `Q(v,m)` function from the thesis roadmap) — it rotates and scans rather than sampling/scoring candidate viewpoints. Swappable later without touching the rest of the FSM. Rotation speed/duration must cover a full `2π` before timing out, or the search can miss a marker that happens to sit in the unswept arc — current values (`0.3 rad/s`, `25s` timeout) intentionally include margin over the `~21s` minimum.

`inspection_logger.py` runs independently of the state machine — it subscribes `/aruco_detections` directly and logs *any* marker it sees, opportunistically, regardless of what state C1 is in. Deduplicates by marker ID; writes a JSON record per unique marker with its `map`-frame position/orientation, observation count, and timestamps.

Both nodes transform camera-frame detections into `map` frame via a `camera_frame` parameter (default: `d435i_color_optical_frame`) — **not** via the incoming message's own `header.frame_id`, which reports an orphaned Gazebo-generated name that was never wired to a real broadcast TF frame (see caveats below).

## Known caveats / design notes

- **CHAMP's leg-odometry path is unavailable in this Gazebo Harmonic port** — no contact-sensor plugin publishes `/foot_contacts`, so `state_estimation_node`'s pose output is unusable. Odometry instead comes from Gazebo's own `gz-sim-odometry-publisher-system` plugin, bridged in as `/odom/gz` and fused by `footprint_to_odom_ekf` as an absolute pose (not integrated velocity) to avoid drift. `base_footprint -> base_link` is a static identity transform (not EKF-estimated) for the same reason.
- **`pointcloud_to_laserscan`'s height slice matters a lot** (`min_height`/`max_height` in `unitree_go2_launch.py`): too thin and most azimuths miss the Velodyne's sparse vertical channels entirely (walls vanish from `/scan`); too low and it clips the floor (phantom ring obstacle around the robot). Current values: `-0.05` to `1.5`.
- **A sensor's `header.frame_id` in its published messages is not reliable evidence that frame exists in the live TF tree.** Gazebo silently substitutes its own scoped `<model>/<link>/<sensor>` name (e.g. `go2/base_link/d435i_rgbd`) when a sensor's configured `gz_frame_id` isn't a recognized SDF element — and that substituted name is often never actually wired to a broadcast transform, only stamped into message headers. Always cross-check against `ros2 run tf2_tools view_frames` before trusting a `header.frame_id` for a TF lookup; both `yolo_detector.py` and `inspection_logger.py` use an explicit `camera_frame` parameter (`d435i_color_optical_frame`, confirmed live) instead of trusting the header, for exactly this reason.
- **`$(find package_name)` xacro substitutions don't work in plain `.sdf` world files** — only in `.xacro`/URDF files that go through the `xacro` preprocessor. Texture/media paths in world files need to be either relative to the world file's own directory, or absolute paths.
- **XML comments cannot contain a double-hyphen (`--`) anywhere except immediately before the closing `-->`.** An easy, silent way to break a world file's parse — use a semicolon or just avoid `--` in SDF comments entirely.
- **`ultralytics`/PyTorch pulls the full CUDA toolkit by default on Linux**, even with no GPU present — several hundred MB to over a gigabyte of unusable CUDA libraries. The Dockerfile explicitly installs the CPU-only PyTorch wheel first (`--index-url https://download.pytorch.org/whl/cpu`) so `ultralytics`'s own install sees `torch` already satisfied and doesn't replace it.
- **`ultralytics`'s dependency chain wants numpy 2.x; `cv_bridge` (compiled against ROS Jazzy's apt-installed numpy 1.x) breaks under numpy 2.x** with a "compiled using NumPy 1.x cannot be run in NumPy 2.x" `ImportError`. Fixed by installing `ultralytics` first, then explicitly re-pinning `numpy<2` afterward in the Dockerfile — order matters, pinning first gets overwritten by ultralytics' own resolution.
- **`cv_bridge.cv2_to_imgmsg` can throw `KeyError` on the write path** (`imgmsg_to_cv2` / reading is unaffected) when `ultralytics`'s pip-installed `opencv-python` coexists with the apt-installed OpenCV `cv_bridge` was compiled against — their internal type codes don't match. `yolo_detector.py`'s debug image publisher builds the `sensor_msgs/Image` message manually instead of via `cv_bridge`, sidestepping the conflict entirely.
- **The `world` launch argument in `unitree_go2_launch.py` needs to actually be wired to the `gz_args` passed to Gazebo** — it's declared as a `DeclareLaunchArgument`, but the Gazebo-launching `IncludeLaunchDescription` needs to reference `LaunchConfiguration('world')` in its `gz_args`, not a hardcoded world filename, or passing `world:=...` on the command line silently does nothing.
- **NBV/rotate-search timing must cover a full `2π`** — `angular_velocity * timeout_duration` needs to exceed `2π` radians with margin, or the search can time out having swept less than a full circle, missing a marker that happened to sit in the unswept arc.

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

Built on top of [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros), [CHAMP](https://github.com/chvmp/champ), [CHAMP Robots](https://github.com/chvmp/robots), [RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy), [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) (MIT license), [fictionlab/ros_aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv) (MIT license), and [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) (AGPL-3.0 license — pretrained weights used as-is, no modification to the library itself).
