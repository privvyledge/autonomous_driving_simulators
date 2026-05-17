# Autonomous Driving Simulators

A ROS2 Humble research platform that bridges **CARLA** and **AWSIM** simulators to the **Autoware** autonomous driving stack. Includes a full sensor suite, perception pipeline, multiple controllers, SLAM, and Docker Compose orchestration for each component.

| Configuration | Simulator | Status |
|---|---|---|
| `docker-compose.yml` | CARLA 0.9.15 + Autoware | Primary |
| `docker-compose.awsim.yml` | AWSIM Labs 1.1.1 + Autoware | Supported |
| Source build | CARLA or AWSIM | See below |

---

## Architecture

### Container topology (CARLA + Autoware)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Host (Ubuntu 22.04, RTX 3070, Docker network_mode: host)               │
│                                                                          │
│  ┌───────────────────┐      ┌──────────────────────┐                    │
│  │   carla-server    │      │  traffic-generator   │  (--profile)       │
│  │  CARLA UE4 binary │      │  Python + CARLA lib  │                    │
│  │  :2000/:2001/2002 │      │  generate_traffic.py │                    │
│  └────────┬──────────┘      └──────────────────────┘                    │
│           │ Python API (port 2000)                                       │
│  ┌────────▼──────────────────────────────────────────┐                  │
│  │          carla-ros-bridge                          │                  │
│  │  carla_ros_bridge  +  carla_autoware_bridge        │                  │
│  │  carla_spawn_objects / carla_waypoint_publisher    │                  │
│  │  Topic remapping: CARLA raw → Autoware sensing/*   │                  │
│  └────────┬──────────────────────────────────────────┘                  │
│           │ ROS2 DDS (CycloneDDS, ROS_DOMAIN_ID)                        │
│  ┌────────▼──────────────────────────────────────────┐                  │
│  │                   autoware                         │                  │
│  │  Perception: ground seg → clustering → tracking    │                  │
│  │  Planning:   global planner + waypoint follower    │                  │
│  │  Control:    MPC / Pure Pursuit / PID cascade      │                  │
│  │  SLAM:       RTABMap (ICP + RGBD odometry)         │                  │
│  └────────────────────────────────────────────────────┘                 │
└─────────────────────────────────────────────────────────────────────────┘
```

### AWSIM topology

```
┌──────────────────────┐     ROS2 DDS     ┌──────────────────────────┐
│        awsim         │ ────────────────► │         autoware         │
│  Unity-based sim     │                  │  (same image as above)   │
│  native Autoware     │                  └──────────────────────────┘
│  topic output        │
└──────────────────────┘
```

### Topic remapping (CARLA → Autoware)

| CARLA raw topic | Autoware sensing topic | Message type |
|---|---|---|
| `/carla/ego_vehicle/rgb_front/image` | `/sensing/camera/traffic_light/image_raw` | `sensor_msgs/Image` |
| `/carla/ego_vehicle/rgb_front/camera_info` | `/sensing/camera/traffic_light/camera_info` | `sensor_msgs/CameraInfo` |
| `/carla/ego_vehicle/lidar` | `/sensing/lidar/top/pointcloud_raw` | `sensor_msgs/PointCloud2` |
| `/carla/ego_vehicle/gnss` | `/sensing/gnss/ublox/nav_sat_fix` | `sensor_msgs/NavSatFix` |
| `/carla/ego_vehicle/imu` | `/sensing/imu/tamagawa/imu_raw` | `sensor_msgs/Imu` |

### Ego vehicle sensor suite (Tesla Model 3)

| Sensor | Spec |
|---|---|
| LiDAR | 32-channel ray cast, 50 m range, 320 000 pts/s |
| RGB camera (front) | standard + camera_info |
| Depth camera | for RGBD SLAM |
| Semantic segmentation | ground truth labels |
| GNSS | no noise |
| IMU | no noise |
| Collision + lane invasion detectors | pseudo-sensors |
| Odometry, speedometer, TF | pseudo-sensors |

---

## Prerequisites

| Requirement | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| NVIDIA driver | ≥ 525 (CUDA 12.3 compatible) |
| NVIDIA Container Toolkit | [install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) |
| Docker | ≥ 24 |
| docker compose | v2 (`docker compose`, not `docker-compose`) |
| GPU VRAM | ≥ 8 GB (RTX 3070 or better recommended) |
| Display | X11 / VNC (required for AWSIM; optional for CARLA with `-RenderOffScreen`) |

Verify NVIDIA runtime is available:

```bash
docker run --rm --runtime=nvidia nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi
```

---

## Quick Start — CARLA + Autoware

```bash
# 1. Clone the repo
git clone https://github.com/YOUR_ORG/autonomous_driving_simulators.git
cd autonomous_driving_simulators

# 2. Configure environment
cp .env.example .env
# Edit .env if needed (CARLA_TOWN, ROS_DOMAIN_ID, etc.)

# 3. Build images (first run — takes ~20–40 min; subsequent runs use cache)
docker compose build

# 4. Start the stack
docker compose up

# 5. Verify ROS2 topics are flowing (in a new terminal)
docker exec carla-ros-bridge bash -c \
  "source /opt/ros/humble/setup.bash && \
   source ~/carla_ros_ws/install/setup.bash && \
   ros2 topic list"
```

Expected topics include `/sensing/lidar/top/pointcloud_raw`, `/sensing/camera/traffic_light/image_raw`, etc.

### With dynamic traffic

```bash
docker compose --profile traffic up
```

---

## Quick Start — AWSIM + Autoware

```bash
# Allow Docker to access the host display
xhost +local:docker

# Start AWSIM + Autoware
DISPLAY=:0 docker compose -f docker-compose.awsim.yml up
```

AWSIM opens a Unity window on the host display. Autoware subscribes to its ROS2 output topics automatically via the shared host network.

---

## Building Images Individually

```bash
# Build a single service
docker compose build carla-server
docker compose build carla-ros-bridge
docker compose build autoware
docker compose build awsim            # uses docker-compose.awsim.yml
docker compose build traffic-generator

# Override build args (example: different CARLA version)
docker compose build --build-arg CARLA_VERSION=0.9.14 carla-server
```

### Build argument reference

| Service | Arg | Default |
|---|---|---|
| carla-server | `CARLA_VERSION` | `0.9.15` |
| carla-server | `CUDA_VERSION` | `12.3.2` |
| carla-ros-bridge | `CARLA_VERSION` | `0.9.15` |
| carla-ros-bridge | `ROBOTICS010_BRANCH` | `feature/add-humble-support` |
| awsim | `AWSIM_VERSION` | `1.1.1` |
| autoware | `ACADOS_NUM_THREADS` | `6` |

---

## Configuration

### Environment variables (`.env`)

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Isolates DDS traffic from other ROS2 nodes on the host |
| `CARLA_VERSION` | `0.9.15` | CARLA release tag |
| `CARLA_HOST` | `localhost` | Host/IP of the CARLA server (for bridge container) |
| `CARLA_PORT` | `2000` | RPC port |
| `CARLA_TOWN` | `Town01` | Map to load on startup |
| `CARLA_FPS` | `20` | Simulation tick rate (Hz) |
| `CARLA_QUALITY` | `Low` | Rendering quality: `Low` \| `Medium` \| `High` \| `Epic` |
| `AWSIM_VERSION` | `1.1.1` | AWSIM Labs release tag |
| `VEHICLE_ID` | `default` | Autoware vehicle configuration ID |
| `MPC_MODEL_PATH` | `/home/carla/sdks/mpc/carla` | Path inside the autoware container to compiled MPC model |
| `WAYPOINTS_CSV` | `/waypoints/waypoints.csv` | Path inside containers to pre-recorded waypoints |
| `NUM_VEHICLES` | `30` | Vehicles for traffic generator |
| `NUM_WALKERS` | `10` | Pedestrians for traffic generator |

### Local overrides (`docker-compose.override.yml`)

Copy the example and edit:

```bash
cp docker-compose.override.yml.example docker-compose.override.yml
```

docker compose merges this file automatically. Use it to enable GUI rendering, mount local source trees for development, or change map/quality settings without touching the tracked compose files.

### Volume mounts

| Host path | Container path | Purpose |
|---|---|---|
| `./config` | `/config` | YAML params and JSON sensor configs |
| `./launch` | `/launch` | ROS2 launch files |
| `./data/waypoints` | `/waypoints` | Pre-recorded waypoint CSVs |
| `./maps` | `/maps` | Autoware HD map files |
| Named `autoware-data` | `/home/carla/autoware_data` | Autoware runtime data (persistent) |

---

## Launch File Reference

### `carla_bringup.launch.py` — main CARLA entry point

Key arguments (pass as `arg_name:=value`):

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `True` | Use CARLA simulated clock |
| `launch_simulator` | `True` | Start the CARLA UE4 process |
| `town` | `Town01` | CARLA map name |
| `host` | `localhost` | CARLA server IP |
| `port` | `2000` | CARLA RPC port |
| `fixed_delta_seconds` | `0.05` | Physics timestep (1/FPS) |
| `simulation_tick_rate` | `20` | Sim FPS |
| `synchronous_mode` | `True` | Deterministic sim ticks |
| `graphics_quality` | `Low` | Rendering quality |
| `headless_rendering` | `True` | `-RenderOffScreen` flag |
| `role_name` | `ego_vehicle` | Ego actor role name |
| `goal_pose` | `127.4,195.4,0,180,0,0` | Default goal in `x,y,z,yaw,pitch,roll` |
| `teleoperate` | `False` | Enable joystick control |
| `record_waypoints` | `False` | Enable waypoint recorder |
| `launch_custom_controller` | `False` | MPC or Pure Pursuit instead of built-in |
| `custom_controller` | `mpc` | `mpc` or `purepursuit` |
| `waypoints_csv` | env `WAYPOINTS_CSV` | Path to waypoints CSV |
| `mpc_build_directory` | env `MPC_MODEL_PATH` | Path to compiled MPC model |

### `carla/carla_ros_bridge.launch.py` — bridge-only launch

| Argument | Default | Description |
|---|---|---|
| `host` | `localhost` | CARLA server host |
| `port` | `2000` | CARLA server port |
| `timeout` | `2` | Connection timeout (seconds) |
| `synchronous_mode` | `True` | Sync mode |
| `fixed_delta_seconds` | `0.05` | Physics timestep |
| `town` | `Town01` | Map |
| `use_sim_time` | `True` | Simulated clock |

### `autoware/autoware.launch.xml` — Autoware modules

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | |
| `launch_perception` | `true` | Enable perception pipeline |
| `launch_localization` | `true` | Enable localization |
| `launch_planning` | `true` | Enable planning |
| `launch_control` | `true` | Enable control |

### Detection modes (`autoware/detection.launch.xml`)

Set via `lidar_detection_model` argument:

| Mode | Description |
|---|---|
| `camera_lidar_fusion` | Camera + LiDAR euclidean clustering (default) |
| `camera_lidar_radar_fusion` | Adds radar objects |
| `lidar_radar_fusion` | LiDAR + radar, no camera |
| `lidar` | LiDAR clustering only |
| `radar` | Radar only |

---

## Sensor Configuration

### `config/objects.json` — active sensor and actor config

Defines the Tesla Model 3 ego vehicle with 13 sensors and spawns:
- 1 × Audi TT
- 1 × Tesla Cybertruck
- 1 × Firetruck
- 2 × Bicycles
- 1 × Pedestrian

### `config/obstacles.json` — collision avoidance scenario

Use this instead of `objects.json` when testing static obstacle avoidance. Pass via `carla_bringup.launch.py` as:

```bash
ros2 launch autonomous_driving_simulators carla_bringup.launch.py \
  objects_definition_file:=$(pwd)/config/obstacles.json
```

---

## Waypoint Recording and Playback

### Record waypoints

```bash
ros2 launch autonomous_driving_simulators carla_bringup.launch.py \
  record_waypoints:=True teleoperate:=True
```

Drive the vehicle manually. The recorder writes to `WAYPOINTS_CSV` (default: `/waypoints/waypoints.csv`).

### Play back waypoints

```bash
ros2 launch autonomous_driving_simulators carla_bringup.launch.py \
  launch_custom_controller:=True custom_controller:=mpc
```

The MPC controller reads the CSV and follows the recorded trajectory.

---

## Known Limitations

| Limitation | Detail |
|---|---|
| CARLA has no `odom` TF frame | CARLA's pseudo-odometry uses a custom frame. RTABMap's `map_frame_id` is set to `map` for TF compatibility. |
| AWSIM requires a display | v1.1.1 has no headless flag. Use `xhost +local:docker` and pass `DISPLAY`. |
| `autoware_perception_simple.launch.xml` | Placeholder — four nodes listed in the header comment are not yet implemented. |
| CARLA `fixed_delta_seconds` must match FPS | Set `fixed_delta_seconds = 1 / CARLA_FPS` (default 0.05 @ 20 Hz). |

---

## Troubleshooting

**CARLA healthcheck keeps failing**

```
carla-server exited with code 1
```

- Check GPU driver: `nvidia-smi`
- Verify NVIDIA runtime: `docker info | grep -i runtime`
- Port conflict: `ss -tlnp | grep 2000`

**ROS2 nodes not discovering each other**

- Ensure all containers use `network_mode: host` (set in compose)
- Check `ROS_DOMAIN_ID` is the same across containers
- Run `sudo ip link set lo multicast on` (done automatically in container startup)

**`KeyError: 'input/lidar_ml/objects'`** _(historical note — fixed)_

This was caused by a trailing space in `launch/autoware/detection.launch.xml:378`. The fix removes the space from the argument name.

**MPC model not found**

Set `MPC_MODEL_PATH` in `.env` to the directory containing the compiled Acados model (`.so` and `.json` files generated by the trajectory_following_ros2 MPC build step).

**AWSIM window does not appear**

```bash
xhost +local:docker
DISPLAY=:0 docker compose -f docker-compose.awsim.yml up
```

---

## Project Structure

```
.
├── config/                        # ROS2 parameter files
│   ├── objects.json               # Ego vehicle + sensor suite + static actors
│   ├── obstacles.json             # Obstacle avoidance scenario variant
│   ├── mpc_parameters.yaml        # MPC horizon, cost weights, vehicle model
│   ├── PID_low_level.yaml         # Speed/accel PID gains
│   ├── joy_teleop.yaml            # Joystick axis/button mapping
│   ├── mux.yaml                   # Ackermann command priority mux
│   └── *.param.yaml               # Autoware perception parameters
├── docker/
│   ├── carla-server/Dockerfile    # UE4 binary only
│   ├── carla-ros-bridge/Dockerfile# ROS bridge (no full Autoware)
│   ├── awsim/Dockerfile           # Unity simulator
│   ├── autoware/Dockerfile        # Autoware + MPC + Nav2 (no CARLA)
│   └── traffic-generator/Dockerfile# Python + CARLA client only
├── launch/
│   ├── carla_bringup.launch.py    # Main CARLA entry point (50+ params)
│   ├── autoware_perception_simple.launch.xml  # Placeholder
│   ├── autoware/                  # Autoware module launches
│   │   ├── autoware.launch.xml
│   │   ├── detection.launch.xml
│   │   ├── perception.launch.xml
│   │   ├── ground_segmentation.launch.py
│   │   └── ...
│   └── carla/
│       ├── carla_ros_bridge.launch.py
│       └── mapping.launch.py      # RTABMap SLAM
├── scripts/
│   ├── generate_traffic.py        # Spawn vehicles + walkers via CARLA API
│   ├── start_carla_simulator.py   # Subprocess wrapper for CarlaUE4.sh
│   └── CarlaUE4.sh                # CARLA launcher script
├── docs/
│   ├── CARLA_SETUP.md
│   └── AWSIMLabs_SETUP.md
├── docker-compose.yml             # CARLA + Autoware (default)
├── docker-compose.awsim.yml       # AWSIM + Autoware
├── docker-compose.override.yml.example
├── .env.example
└── package.xml                    # ROS2 package metadata
```

---

## License

| Component | License |
|---|---|
| CARLA Simulator | MIT |
| Autoware Foundation | Apache 2.0 |
| carla_ros_bridge (Robotics010 fork) | MIT |
| AWSIM Labs | Apache 2.0 |
| This repository | MIT |
