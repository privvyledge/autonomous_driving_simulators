# CARLA Smoke Tests — No Autoware, No Custom Controller

Tests the CARLA server + ROS bridge + built-in global planner stack only.

---

## Self-Sufficient Simulator & Vehicle Interface Layer

The `carla-ros-bridge` service runs `simulation_bringup.launch.py`. This launch file starts the bridge node, spawns the ego vehicle, sets target speed, starts the global planner, and runs the built-in AD agent (default True) for smoke tests.

Because this layer owns the single bridge, we can run the entire simulation layer using `docker compose up carla-server carla-ros-bridge`.

---

## Test 1 — CARLA + bridge + global planner (headless, built-in driver)

### Step 1 — Start CARLA server and carla-ros-bridge

```bash
# Start CARLA server and the new self-sufficient bridge service
docker compose up carla-server carla-ros-bridge -d
```

### Step 2 — Verify topics are flowing

```bash
docker compose exec carla-ros-bridge bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /home/carla/carla_ros_ws/install/setup.bash
ros2 topic list
```

Expected topics (non-exhaustive):

```
/carla/ego_vehicle/odometry
/carla/ego_vehicle/rgb_front/image
/carla/ego_vehicle/lidar
/carla/ego_vehicle/gnss
/carla/ego_vehicle/imu
/carla/ego_vehicle/waypoints        ← global planner publishing
/carla/ego_vehicle/vehicle_status
/clock
```

### Step 3 — Confirm the vehicle is moving

Inside the same container (or using `docker compose exec`):

```bash
ros2 topic echo /carla/ego_vehicle/odometry --once
```

Re-run a few times; the `pose.pose.position` values should change as the built-in AD agent drives.

---

## Test 2 — Visualize with RViz2 (requires display / X11)

If you have a display available (native Linux, VcXsrv on Windows, or WSLg):

```bash
# On the host or from inside the container with DISPLAY set:
docker compose exec -e DISPLAY=$DISPLAY custom-nodes bash -c "
  source /opt/ros/humble/setup.bash &&
  source /home/carla/carla_ros_ws/install/setup.bash &&
  rviz2"
```

Useful displays to add in RViz:

| Topic | Display type |
|---|---|
| `/carla/ego_vehicle/rgb_front/image` | Image |
| `/carla/ego_vehicle/lidar` | PointCloud2 |
| `/carla/ego_vehicle/waypoints` | Path |
| `/carla/ego_vehicle/odometry` | Odometry |

Fixed frame: `map`

### Alternative — carla_manual_control (pygame window, needs display)

Re-launch with `view:=True`:

```bash
ros2 launch /launch/carla_bringup.launch.py \
  launch_simulator:=False \
  view:=True \
  remap_to_autoware:=False \
  town:=Town01
```

---

## Test 3 — Verify sensor data rates

```bash
docker compose exec custom-nodes bash -c "
  source /opt/ros/humble/setup.bash &&
  source /home/carla/carla_ros_ws/install/setup.bash &&
  ros2 topic hz /carla/ego_vehicle/lidar &
  ros2 topic hz /carla/ego_vehicle/rgb_front/image &
  ros2 topic hz /clock &
  wait"
```

Expected at `CARLA_FPS=20` / `fixed_delta_seconds=0.05`:
- `/clock` — ~20 Hz
- `/carla/ego_vehicle/lidar` — ~20 Hz
- `/carla/ego_vehicle/rgb_front/image` — ~20 Hz

---

## Test 4 — Custom Autonomy (MPC/Pure Pursuit)

When running custom autonomy, we must disable the built-in AD agent so it doesn't fight our controller:

### Step 1 — Start the simulation layer with built-in agent disabled

```bash
LAUNCH_BUILTIN_AGENT=False docker compose up carla-server carla-ros-bridge -d
```

### Step 2 — Start the custom-nodes container

```bash
docker compose up custom-nodes -d
```

### Step 3 — Launch the custom controller inside custom-nodes

```bash
docker compose exec custom-nodes bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /home/carla/carla_ros_ws/install/setup.bash

ros2 launch autonomous_driving_simulators custom_nodes.launch.py \
  launch_custom_controller:=True \
  custom_controller:=purepursuit
```

Verify that only one driver is active and the vehicle drives using the purepursuit controller.

---

## Test 5 — Record waypoints with the built-in agent (deterministic, no manual driving)

Goal: let the **built-in AD agent + global planner** drive the route while the
**waypoint recorder is provably running before the ego moves** — so the CSV
captures the path from the very first motion. This replaces the old pre-Compose
ritual of pressing `b` in the pygame window to hold the car, then `b` again to
release it.

The key is `launch/carla/built_in_agent.launch.py`: the agent is extracted from
`simulation_bringup.launch.py` so it can be started **on its own, last**.

> **One-time rebuild:** `built_in_agent.launch.py` is a NEW launch file, so the
> live `./launch` bind-mount alone won't expose it to `get_package_share_directory`
> (setup.py enumerates launch files at build time). Run once:
> ```bash
> docker compose build carla-ros-bridge
> ```
> Until you do, use the direct-path form in Phase 3 (it resolves via the
> `./launch:/launch` mount without a rebuild).

### Phase 1 — bring the world up STATIONARY (agent off, planner on)

```bash
LAUNCH_BUILTIN_AGENT=False START_GLOBAL_PLANNER_CARLA=True VIEW=False \
OBJECTS_DEFINITION_FILE=/config/objects_record.json \
./scripts/setup_x11_forwarding.sh --up
```

The ego spawns and sits still — nothing is commanding it yet.

### Phase 2 — arm the recorder in custom-nodes and CONFIRM it's up

```bash
docker compose -f docker-compose.yml -f docker-compose-native.override.yml \
  up -d --no-deps custom-nodes

docker compose exec custom-nodes bash -lc \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 launch autonomous_driving_simulators custom_nodes.launch.py \
     record_waypoints:=True role_name:=ego_vehicle"
```

Confirm in a separate shell **before** releasing the car:

```bash
docker compose exec custom-nodes bash -lc \
  "source ~/carla_ros_ws/install/setup.bash && ros2 node list | grep waypoint_recording_node"
```

The recorder writes `WAYPOINTS_CSV` every 1 s from `/carla/ego_vehicle/odometry`
(frame `map`).

### Phase 3 — release the ego to autonomy (car drives, recorded from t=0)

```bash
docker compose exec carla-ros-bridge bash -lc \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 launch autonomous_driving_simulators carla/built_in_agent.launch.py role_name:=ego_vehicle"
```

Before the one-time rebuild, run Phase 3 by direct path instead:

```bash
docker compose exec carla-ros-bridge bash -lc \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 launch /launch/carla/built_in_agent.launch.py role_name:=ego_vehicle"
```

Watch the first run: confirm the agent picks up the **already-published** global
route when started late (it subscribes to `/carla/ego_vehicle/waypoints`).

---

## Tear down

```bash
docker compose down
```

---

## Common issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Bridge keeps retrying connection | `carla-server` healthcheck not passed yet | Wait; it retries for 10×20 s = 3 min |
| Two drivers fighting (ego jitters/stalls) | Built-in agent and custom controller both active | Set `LAUNCH_BUILTIN_AGENT=False` when running `custom-nodes` autonomy |
| `get_package_share_directory` error | Package not installed in this container | `autonomous_driving_simulators` is baked into both `carla-ros-bridge` and `custom-nodes`; rebuild the image if missing |
| Ego vehicle not moving | Goal pose publisher fired before spawn | Re-run; 10 s timer in launch file handles this |
| `/carla/ego_vehicle/waypoints` empty | `carla_waypoint_publisher` couldn't reach CARLA | Check `host`/`port` args; default is `localhost:2000` |