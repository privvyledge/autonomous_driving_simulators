# Static Obstacles & Object Merging — command reference

CARLA's light poles, signs, fences and walls are **baked Unreal level geometry,
not actors**. `carla_ros_bridge` builds `/carla/ego_vehicle/objects` by walking
the actor list, so none of that street furniture ever reaches ROS. A controller
subscribing only to that topic is blind to every pole on the route.

Three scripts close the gap (all in `scripts/`, bind-mounted at `/scripts`):

| Script | Role |
|---|---|
| `dump_static_obstacles.py` | one-shot JSON dump — inspection / offline use |
| `static_obstacle_publisher.py` | publishes level geometry → `/carla/static_obstacles` (latched) |
| `object_array_merger.py` | N `ObjectArray` topics → `/carla/merged_obstacles` (10 Hz) |

## Topics

| Topic | Type | QoS | Contents |
|---|---|---|---|
| `/carla/ego_vehicle/objects` | `derived_object_msgs/ObjectArray` | volatile, 20 Hz | bridge actors (vehicles, walkers) |
| `/carla/static_obstacles` | `derived_object_msgs/ObjectArray` | **latched** | level geometry (poles, signs) |
| `/carla/static_obstacles/markers` | `visualization_msgs/MarkerArray` | latched | RViz, orange |
| `/carla/merged_obstacles` | `derived_object_msgs/ObjectArray` | volatile, 10 Hz | **both — subscribe to this** |
| `/carla/merged_obstacles/markers` | `visualization_msgs/MarkerArray` | volatile, 10 Hz | RViz, blue = actors, orange = static |

Merged ids are namespaced `(source_index << 24) | original_id`, so `208` is a
bridge actor and `0x01000000 + n` is a static object.

## Two invocation gotchas

**1. ROS is not on the path under `docker compose exec`.** The images source it
only from `~/.bashrc`, which a non-interactive `exec ... python3` never reads —
so anything importing `rclpy` dies on `ModuleNotFoundError`. The services
themselves are fine because their compose `command:` sources it explicitly.
Every command below wraps with:

```bash
bash -c "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && ..."
```

**2. `--near` and `--z-band` need the `=` form.** argparse only accepts a value
beginning with `-` if it matches its negative-number regex; the comma defeats
that, so `--near -2.0,-165.0` is read as an unknown option and rejected.
Quoting does not help — argparse inspects the token, not the shell's splitting.
Write `--near=-2.0,-165.0`.

## Manual commands (no rebuild)

### Choosing which geometry counts as an obstacle

`config/static_obstacles.yaml` lists every `carla.CityObjectLabel` with a
true/false flag, plus filter defaults (`z_band`, `near`, `radius`). Both tools
read it via `--config`; any explicit flag overrides the file.

Ask the simulator what it actually exposes — authoritative for your CARLA
build and map, unlike the hardcoded list in the YAML comments:

```bash
docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py --list-labels
```

That prints each label with its object count on the current map, so it doubles
as a cheap way to find which categories are worth enabling. Then:

```bash
docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py \
    --config /config/static_obstacles.yaml | head -40
```

`--config` needs PyYAML. It is present in the ROS Python, so if the bare
interpreter lacks it, source ROS first (see the gotchas above).

### Inspect what the map actually contains

```bash
docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py \
    --labels Poles --near=-2.0,-165.0 --radius 200 | head -40
```

Town01 returns ~220 poles for that window. Add `-o /tmp/poles.json` to save.
The dump's `filters` block records exactly which labels and windows produced
it, so a saved file is self-describing.

Is `bounding_box.location` world-space or relative to `transform`? This prints
both, and settles whether the reported positions can be trusted:

```bash
docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py \
    --labels Poles --diagnose 3
```

### Publish the static geometry

```bash
docker compose exec carla-ros-bridge bash -c \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   python3 /scripts/static_obstacle_publisher.py \
     --config /config/static_obstacles.yaml --markers"
```

It publishes once and goes quiet — that is the latching working, not a hang.

### Merge

```bash
docker compose exec custom-nodes bash -c \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   python3 /scripts/object_array_merger.py \
     --source /carla/ego_vehicle/objects \
     --source /carla/static_obstacles:latched,timeout=0 \
     --output /carla/merged_obstacles --rate 10 --markers"
```

`:latched,timeout=0` is load-bearing, and both halves are per-source on purpose:

- **`latched`** makes the subscription `TRANSIENT_LOCAL` so it still receives a
  message published before the merger started. It must be set **only** on the
  latched source — a `TRANSIENT_LOCAL` subscription is QoS-incompatible with a
  `VOLATILE` publisher, so applying it to the bridge's topic would silently
  receive nothing at all.
- **`timeout=0`** disables eviction. A publish-once producer under the normal
  watchdog gets dropped a second after startup and never returns.

### Verify

```bash
docker compose exec custom-nodes bash -c \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 topic hz /carla/merged_obstacles"
```

Rate alone is not proof — an empty array still reads 10 Hz. Check both id
ranges are present:

```bash
docker compose exec custom-nodes bash -c \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 topic echo --once --flow-style /carla/merged_obstacles" \
  | grep -o 'id: [0-9][0-9]*'
```

Small values (< 256) are bridge actors; values ≥ 16777216 are static geometry.
Only one range means a source is silently missing.

## Launch file (needs one rebuild)

`ros2 run` and `Node(executable=...)` require the package to be installed;
running the scripts directly does not.

```bash
docker compose exec carla-ros-bridge bash -c \
  "source /opt/ros/humble/setup.bash && cd ~/carla_ros_ws && \
   colcon build --symlink-install --packages-select autonomous_driving_simulators"
```

Then both nodes come up together:

```bash
docker compose exec carla-ros-bridge bash -c \
  "source /opt/ros/humble/setup.bash && source ~/carla_ros_ws/install/setup.bash && \
   ros2 launch autonomous_driving_simulators carla/static_obstacles.launch.py \
     labels:='Poles TrafficSigns' z_band:=0,2.5 radius:=200.0"
```

Arguments: `host` `port` `labels` `near` `radius` `z_band` `static_topic`
`actor_topic` `merged_topic` `frame_id` `rate` `dedup_radius` `markers`
`launch_merger`.

## RViz

Add two `MarkerArray` displays, fixed frame `map`:

- `/carla/static_obstacles/markers`
- `/carla/merged_obstacles/markers` — blue = live actors, orange = static

## Height filtering — why `--z-band` matters

Town01 street lamps decompose into three components. `..._SM_0` and `..._SM_1`
are the pole (radius ≈ 0.25 m); `..._SM_2` is the **arm overhanging the road**,
whose bounding box has a ≈1.93 m radius several metres up. Fed to a planner as
a ground obstacle, that ~4 m wide box sits across the carriageway and makes the
street impassable. `--z-band=0,2.5` keeps only geometry whose vertical extent
overlaps the vehicle's height band.
