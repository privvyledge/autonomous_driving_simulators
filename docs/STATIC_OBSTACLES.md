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

With no `--rate` it publishes once and goes quiet — that is the latching
working, not a hang. The launch file passes `--rate 1.0` instead, because an
RViz `MarkerArray` display subscribes **VOLATILE** by default and therefore sees
nothing at all from a publish-once producer: it joins after the only message was
sent. A slow republish costs nothing and makes the topic visible whatever the
subscriber QoS. To echo a publish-once run by hand you must ask for the
durability explicitly:

```bash
ros2 topic echo --once --qos-durability transient_local /carla/static_obstacles
```

The node also re-extracts the set when CARLA starts a new episode (`--refresh`,
default 2 s). Without that a map reload leaves the latched topic serving
geometry from the world that no longer exists — CARLA regenerates every
environment-object id on load, so the published boxes stop corresponding to
anything and phantoms appear wherever the old geometry used to be. `world.id`
is the signal; the map *name* is unchanged by a same-map reload.

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

Arguments: `host` `port` `labels` `near` `radius` `z_band` `slim_overhead`
`slim_radius` `static_topic` `static_rate` `actor_topic` `merged_topic`
`frame_id` `rate` `dedup_radius` `markers` `launch_merger`.

`static_rate` (default 1.0) is the static publisher's republish rate; `rate`
(default 10.0) is the merger's.

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

## Overhead slimming — why the height band is not enough

The band drops `_SM_0`/`_SM_1`, but `_SM_2` is the pole **and** the arm in a
single mesh, so its box reaches the ground and survives the filter while
keeping the arm's full width. Measured on Town01:

```
transform.location     94.987  209.330    0.100   <- pole base (mesh pivot)
bounding_box.location  93.279  209.330    3.978   <- already world frame
extent                  1.917    0.220    3.877   -> x 91.36..95.20, z 0.10..7.86
```

A 3.83 m wide obstacle centred 1.7 m off the pole base, i.e. out over the
carriageway — the same false blockage the band was meant to remove.

CARLA exposes one axis-aligned box per mesh and no way to clip it to the band.
But `env.transform.location` is the mesh pivot (the pole base) and lies inside
the box footprint, so `collect()` re-anchors any object that

1. pokes above the band's top,
2. is wider than `slim_radius`, and
3. contains its own pivot in its footprint

to a `slim_radius` × `slim_radius` post at that pivot, clipped to the band —
`ext 0.30, 0.30, 1.20` at ROS `x = 94.987`. Everything else keeps its raw box.
Objects carry a `slimmed` flag and the dump reports `slimmed_count`. Tune with
`slim_overhead` / `slim_radius` in `config/static_obstacles.yaml`, or
`--slim-radius` / `--no-slim` on either script.

`env.bounding_box.location` is world-frame for environment objects — composing
`env.transform` on top of it double-counts the translation. Re-check with
`--diagnose N` on a new CARLA version; it prints both plus whether the pivot
falls inside the footprint.
