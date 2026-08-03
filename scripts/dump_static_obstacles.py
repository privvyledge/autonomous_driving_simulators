#!/usr/bin/env python3
"""Dump CARLA's static level geometry (light poles, signs, fences, walls...) to JSON.

carla_ros_bridge only publishes *actors* — vehicles, walkers, and the props it
spawned itself. Street furniture such as light poles is baked into the Unreal
level, so it never reaches ROS on /carla/*/objects, and an obstacle-avoiding
MPC that only listens to that topic is blind to it.

`world.get_environment_objects(label)` exposes exactly this geometry, with a
world-frame oriented bounding box per object. It is static for the lifetime of
the map, so dump it once at startup and load it as a fixed obstacle set rather
than polling it every control cycle.

`scripts/` is bind-mounted into the carla-ros-bridge container at /scripts. This
tool needs only the CARLA client, not ROS, so it runs without sourcing anything:

    docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py
    docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py \
        --labels Poles TrafficSigns --near=-2.0,-165.0 --radius 200 \
        -o /config/static_obstacles.json

--near MUST use the `=` form whenever the coordinates are negative. Written as
`--near -2.0,-165.0` argparse sees a token starting with `-` that is not a bare
negative number (the comma defeats its negative-number matcher), treats it as an
unknown option and fails with "expected one argument". Quoting does not help:
argparse inspects the string itself, not the shell's word splitting.

Output holds both frames: `carla` (left-handed, as the simulator reports it) and
`ros` (y and yaw negated — the frame objects.json, /carla/ego_vehicle/odometry
and the recorded waypoints all use).
"""
import argparse
import json
import math

import carla

# Level geometry worth treating as an obstacle. Buildings/Vegetation are
# available too but are off-road in Town01 and would bloat the output.
DEFAULT_LABELS = ['Poles', 'TrafficSigns', 'TrafficLight', 'Fences', 'Walls', 'GuardRail']


def available_labels():
    """CityObjectLabel names the *connected* CARLA build actually exposes.

    Read from the enum at runtime rather than hardcoded, so this keeps working
    across CARLA versions that add or rename members.
    """
    return sorted(name for name in dir(carla.CityObjectLabel)
                  if not name.startswith('_')
                  and isinstance(getattr(carla.CityObjectLabel, name),
                                 carla.CityObjectLabel))


def load_config(path):
    """Read config/static_obstacles.yaml -> (labels, defaults).

    Returns the labels whose value is true, plus the `defaults:` mapping.
    """
    try:
        import yaml
    except ImportError:
        raise SystemExit(
            '--config needs PyYAML, which is not in this interpreter. Either '
            'pass --labels explicitly, or source ROS first (its Python has '
            'PyYAML): bash -c "source /opt/ros/humble/setup.bash && python3 ..."')
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    labels = [name for name, on in (data.get('labels') or {}).items() if on]
    return labels, (data.get('defaults') or {})


def to_ros(x, y, z, yaw_deg):
    """CARLA (left-handed, y right) -> ROS (right-handed, y left)."""
    return x, -y, z, -yaw_deg


def collect(world, labels, near=None, radius=50.0, z_band=None):
    """Return the level's static geometry as plain dicts, sorted along the road.

    `near` is an optional (ros_x, ros_y) filter centre. `z_band` is an optional
    (min_z, max_z) height window: an object is kept only if its vertical extent
    overlaps that band. Use it to drop overhead geometry -- a street lamp's arm
    is a ~4 m wide box several metres up, and treating that as a ground obstacle
    makes the road impassable. Shared with static_obstacle_publisher.py so both
    tools see an identical obstacle set.
    """
    objects = []
    for name in labels:
        label = getattr(carla.CityObjectLabel, name, None)
        if label is None:
            print('# skipping unknown CityObjectLabel: {}'.format(name))
            continue
        for env in world.get_environment_objects(label):
            bb = env.bounding_box
            # For environment objects the bounding box is already world-frame.
            loc, ext, rot = bb.location, bb.extent, bb.rotation
            rx, ry, rz, ryaw = to_ros(loc.x, loc.y, loc.z, rot.yaw)
            if near and math.hypot(rx - near[0], ry - near[1]) > radius:
                continue
            z_lo, z_hi = rz - ext.z, rz + ext.z
            if z_band is not None and (z_hi < z_band[0] or z_lo > z_band[1]):
                continue
            objects.append({
                # CARLA ids are 64-bit; derived_object_msgs/Object.id is uint32.
                'id': env.id & 0xFFFFFFFF,
                'name': env.name,
                'label': name,
                'carla': {
                    'x': round(loc.x, 3), 'y': round(loc.y, 3), 'z': round(loc.z, 3),
                    'yaw': round(rot.yaw, 3),
                },
                'ros': {
                    'x': round(rx, 3), 'y': round(ry, 3), 'z': round(rz, 3),
                    'yaw': round(ryaw, 3),
                },
                # Half-extents, frame-independent.
                'extent': {
                    'x': round(ext.x, 3), 'y': round(ext.y, 3), 'z': round(ext.z, 3),
                },
                # Convenient for a circular/elliptical MPC constraint.
                'radius': round(math.hypot(ext.x, ext.y), 3),
                # Vertical span, so a consumer can tell a pole from an overhead
                # lamp arm without recomputing it.
                'z_span': [round(z_lo, 3), round(z_hi, 3)],
            })
    objects.sort(key=lambda o: o['ros']['y'], reverse=True)
    return objects


def add_filter_arguments(parser):
    """Filter flags shared with static_obstacle_publisher.py.

    Every default is None so resolve_filters() can tell "not given" from an
    explicit value and let the command line win over --config.
    """
    parser.add_argument('--labels', nargs='+', default=None,
                        help='carla.CityObjectLabel names; overrides --config. '
                             'Default when neither is given: {}'.format(
                                 ' '.join(DEFAULT_LABELS)))
    parser.add_argument('--near', default=None, metavar='ROS_X,ROS_Y',
                        help='keep only objects within --radius of this ROS-frame '
                             'point. Use the = form for negative coordinates')
    parser.add_argument('--radius', type=float, default=None)
    parser.add_argument('--z-band', default=None, metavar='MIN,MAX',
                        help='keep only objects whose vertical extent overlaps '
                             'this height window, e.g. --z-band=0,2.5 to drop '
                             'overhead lamp arms and traffic-light gantries')
    parser.add_argument('--config', default=None, metavar='PATH',
                        help='YAML label selection, e.g. /config/static_obstacles.yaml. '
                             'Explicit --labels/--near/--radius/--z-band override it')


def _pair(value):
    if value is None:
        return None
    if isinstance(value, str):
        return tuple(float(v) for v in value.split(','))
    return tuple(float(v) for v in value)


def resolve_filters(args):
    """Merge --config with the command line -> (labels, near, radius, z_band)."""
    labels, defaults = ([], {})
    if args.config:
        labels, defaults = load_config(args.config)

    labels = args.labels if args.labels is not None else (labels or DEFAULT_LABELS)
    near = _pair(args.near) if args.near is not None else _pair(defaults.get('near'))
    z_band = _pair(args.z_band) if args.z_band is not None else _pair(defaults.get('z_band'))
    radius = args.radius if args.radius is not None else defaults.get('radius', 50.0)
    return labels, near, float(radius), z_band


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    add_filter_arguments(parser)
    parser.add_argument('-o', '--output', default='-',
                        help='output path, or - for stdout (default)')
    parser.add_argument('--diagnose', type=int, default=0, metavar='N',
                        help='print transform vs bounding_box for the first N '
                             'objects per label and exit, without dumping JSON')
    parser.add_argument('--list-labels', action='store_true',
                        help='print every CityObjectLabel this CARLA build exposes, '
                             'with how many objects the current map holds, then exit')
    args = parser.parse_args(argv)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    labels, near, radius, z_band = resolve_filters(args)

    if args.list_labels:
        print('CityObjectLabel members exposed by this CARLA build, with the '
              'object count on {}:'.format(world.get_map().name))
        for name in available_labels():
            try:
                count = len(world.get_environment_objects(
                    getattr(carla.CityObjectLabel, name)))
            except RuntimeError as exc:
                count = '? ({})'.format(exc)
            print('  {:<16} {}'.format(name, count))
        return

    if args.diagnose:
        # Is bounding_box.location already world-space, or is it relative to
        # env.transform? If the two differ, the boxes must be placed through
        # the transform -- and every position this tool has reported is wrong.
        for name in labels:
            label = getattr(carla.CityObjectLabel, name, None)
            if label is None:
                continue
            print('== {} =='.format(name))
            for env in world.get_environment_objects(label)[:args.diagnose]:
                t, bb = env.transform.location, env.bounding_box
                composed = carla.Location(bb.location.x, bb.location.y, bb.location.z)
                env.transform.transform(composed)   # local -> world, in place
                print('  {}\n    transform.location    {:9.3f} {:9.3f} {:9.3f}'
                      '\n    bounding_box.location {:9.3f} {:9.3f} {:9.3f}'
                      '\n    transform(bb.location){:9.3f} {:9.3f} {:9.3f}'
                      '\n    extent                {:9.3f} {:9.3f} {:9.3f}'.format(
                          env.name, t.x, t.y, t.z,
                          bb.location.x, bb.location.y, bb.location.z,
                          composed.x, composed.y, composed.z,
                          bb.extent.x, bb.extent.y, bb.extent.z))
        return

    objects = collect(world, labels, near=near, radius=radius, z_band=z_band)
    payload = {
        'map': world.get_map().name,
        'frame_note': 'ros = objects.json / odometry frame; carla = simulator frame',
        'filters': {
            'labels': labels, 'near': near, 'radius': radius, 'z_band': z_band,
        },
        'count': len(objects),
        'objects': objects,
    }
    text = json.dumps(payload, indent=2)

    if args.output == '-':
        print(text)
    else:
        with open(args.output, 'w') as handle:
            handle.write(text + '\n')
        print('wrote {} objects to {}'.format(len(objects), args.output))


if __name__ == '__main__':
    main()
