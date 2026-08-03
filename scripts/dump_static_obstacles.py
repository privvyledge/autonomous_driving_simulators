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

`scripts/` is bind-mounted into the carla-ros-bridge container at /scripts:

    docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py
    docker compose exec carla-ros-bridge python3 /scripts/dump_static_obstacles.py \
        --labels Poles TrafficSigns --near -2.0,-180.0 --radius 60 -o /config/static_obstacles.json

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


def to_ros(x, y, z, yaw_deg):
    """CARLA (left-handed, y right) -> ROS (right-handed, y left)."""
    return x, -y, z, -yaw_deg


def collect(world, labels, near=None, radius=50.0):
    """Return the level's static geometry as plain dicts, sorted along the road.

    `near` is an optional (ros_x, ros_y) filter centre. Shared with
    static_obstacle_publisher.py so both tools see an identical obstacle set.
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
            })
    objects.sort(key=lambda o: o['ros']['y'], reverse=True)
    return objects


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--labels', nargs='+', default=DEFAULT_LABELS,
                        help='carla.CityObjectLabel names (default: %(default)s)')
    parser.add_argument('--near', default=None, metavar='ROS_X,ROS_Y',
                        help='keep only objects within --radius of this ROS-frame point')
    parser.add_argument('--radius', type=float, default=50.0)
    parser.add_argument('-o', '--output', default='-',
                        help='output path, or - for stdout (default)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    near = None
    if args.near:
        near = tuple(float(v) for v in args.near.split(','))

    objects = collect(world, args.labels, near=near, radius=args.radius)
    payload = {
        'map': world.get_map().name,
        'frame_note': 'ros = objects.json / odometry frame; carla = simulator frame',
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
