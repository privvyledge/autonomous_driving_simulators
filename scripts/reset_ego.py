#!/usr/bin/env python3
"""Teleport the ego vehicle back to its spawn pose without restarting anything.

Re-uses the already-connected CARLA world, so the server, the map, the
carla_ros_bridge and every ROS node keep running. Intended for iterating on
controller runs (MPC / pure-pursuit) where only the vehicle state needs to go
back to the start.

`scripts/` is bind-mounted into the carla-ros-bridge container at /scripts, so:

    docker compose exec carla-ros-bridge python3 /scripts/reset_ego.py
    docker compose exec carla-ros-bridge python3 /scripts/reset_ego.py --pose 10,20,1,0,0,90

Without the mount (older containers), pipe it over stdin instead:

    docker compose exec -T carla-ros-bridge python3 - < scripts/reset_ego.py
"""
import argparse

import carla

# Same default as SPAWN_POINT in docker-compose.yml / simulation_bringup.launch.py.
# Format: x,y,z,roll,pitch,yaw (metres, degrees).
#
# NOTE this pose is in the ROS frame, because that is what consumes it:
# simulation_bringup.launch.py hands it to carla_spawn_objects as
# `spawn_point_ego_vehicle`, and carla_spawn_objects reads ROS poses (same
# frame as config/objects.json and /carla/ego_vehicle/objects). CARLA's own
# frame is left-handed, so it must be converted before use -- see ros_to_carla.
DEFAULT_POSE = '0.8798897862434387,-1.6753101348876953,4.0,-0.035736084,0.0263918489,-88.118721'


def ros_to_carla(x, y, z, roll, pitch, yaw):
    """ROS (right-handed, y left) -> CARLA (left-handed, y right).

    Mirrors carla_common.transforms: y, pitch and yaw flip sign; roll does not.
    """
    return x, -y, z, roll, -pitch, -yaw


def find_ego(world, role_name):
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == role_name:
            return actor
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--role-name', default='ego_vehicle')
    parser.add_argument('--pose', default=DEFAULT_POSE,
                        help='x,y,z,roll,pitch,yaw — defaults to the compose SPAWN_POINT')
    parser.add_argument('--frame', choices=('ros', 'carla'), default='ros',
                        help='frame --pose is expressed in (default: ros, matching SPAWN_POINT)')
    parser.add_argument('--z-offset', type=float, default=0.3,
                        help='extra height added to z so the car does not clip the road')
    parser.add_argument('--retries', type=int, default=5,
                        help='ticks to wait for the actor list to populate')
    args = parser.parse_args()

    x, y, z, roll, pitch, yaw = [float(v) for v in args.pose.split(',')]
    if args.frame == 'ros':
        x, y, z, roll, pitch, yaw = ros_to_carla(x, y, z, roll, pitch, yaw)
    transform = carla.Transform(
        carla.Location(x=x, y=y, z=z + args.z_offset),
        carla.Rotation(roll=roll, pitch=pitch, yaw=yaw))

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # A freshly-connected client holds no world snapshot until the first tick
    # reaches it, so get_actors() can come back empty even though the ego is
    # long since spawned -- this is what "no vehicle with role_name=... found"
    # usually means. In synchronous mode the bridge owns the tick, so wait for
    # one rather than calling world.tick() (that would steal the bridge's tick).
    ego = find_ego(world, args.role_name)
    for _ in range(args.retries):
        if ego is not None:
            break
        try:
            world.wait_for_tick(seconds=5.0)
        except RuntimeError:
            break
        ego = find_ego(world, args.role_name)

    if ego is None:
        seen = ['{} (role_name={!r})'.format(a.type_id, a.attributes.get('role_name'))
                for a in world.get_actors().filter('vehicle.*')]
        raise SystemExit(
            'no vehicle with role_name={} found on {}:{} (map={}).\n'
            'vehicles present: {}\n'
            'if the list is empty the ego has not spawned yet, or the simulation '
            'is not ticking (nothing to wait for).'.format(
                args.role_name, args.host, args.port, world.get_map().name,
                ', '.join(seen) or '<none>'))

    # Stop the car before moving it: a teleport keeps linear/angular velocity,
    # which makes the vehicle shoot off (and the MPC chase a bad state).
    ego.set_autopilot(False)
    ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True))
    ego.set_target_velocity(carla.Vector3D(0, 0, 0))
    ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
    ego.set_transform(transform)

    # In synchronous mode the bridge owns the tick, so just wait for the next one
    # rather than calling world.tick() ourselves (that would steal the tick).
    settings = world.get_settings()
    if settings.synchronous_mode:
        world.wait_for_tick()
    ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False))

    # Report in the ROS frame so it can be compared directly against
    # /carla/ego_vehicle/odometry and the recorded waypoint CSVs.
    tf = ego.get_transform()
    rx, ry, rz, _, _, ryaw = ros_to_carla(  # involution: carla -> ros uses the same flips
        tf.location.x, tf.location.y, tf.location.z,
        tf.rotation.roll, tf.rotation.pitch, tf.rotation.yaw)
    print('reset {} (id={}) to ros x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}'.format(
        args.role_name, ego.id, rx, ry, rz, ryaw))


if __name__ == '__main__':
    main()
