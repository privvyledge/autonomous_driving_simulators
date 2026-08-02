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

# Same default as SPAWN_POINT in docker-compose.yml / simulation_bringup.launch.py
# Format: x,y,z,roll,pitch,yaw (metres, degrees)
DEFAULT_POSE = '0.8798897862434387,-1.6753101348876953,4.0,-0.035736084,0.0263918489,-88.118721'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--role-name', default='ego_vehicle')
    parser.add_argument('--pose', default=DEFAULT_POSE,
                        help='x,y,z,roll,pitch,yaw — defaults to the compose SPAWN_POINT')
    parser.add_argument('--z-offset', type=float, default=0.3,
                        help='extra height added to z so the car does not clip the road')
    args = parser.parse_args()

    x, y, z, roll, pitch, yaw = [float(v) for v in args.pose.split(',')]
    transform = carla.Transform(
        carla.Location(x=x, y=y, z=z + args.z_offset),
        carla.Rotation(roll=roll, pitch=pitch, yaw=yaw))

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    egos = [a for a in world.get_actors().filter('vehicle.*')
            if a.attributes.get('role_name') == args.role_name]
    if not egos:
        raise SystemExit('no vehicle with role_name={} found'.format(args.role_name))
    ego = egos[0]

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

    loc = ego.get_transform().location
    print('reset {} (id={}) to x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}'.format(
        args.role_name, ego.id, loc.x, loc.y, loc.z, yaw))


if __name__ == '__main__':
    main()
