#!/usr/bin/env python3
"""Publish CARLA's static level geometry (light poles, signs, fences) to ROS 2.

carla_ros_bridge builds /carla/ego_vehicle/objects by walking the *actor* list,
so it only ever contains things that were spawned -- vehicles and walkers.
Street furniture is baked into the Unreal level and is not an actor, so it never
reaches ROS at all. An obstacle-avoiding MPC that subscribes only to that topic
is blind to every light pole on the route.

This node reads the same geometry via world.get_environment_objects() and
republishes it as derived_object_msgs/ObjectArray -- the *same message type* as
/carla/ego_vehicle/objects, so an existing callback can subscribe to both and
merge them without any new parsing.

The geometry is static for the lifetime of the map, so the topic is latched
(TRANSIENT_LOCAL): it is published once and any node that starts later still
receives it. Use --rate to republish periodically instead.

`scripts/` is bind-mounted into the carla-ros-bridge container at /scripts, so
this runs with no image rebuild -- but ROS is sourced only from ~/.bashrc, which
a non-interactive `docker compose exec ... python3` never reads. Without the
explicit source it dies on `ModuleNotFoundError: No module named 'rclpy'`:

    docker compose exec carla-ros-bridge bash -c \
        "source /opt/ros/humble/setup.bash \
         && source ~/carla_ros_ws/install/setup.bash \
         && python3 /scripts/static_obstacle_publisher.py \
              --labels Poles --near=-2.0,-165.0 --radius 200 --markers"

--near needs the `=` form for negative coordinates; see dump_static_obstacles.py.

Then, from any container on the same ROS_DOMAIN_ID:

    ros2 topic echo --once --flow-style /carla/static_obstacles
"""
import argparse
import math
import os
import sys

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from derived_object_msgs.msg import Object, ObjectArray
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Share the extraction logic with the dump tool so both see the same set.
# __file__ is undefined when this is piped in over stdin (`python3 - < ...`),
# so fall back to the bind-mount path.
try:
    _HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _HERE = '/scripts'
sys.path.insert(0, _HERE)
from dump_static_obstacles import (add_filter_arguments, collect,  # noqa: E402
                                   resolve_filters)

# derived_object_msgs/Object classification constants. Matches what the bridge
# already emits on /carla/ego_vehicle/objects (4=PEDESTRIAN .. 8=MOTORCYCLE).
CLASSIFICATION_BARRIER = 10
CLASSIFICATION_SIGN = 11

LABEL_CLASSIFICATION = {
    'Poles': CLASSIFICATION_BARRIER,
    'Fences': CLASSIFICATION_BARRIER,
    'Walls': CLASSIFICATION_BARRIER,
    'GuardRail': CLASSIFICATION_BARRIER,
    'TrafficSigns': CLASSIFICATION_SIGN,
    'TrafficLight': CLASSIFICATION_SIGN,
}


def yaw_to_quaternion(yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return math.sin(half), math.cos(half)


def build_object_array(objects, frame_id, stamp):
    msg = ObjectArray()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    for src in objects:
        obj = Object()
        obj.header.frame_id = frame_id
        obj.header.stamp = stamp
        obj.id = src['id']
        obj.detection_level = Object.OBJECT_DETECTED
        obj.object_classified = True
        obj.classification = LABEL_CLASSIFICATION.get(src['label'], CLASSIFICATION_BARRIER)
        obj.classification_certainty = 255

        obj.pose.position.x = float(src['ros']['x'])
        obj.pose.position.y = float(src['ros']['y'])
        obj.pose.position.z = float(src['ros']['z'])
        qz, qw = yaw_to_quaternion(src['ros']['yaw'])
        obj.pose.orientation.z = qz
        obj.pose.orientation.w = qw

        # Full extents, matching how the bridge reports vehicle dimensions
        # (e.g. the audi.tt publishes 4.18 x 1.99, not its half-extents).
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions = [
            float(src['extent']['x']) * 2.0,
            float(src['extent']['y']) * 2.0,
            float(src['extent']['z']) * 2.0,
        ]
        # twist/accel stay zero -- this geometry never moves.
        msg.objects.append(obj)
    return msg


def build_marker_array(objects, frame_id, stamp):
    markers = MarkerArray()
    for i, src in enumerate(objects):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'carla_static_obstacles'
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(src['ros']['x'])
        m.pose.position.y = float(src['ros']['y'])
        m.pose.position.z = float(src['ros']['z'])
        qz, qw = yaw_to_quaternion(src['ros']['yaw'])
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        # Markers need a non-zero scale or RViz drops them silently.
        m.scale.x = max(float(src['extent']['x']) * 2.0, 0.05)
        m.scale.y = max(float(src['extent']['y']) * 2.0, 0.05)
        m.scale.z = max(float(src['extent']['z']) * 2.0, 0.05)
        m.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.6)
        markers.markers.append(m)
    return markers


class StaticObstaclePublisher(Node):

    def __init__(self, args):
        super().__init__('carla_static_obstacle_publisher')
        self.args = args
        self.frame_id = args.frame_id

        # Latched: the set never changes, so late-joining subscribers (an MPC
        # started after this node) must still receive it.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.object_pub = self.create_publisher(ObjectArray, args.topic, qos)
        self.marker_pub = (
            self.create_publisher(MarkerArray, args.topic + '/markers', qos)
            if args.markers else None)

        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.0)
        world = self.client.get_world()
        self.world_id = world.id

        self.filters = resolve_filters(args)
        self.objects = self.recollect(world)

        self.publish_once()
        if args.rate > 0.0:
            self.create_timer(1.0 / args.rate, self.publish_once)
        if args.refresh > 0.0:
            self.create_timer(args.refresh, self.refresh_if_world_changed)

    def recollect(self, world):
        """Extract the obstacle set from `world` and report what came back."""
        labels, near, radius, z_band, slim, max_footprint = self.filters
        dropped = []
        objects = collect(world, labels, near=near, radius=radius,
                          z_band=z_band, slim=slim,
                          max_footprint=max_footprint, dropped=dropped)
        if dropped:
            # These are meshes whose single AABB spans a whole plot -- a spline
            # fence around a garden, say -- so the box covers open road. Naming
            # them keeps the drop auditable rather than silent.
            self.get_logger().info(
                '{} object(s) dropped as bulk boxes (smaller horizontal '
                'dimension > {} m): {}'.format(
                    len(dropped), max_footprint, ', '.join(dropped[:10])))
        self.get_logger().info(
            'collected {} static objects from {} (labels: {}; z_band: {}); '
            '{} re-anchored to a {} m post because their box carried '
            'overhead geometry'.format(
                len(objects), world.get_map().name, ', '.join(labels),
                z_band if z_band else 'unfiltered — overhead geometry included',
                sum(1 for o in objects if o['slimmed']), slim))
        unslimmable = [o for o in objects if o['unslimmable']]
        if unslimmable:
            # Overhead geometry with no support to re-anchor to. Whatever these
            # are, they are wide boxes several metres up that a planner will see
            # as ground obstacles, so name them rather than burying the count.
            self.get_logger().warn(
                '{} object(s) carry overhead geometry but their mesh pivot fell '
                'outside their own footprint, so the full box was kept and may '
                'block the road: {}'.format(
                    len(unslimmable),
                    ', '.join('{} at ({:.1f}, {:.1f}) {:.2f}x{:.2f} m'.format(
                        o['name'], o['ros']['x'], o['ros']['y'],
                        o['extent']['x'] * 2.0, o['extent']['y'] * 2.0)
                        for o in unslimmable[:10])))
        if not objects:
            self.get_logger().warn(
                'no static geometry matched -- check --labels and the --near/--radius filter')
        return objects

    def refresh_if_world_changed(self):
        """Re-extract the set when CARLA has started a new episode.

        The obstacle set is a snapshot taken once at startup, which is only safe
        while the world lives. Reloading the map -- or any world reset, including
        the one carla_ros_bridge performs when RELOAD_MAP is on -- makes CARLA
        rebuild every environment object with a fresh id. The latched message
        then keeps advertising geometry from a world that no longer exists:
        measured on Town01, 281 of 283 published ids were absent from the
        reloaded world, leaving phantom boxes over the road that no dump could
        reproduce. carla.World.id changes per episode, so it is the invalidation
        signal; comparing map names is not enough, because a reload of the *same*
        map regenerates the ids just the same.
        """
        try:
            world = self.client.get_world()
            world_id = world.id
        except RuntimeError as exc:
            self.get_logger().warn(
                'cannot reach CARLA to check for a world reload: {}'.format(exc))
            return
        if world_id == self.world_id:
            return
        self.get_logger().warn(
            'CARLA episode changed ({} -> {}); the {} published object(s) belong '
            'to the previous world -- re-extracting'.format(
                self.world_id, world_id, len(self.objects)))
        self.world_id = world_id
        self.objects = self.recollect(world)
        self.publish_once()

    def publish_once(self):
        stamp = self.get_clock().now().to_msg()
        self.object_pub.publish(build_object_array(self.objects, self.frame_id, stamp))
        if self.marker_pub is not None:
            self.marker_pub.publish(build_marker_array(self.objects, self.frame_id, stamp))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    add_filter_arguments(parser)
    parser.add_argument('--topic', default='/carla/static_obstacles')
    parser.add_argument('--frame-id', default='map',
                        help='must match the bridge, which publishes objects in map')
    parser.add_argument('--rate', type=float, default=0.0,
                        help='republish rate in Hz; 0 = publish once and rely on latching')
    parser.add_argument('--refresh', type=float, default=2.0, metavar='SECONDS',
                        help='how often to check whether CARLA started a new '
                             'episode and re-extract if so; 0 disables (default: '
                             '%(default)s). Without this a map reload leaves the '
                             'latched topic serving obstacles from the dead world')
    parser.add_argument('--markers', action='store_true',
                        help='also publish a MarkerArray on <topic>/markers for RViz')
    args = parser.parse_args()

    rclpy.init()
    node = StaticObstaclePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
