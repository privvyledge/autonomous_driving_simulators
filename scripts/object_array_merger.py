#!/usr/bin/env python3
"""Merge N derived_object_msgs/ObjectArray topics into one, at a fixed rate.

Sim- and hardware-agnostic: it knows nothing about CARLA. It exists so a
downstream consumer (e.g. an obstacle-avoiding MPC) keeps a single-topic
contract while obstacles arrive from several independent producers -- the CARLA
bridge's actor stream, static level geometry, a perception stack, whatever.

What it does:
  * republishes at a fixed rate (--rate), VOLATILE, re-emitting every source's
    latest content each cycle -- so a downstream silence watchdog stays
    meaningful for the merged topic as a whole;
  * namespaces ids per source, so two producers numbering from 0 cannot collide;
  * evicts a source's objects when it goes quiet for longer than its timeout;
  * optionally drops near-duplicate objects between sources;
  * optionally keeps only objects within --ego-radius of the ego pose.

The ego-radius filter is a throughput fix, not a semantic one. A subscriber
cannot filter its way out of this cost: rclpy deserializes the whole ObjectArray
inside the executor *before* any callback runs (measured on a 1235-object Town01
array: 1.4 ms raw vs 137 ms deserialized -- about one full core at 8 Hz). The cut
only pays off if it happens here, before the outgoing message is built.

Per-source options matter more than they look:

  timeout=0   never evict. Required for a publish-once latched producer such as
              static_obstacle_publisher.py -- a plain watchdog would drop the
              light poles a second after startup and never bring them back.
  latched     subscribe TRANSIENT_LOCAL so a producer that published before this
              node started is still received. Do NOT set it for a normal
              VOLATILE producer: a TRANSIENT_LOCAL *subscription* is
              incompatible with a VOLATILE *publisher* and would receive
              nothing at all, silently.

Typical CARLA use (bridge actors + static poles -> one topic for the MPC):

    ros2 run autonomous_driving_simulators object_array_merger.py \
        --source /carla/ego_vehicle/objects \
        --source /carla/static_objects:latched,timeout=0 \
        --output /obstacles --rate 10

or, with no rebuild, straight off the bind mount. ROS is sourced only from
~/.bashrc, which a non-interactive `docker compose exec ... python3` never
reads, so source it explicitly or this dies on ModuleNotFoundError: rclpy:

    docker compose exec custom-nodes bash -c \
        "source /opt/ros/humble/setup.bash \
         && source ~/carla_ros_ws/install/setup.bash \
         && python3 /scripts/object_array_merger.py \
              --source /carla/ego_vehicle/objects \
              --source /carla/static_objects:latched,timeout=0 \
              --output /obstacles --rate 10"
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.utilities import remove_ros_args

from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# derived_object_msgs/Object.id is uint32. Split it into an 8-bit source index
# and a 24-bit per-source id so ids stay unique and their origin stays readable
# (0x02_00_00_7B == source 2, original id 123).
SOURCE_SHIFT = 24
ID_MASK = (1 << SOURCE_SHIFT) - 1
MAX_SOURCES = 1 << (32 - SOURCE_SHIFT)

# Marker colour per source index, so it is obvious in RViz which producer a box
# came from. Cycles if there are more sources than colours.
SOURCE_COLOURS = [
    (0.10, 0.60, 1.00),   # blue    — source 0 (typically the live actor stream)
    (1.00, 0.35, 0.00),   # orange  — source 1 (typically static level geometry)
    (0.20, 0.85, 0.30),   # green
    (0.90, 0.20, 0.65),   # magenta
]


class Source:
    """One input topic plus the policy for how its objects are kept alive."""

    def __init__(self, index, spec, default_timeout):
        topic, _, raw_opts = spec.partition(':')
        opts = {}
        for item in filter(None, raw_opts.split(',')):
            key, _, value = item.partition('=')
            opts[key] = value if value else True

        unknown = set(opts) - {'latched', 'timeout', 'offset'}
        if unknown:
            raise ValueError('unknown option(s) {} in --source {!r}'.format(
                sorted(unknown), spec))

        self.index = index
        self.topic = topic
        self.latched = bool(opts.get('latched', False))
        self.timeout = float(opts.get('timeout', default_timeout))
        self.offset = int(opts.get('offset', index << SOURCE_SHIFT))
        self.objects = []
        self.last_rx = None      # rclpy Time of the most recent message
        self.stale = False
        self.warned_frame = False

    def qos(self):
        return QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if self.latched
                        else QoSDurabilityPolicy.VOLATILE))

    def __str__(self):
        return '{} (idx={}, {}, timeout={})'.format(
            self.topic, self.index,
            'latched' if self.latched else 'volatile',
            'never' if self.timeout <= 0.0 else '{:g}s'.format(self.timeout))


class ObjectArrayMerger(Node):

    def __init__(self, args):
        super().__init__('object_array_merger')
        self.frame_id = args.frame_id
        self.dedup_radius = args.dedup_radius
        self.keep_stamps = not args.restamp

        self.ego_radius = args.ego_radius
        self.ego_timeout = args.ego_timeout
        self.ego_xy = None
        self.ego_rx = None       # rclpy Time of the most recent ego pose
        self.warned_no_ego = False
        self.warned_stale_ego = False

        self.sources = [Source(i, spec, args.timeout)
                        for i, spec in enumerate(args.source)]
        if len(self.sources) > MAX_SOURCES:
            raise ValueError('at most {} sources supported'.format(MAX_SOURCES))

        out_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(ObjectArray, args.output, out_qos)
        self.marker_pub = (
            self.create_publisher(MarkerArray, args.output + '/markers', out_qos)
            if args.markers else None)

        for source in self.sources:
            # Default arg binds the loop variable -- otherwise every callback
            # would close over the last source.
            self.create_subscription(
                ObjectArray, source.topic,
                lambda msg, s=source: self._on_objects(s, msg),
                source.qos())
            self.get_logger().info('source {}'.format(source))

        if self.ego_radius > 0.0:
            self.create_subscription(Odometry, args.ego_topic, self._on_ego, 10)
            self.get_logger().info(
                'ego-radius filter: keeping objects within {:g} m (planar) of '
                '{}'.format(self.ego_radius, args.ego_topic))

        self.get_logger().info(
            'publishing merged ObjectArray on {} at {:g} Hz (frame {})'.format(
                args.output, args.rate, self.frame_id))
        self.create_timer(1.0 / args.rate, self._publish)

    def _on_objects(self, source, msg):
        if (msg.header.frame_id and msg.header.frame_id != self.frame_id
                and not source.warned_frame):
            source.warned_frame = True
            # No TF transform is applied -- merging frames silently would put
            # obstacles in the wrong place, so say so loudly instead.
            self.get_logger().error(
                '{} publishes frame {!r} but the merged topic is {!r}; objects '
                'are passed through UNTRANSFORMED and will be misplaced'.format(
                    source.topic, msg.header.frame_id, self.frame_id))

        for obj in msg.objects:
            obj.id = source.offset | (obj.id & ID_MASK)
        source.objects = msg.objects
        source.last_rx = self.get_clock().now()
        if source.stale:
            source.stale = False
            self.get_logger().info('{} recovered ({} objects)'.format(
                source.topic, len(msg.objects)))

    def _on_ego(self, msg):
        if (msg.header.frame_id and msg.header.frame_id != self.frame_id
                and not self.warned_no_ego):
            # Not fatal -- the pose is only used as a filter centre -- but a
            # mismatched frame silently shifts which objects survive.
            self.get_logger().warn(
                'ego odometry is in frame {!r}, merged topic is {!r}; the '
                'radius filter assumes they coincide'.format(
                    msg.header.frame_id, self.frame_id))
        self.ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.ego_rx = self.get_clock().now()
        if self.warned_stale_ego:
            self.warned_stale_ego = False
            self.get_logger().info('ego pose recovered; radius filter active')

    def _filter_centre(self, now):
        """Centre of the ego-radius filter, or None to publish unfiltered.

        Both fallbacks are deliberate. An empty or truncated feed is
        indistinguishable downstream from a clear road ahead, so when the ego
        pose is missing or stale it is safer to ship every object and let the
        consumer's own gate do the cutting than to silently hide obstacles
        around a pose that no longer describes where the car is.
        """
        if self.ego_radius <= 0.0:
            return None
        if self.ego_xy is None:
            if not self.warned_no_ego:
                self.warned_no_ego = True
                self.get_logger().warn(
                    'no ego pose yet — publishing UNFILTERED until one arrives')
            return None
        if (self.ego_timeout > 0.0
                and (now - self.ego_rx).nanoseconds * 1e-9 > self.ego_timeout):
            if not self.warned_stale_ego:
                self.warned_stale_ego = True
                self.get_logger().warn(
                    'ego pose older than {:g}s — publishing UNFILTERED'.format(
                        self.ego_timeout))
            return None
        return self.ego_xy

    def _expire(self, source, now):
        """Drop a source's objects once it has been quiet for too long."""
        if source.timeout <= 0.0 or source.last_rx is None or not source.objects:
            return
        if (now - source.last_rx).nanoseconds * 1e-9 <= source.timeout:
            return
        self.get_logger().warn(
            '{} silent for >{:g}s — dropping its {} object(s)'.format(
                source.topic, source.timeout, len(source.objects)))
        source.objects = []
        source.stale = True

    def _publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        merged = ObjectArray()
        merged.header.frame_id = self.frame_id
        merged.header.stamp = stamp

        centre = self._filter_centre(now)
        radius_sq = self.ego_radius ** 2

        kept = []
        origin = []          # source index per kept object, for marker colouring
        for source in self.sources:
            self._expire(source, now)
            for obj in source.objects:
                if centre is not None:
                    # Planar only: height is not a plan-view dimension, and
                    # Town01's overhead geometry sits well above the ego.
                    dx = obj.pose.position.x - centre[0]
                    dy = obj.pose.position.y - centre[1]
                    if dx * dx + dy * dy > radius_sq:
                        continue
                if self.dedup_radius > 0.0 and self._is_duplicate(obj, kept):
                    continue
                if not self.keep_stamps:
                    obj.header.stamp = stamp
                obj.header.frame_id = self.frame_id
                kept.append(obj)
                origin.append(source.index)

        merged.objects = kept
        self.pub.publish(merged)
        if self.marker_pub is not None:
            self.marker_pub.publish(self._markers(kept, origin, stamp))

    def _markers(self, objects, origin, stamp):
        markers = MarkerArray()
        # Evicting a source shrinks the array; without an explicit clear, RViz
        # would keep showing the markers whose ids are no longer republished.
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (obj, src) in enumerate(zip(objects, origin)):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'merged_obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = obj.pose
            dims = list(obj.shape.dimensions) + [0.0, 0.0, 0.0]
            # CARLA's two-wheeler blueprints report a zero bounding box, and a
            # zero-scale marker is dropped by RViz without warning -- so clamp.
            m.scale.x = max(float(dims[0]), 0.1)
            m.scale.y = max(float(dims[1]), 0.1)
            m.scale.z = max(float(dims[2]), 0.1)
            r, g, b = SOURCE_COLOURS[src % len(SOURCE_COLOURS)]
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.65)
            markers.markers.append(m)
        return markers

    def _is_duplicate(self, obj, kept):
        """True if an earlier source already emitted an object at this spot.

        Earlier --source entries win, so list the most trustworthy producer
        first.
        """
        for other in kept:
            if math.dist(
                    (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z),
                    (other.pose.position.x, other.pose.position.y,
                     other.pose.position.z)) <= self.dedup_radius:
                return True
        return False


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--source', action='append', required=True, metavar='TOPIC[:OPTS]',
        help='input topic, repeatable. OPTS is a comma-separated list of '
             'latched, timeout=SECONDS (0 = never evict), offset=INT')
    parser.add_argument('--output', default='/carla/merged_obstacles')
    parser.add_argument('--markers', action='store_true',
                        help='also publish a MarkerArray on <output>/markers, '
                             'coloured by source index, for RViz')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='republish rate in Hz (default: %(default)s)')
    parser.add_argument('--timeout', type=float, default=1.0,
                        help='default per-source eviction timeout in seconds; '
                             '0 disables eviction (default: %(default)s)')
    parser.add_argument('--frame-id', default='map',
                        help='frame of the merged topic; sources publishing a '
                             'different frame are reported as an error')
    parser.add_argument('--dedup-radius', type=float, default=0.0,
                        help='drop an object within this distance of one from '
                             'an earlier source; 0 disables (default: %(default)s)')
    parser.add_argument('--ego-topic', default='/carla/ego_vehicle/odometry',
                        help='nav_msgs/Odometry giving the ego pose; only '
                             'subscribed when --ego-radius > 0 '
                             '(default: %(default)s)')
    parser.add_argument('--ego-radius', type=float, default=0.0,
                        help='keep only objects within this planar distance of '
                             'the latest ego pose; 0 disables (default: '
                             '%(default)s). Keep it comfortably larger than any '
                             "consumer's own gate so this stays a pure "
                             'throughput cut with no behavioural effect')
    parser.add_argument('--ego-timeout', type=float, default=2.0,
                        help='publish unfiltered once the ego pose is older '
                             'than this many seconds — filtering around a stale '
                             'pose hides real obstacles; 0 disables the check '
                             '(default: %(default)s)')
    parser.add_argument('--restamp', action='store_true',
                        help='overwrite each object stamp with the publish time; '
                             'by default original stamps are kept so downstream '
                             'code can still tell how stale an object is')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.rate <= 0.0:
        parser.error('--rate must be > 0')

    rclpy.init()
    node = ObjectArrayMerger(args)
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
