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
  * optionally drops near-duplicate objects between sources.

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

or, with no rebuild, straight off the bind mount:

    docker compose exec carla-ros-bridge python3 /scripts/object_array_merger.py ...
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

# derived_object_msgs/Object.id is uint32. Split it into an 8-bit source index
# and a 24-bit per-source id so ids stay unique and their origin stays readable
# (0x02_00_00_7B == source 2, original id 123).
SOURCE_SHIFT = 24
ID_MASK = (1 << SOURCE_SHIFT) - 1
MAX_SOURCES = 1 << (32 - SOURCE_SHIFT)


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

        for source in self.sources:
            # Default arg binds the loop variable -- otherwise every callback
            # would close over the last source.
            self.create_subscription(
                ObjectArray, source.topic,
                lambda msg, s=source: self._on_objects(s, msg),
                source.qos())
            self.get_logger().info('source {}'.format(source))

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

        kept = []
        for source in self.sources:
            self._expire(source, now)
            for obj in source.objects:
                if self.dedup_radius > 0.0 and self._is_duplicate(obj, kept):
                    continue
                if not self.keep_stamps:
                    obj.header.stamp = stamp
                obj.header.frame_id = self.frame_id
                kept.append(obj)

        merged.objects = kept
        self.pub.publish(merged)

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
    parser.add_argument('--output', default='/obstacles')
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
