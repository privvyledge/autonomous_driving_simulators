"""Publish CARLA static level geometry and merge it with the bridge's actors.

Brings up two nodes:

  static_obstacle_publisher.py  ->  /carla/static_obstacles   (latched)
  object_array_merger.py        ->  /carla/merged_obstacles   (10 Hz, volatile)

The merged topic is what a controller should subscribe to: it carries the
bridge's live actors AND the baked level geometry (light poles, signs) that
never reaches ROS otherwise, on one contract.

Both nodes are installed through setup.py's scripts=[] list, so this launch
file needs the package to have been colcon-built at least once:

    colcon build --symlink-install --packages-select autonomous_driving_simulators

Run it inside the carla-ros-bridge container -- static_obstacle_publisher.py
needs the CARLA client:

    ros2 launch autonomous_driving_simulators carla/static_obstacles.launch.py \
        labels:="Poles TrafficSigns" z_band:=0,2.5 near:=-2.0,-165.0 radius:=200.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return cfg(name).lower() in ('true', '1', 'yes')

    publisher_args = [
        '--host', cfg('host'),
        '--port', cfg('port'),
        '--topic', cfg('static_topic'),
        '--frame-id', cfg('frame_id'),
    ]
    # config_file supplies the label selection and filter defaults; the
    # remaining arguments override it only when explicitly set, so leaving
    # labels/near/radius/z_band empty means "use whatever the YAML says".
    if cfg('config_file'):
        publisher_args += ['--config', cfg('config_file')]
    if cfg('labels'):
        publisher_args += ['--labels'] + cfg('labels').split()
    if cfg('radius'):
        publisher_args += ['--radius', cfg('radius')]
    # These take the `key=value` form deliberately: argparse rejects a value
    # starting with '-' unless it is a bare negative number, so a separate
    # token like `--near -2.0,-165.0` fails.
    if cfg('near'):
        publisher_args.append('--near={}'.format(cfg('near')))
    if cfg('z_band'):
        publisher_args.append('--z-band={}'.format(cfg('z_band')))
    if flag('markers'):
        publisher_args.append('--markers')

    merger_args = [
        '--source', cfg('actor_topic'),
        '--source', '{}:latched,timeout=0'.format(cfg('static_topic')),
        '--output', cfg('merged_topic'),
        '--frame-id', cfg('frame_id'),
        '--rate', cfg('rate'),
        '--dedup-radius', cfg('dedup_radius'),
    ]
    if flag('markers'):
        merger_args.append('--markers')

    nodes = [
        Node(
            package='autonomous_driving_simulators',
            executable='static_obstacle_publisher.py',
            name='carla_static_obstacle_publisher',
            output='screen',
            arguments=publisher_args,
        ),
    ]
    if flag('launch_merger'):
        nodes.append(Node(
            package='autonomous_driving_simulators',
            executable='object_array_merger.py',
            name='object_array_merger',
            output='screen',
            arguments=merger_args,
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument(
            'config_file', default_value='/config/static_obstacles.yaml',
            description='YAML listing every carla.CityObjectLabel and whether to '
                        'treat it as an obstacle, plus filter defaults. The host '
                        './config dir is bind-mounted at /config'),
        DeclareLaunchArgument(
            'labels', default_value='',
            description='space-separated carla.CityObjectLabel names. Empty means '
                        'use config_file; setting it overrides the YAML entirely'),
        DeclareLaunchArgument(
            'near', default_value='',
            description='ROS-frame x,y centre of the extraction window; empty falls '
                        'back to config_file'),
        DeclareLaunchArgument('radius', default_value=''),
        DeclareLaunchArgument(
            'z_band', default_value='',
            description='height window MIN,MAX. Keeps overhead lamp arms and '
                        'traffic-light gantries out of the obstacle set; empty '
                        'falls back to config_file'),
        DeclareLaunchArgument('static_topic', default_value='/carla/static_obstacles'),
        DeclareLaunchArgument('actor_topic', default_value='/carla/ego_vehicle/objects'),
        DeclareLaunchArgument(
            'merged_topic', default_value='/carla/merged_obstacles',
            description='what a controller should subscribe to'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('rate', default_value='10.0'),
        DeclareLaunchArgument(
            'dedup_radius', default_value='0.0',
            description='drop an object within this distance of one from an '
                        'earlier source; 0 disables'),
        DeclareLaunchArgument('markers', default_value='True'),
        DeclareLaunchArgument('launch_merger', default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])
