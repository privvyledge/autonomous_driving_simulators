import os
import sys
from pathlib import Path
import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Setup default paths
    objects_definition_json = os.path.join(
        get_package_share_directory('autonomous_driving_simulators'),
        'config',
        'obstacles.json'
    )

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    timeout = LaunchConfiguration('timeout')
    passive = LaunchConfiguration('passive')
    fixed_delta_seconds = LaunchConfiguration('fixed_delta_seconds')
    town = LaunchConfiguration('town')
    register_all_sensors = LaunchConfiguration('register_all_sensors')
    ego_vehicle_role_name = LaunchConfiguration('ego_vehicle_role_name')
    role_name = LaunchConfiguration('role_name')
    remap_to_autoware = LaunchConfiguration('remap_to_autoware')
    objects_definition_file = LaunchConfiguration('objects_definition_file')
    spawn_point = LaunchConfiguration('spawn_point')
    target_speed = LaunchConfiguration('target_speed')
    avoid_risk = LaunchConfiguration('avoid_risk')
    goal_pose = LaunchConfiguration('goal_pose')
    start_global_planner_carla = LaunchConfiguration('start_global_planner_carla')
    launch_builtin_agent = LaunchConfiguration('launch_builtin_agent')
    launch_autoware_bridge = LaunchConfiguration('launch_autoware_bridge')
    reload_map = LaunchConfiguration('reload_map')
    view = LaunchConfiguration('view')
    launch_actuation = LaunchConfiguration('launch_actuation')
    publish_twist = LaunchConfiguration('publish_twist')
    control_loop_rate = LaunchConfiguration('control_loop_rate')
    input_msg_is_stamped = LaunchConfiguration('input_msg_is_stamped')
    teleoperate = LaunchConfiguration('teleoperate')

    carla_waypoint_following_kp_lateral = LaunchConfiguration('carla_waypoint_following_kp_lateral')
    carla_waypoint_following_ki_lateral = LaunchConfiguration('carla_waypoint_following_ki_lateral')
    carla_waypoint_following_kd_lateral = LaunchConfiguration('carla_waypoint_following_kd_lateral')
    carla_waypoint_following_kp_longitudinal = LaunchConfiguration('carla_waypoint_following_kp_longitudinal')
    carla_waypoint_following_ki_longitudinal = LaunchConfiguration('carla_waypoint_following_ki_longitudinal')
    carla_waypoint_following_kd_longitudinal = LaunchConfiguration('carla_waypoint_following_kd_longitudinal')

    # Resolve context strings
    role_name_string = role_name.perform(context)
    target_speed_string = target_speed.perform(context)

    # Resolve the town to pass to the bridge. When reload_map is False, query the
    # currently-loaded map and pass its FULL name so carla_ros_bridge skips
    # load_world() (bridge.py only reloads when get_map().name != town). This
    # avoids the heavy-map reload that crashes resource-limited CARLA servers.
    # Falls back to passing 'town' through (normal reload) if CARLA is unreachable.
    bridge_town = town
    if reload_map.perform(context).lower() in ('false', '0', 'no'):
        try:
            import carla
            _client = carla.Client(host.perform(context), int(port.perform(context)))
            _client.set_timeout(float(timeout.perform(context)))
            bridge_town = _client.get_world().get_map().name
            print("[simulation_bringup] reload_map:=False -> attaching to loaded map "
                  "'{}' (skipping load_world)".format(bridge_town))
            del _client
        except Exception as exc:  # noqa: broad - any client/connect error -> fall back
            print("[simulation_bringup] reload_map:=False but could not query the "
                  "current map ({}); passing town through (will reload)".format(exc))

    # 1. Include the CARLA ROS Bridge
    carla_ros_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autonomous_driving_simulators'),
                'launch', 'carla', 'carla_ros_bridge.launch.py'
            )
        ),
        launch_arguments={
            'host': host,
            'port': port,
            'town': bridge_town,
            'timeout': timeout,
            'synchronous_mode': 'True',
            'passive': passive,
            'fixed_delta_seconds': fixed_delta_seconds,
            'remap_to_autoware': remap_to_autoware,
            'use_sim_time': use_sim_time,
            'register_all_sensors': register_all_sensors,
            'ego_vehicle_role_name': ego_vehicle_role_name,
        }.items()
    )

    # 2. Spawn Ego Vehicle
    spawn_point_param_name = 'spawn_point_' + role_name_string
    carla_spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('carla_spawn_objects'),
                'carla_example_ego_vehicle.launch.py'
            )
        ),
        launch_arguments={
            'objects_definition_file': objects_definition_file,
            spawn_point_param_name: spawn_point,
            'spawn_point_ego_vehicle': spawn_point,
            'role_name': role_name
        }.items()
    )

    # 3. Target Speed Publisher
    topic_name = "/carla/" + role_name_string + "/target_speed"
    if target_speed_string.lower() != 'none':
        data_string = "{'data': " + target_speed_string + "}"
    else:
        data_string = ""

    carla_target_speed_publisher_node = ExecuteProcess(
        condition=LaunchConfigurationNotEquals('target_speed', 'none'),
        output="log",
        cmd=["ros2", "topic", "pub", topic_name,
             "std_msgs/msg/Float64", data_string, "--qos-durability", "transient_local"],
        name='topic_pub_target_speed'
    )

    # 4. Global Waypoint Publisher (CARLA Planner)
    carla_waypoint_publisher_node = Node(
        package='carla_waypoint_publisher',
        executable='carla_waypoint_publisher',
        name='carla_waypoint_publisher',
        condition=IfCondition(start_global_planner_carla),
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'host': host,
                'port': port,
                'timeout': timeout,
                'role_name': role_name
            }
        ]
    )

    # 5. Built-in AD agent (ad_agent + local_planner + goal publisher/relay).
    #    Extracted to built_in_agent.launch.py so it can ALSO be launched
    #    standalone AFTER the recorder is confirmed up (decoupled start), which
    #    guarantees the recorder captures the path from the first motion. Gated
    #    here by launch_builtin_agent for the bundled (one-shot) behaviour.
    built_in_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autonomous_driving_simulators'),
                'launch', 'carla', 'built_in_agent.launch.py'
            )
        ),
        condition=IfCondition(launch_builtin_agent),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'role_name': role_name,
            'avoid_risk': avoid_risk,
            'goal_pose': goal_pose,
            'fixed_delta_seconds': fixed_delta_seconds,
            'carla_waypoint_following_kp_lateral': carla_waypoint_following_kp_lateral,
            'carla_waypoint_following_ki_lateral': carla_waypoint_following_ki_lateral,
            'carla_waypoint_following_kd_lateral': carla_waypoint_following_kd_lateral,
            'carla_waypoint_following_kp_longitudinal': carla_waypoint_following_kp_longitudinal,
            'carla_waypoint_following_ki_longitudinal': carla_waypoint_following_ki_longitudinal,
            'carla_waypoint_following_kd_longitudinal': carla_waypoint_following_kd_longitudinal,
        }.items()
    )

    # 6. Optional Autoware Bridge
    carla_autoware_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('autonomous_driving_simulators'),
                'launch', 'autoware', 'carla_autoware_bridge.launch.py'
            )
        ),
        condition=IfCondition(launch_autoware_bridge),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 7. Optional manual-control pygame viewer (needs a DISPLAY + a spawned ego)
    carla_manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('carla_manual_control'),
                'carla_manual_control.launch.py'
            )
        ),
        condition=IfCondition(view),
        launch_arguments={
            'role_name': role_name
        }.items()
    )

    # 8. Low-level actuation for the custom-controller path. Converts /drive
    #    (ackermann) or /twist into a CARLA vehicle_control_cmd. These nodes
    #    (carla_ackermann_control / carla_twist_to_control) ship with the
    #    ros-bridge that is built into THIS container, so they live here rather
    #    than in custom-nodes. Gated off by default: when launch_actuation:=True
    #    set launch_builtin_agent:=False, otherwise the built-in AD agent and the
    #    actuation node both publish vehicle_control_cmd and fight for control.
    pkg_share = get_package_share_directory('autonomous_driving_simulators')

    actuation_ackermann_cmd_topic = '/drive'
    if teleoperate.perform(context).lower() == 'true':
        actuation_ackermann_cmd_topic = '/ackermann_cmd_' + role_name_string

    carla_ackermann_control_node = Node(
        package='carla_ackermann_control',
        executable='carla_ackermann_control_node',
        name='carla_ackermann_control_' + role_name_string,
        output='screen',
        condition=UnlessCondition(publish_twist),
        parameters=[
            os.path.join(pkg_share, 'config', 'PID_low_level.yaml'),
            {
                'use_sim_time': use_sim_time,
                'role_name': role_name,
                'control_loop_rate': control_loop_rate,
                'input_msg_is_stamped': input_msg_is_stamped
            }
        ],
        remappings=[
            ('/carla/' + role_name_string + '/ackermann_cmd', actuation_ackermann_cmd_topic)
        ]
    )

    carla_twist_to_control_node = Node(
        package='carla_twist_to_control',
        executable='carla_twist_to_control',
        name='carla_twist_to_control_' + role_name_string,
        output='screen',
        emulate_tty=True,
        condition=IfCondition(publish_twist),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'role_name': role_name,
                'input_msg_is_stamped': input_msg_is_stamped
            }
        ],
        remappings=[
            ('/carla/' + role_name_string + '/twist', '/twist')
        ]
    )

    actuation_group = GroupAction(
        condition=IfCondition(launch_actuation),
        actions=[carla_ackermann_control_node, carla_twist_to_control_node]
    )

    return [
        carla_ros_bridge_launch,
        carla_spawn_objects_launch,
        carla_target_speed_publisher_node,
        carla_waypoint_publisher_node,
        built_in_agent_launch,
        carla_autoware_bridge_launch,
        carla_manual_control_launch,
        actuation_group
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_driving_simulators')
    default_objects_definition = os.path.join(pkg_share, 'config', 'obstacles.json')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulated clock.'),
        DeclareLaunchArgument('host', default_value='localhost', description='IP of the CARLA server'),
        DeclareLaunchArgument('port', default_value='2000', description='TCP port of the CARLA server'),
        DeclareLaunchArgument('timeout', default_value='40', description='Connection timeout'),
        DeclareLaunchArgument('passive', default_value='False', description='Passive bridge mode'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05', description='Simulation step size'),
        DeclareLaunchArgument('town', default_value='Town01', description='CARLA town/map'),
        DeclareLaunchArgument('register_all_sensors', default_value='True', description='Register all sensors'),
        DeclareLaunchArgument('ego_vehicle_role_name', default_value="['hero', 'ego_vehicle', 'hero0', 'hero1', 'hero2', 'hero3', 'hero4', 'hero5', 'hero6', 'hero7', 'hero8', 'hero9']", description='Ego role names'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle', description='Role name'),
        DeclareLaunchArgument('remap_to_autoware', default_value='False', description='Remap sensors to Autoware topics'),
        DeclareLaunchArgument('objects_definition_file', default_value=default_objects_definition, description='Objects definition json file'),
        DeclareLaunchArgument('spawn_point', default_value='0.8798897862434387,-1.6753101348876953,4.0,-0.035736084,0.0263918489,-88.118721', description='Spawning pose. Set to "None" for a random spawn point.'),
        DeclareLaunchArgument('target_speed', default_value='8.33', description='Target speed or "none"'),
        DeclareLaunchArgument('avoid_risk', default_value='True', description='Avoid risk and obey rules'),
        DeclareLaunchArgument('goal_pose', default_value='127.4,195.4,0.0,180.0,0,0', description='Target goal pose or "none"'),
        DeclareLaunchArgument('start_global_planner_carla', default_value='True', description='Start global planner'),
        DeclareLaunchArgument('launch_builtin_agent', default_value='True', description='Launch built-in AD agent'),
        DeclareLaunchArgument('launch_autoware_bridge', default_value='False', description='Launch autoware bridge'),
        DeclareLaunchArgument('reload_map', default_value='True', description="If False, attach to the already-loaded CARLA map instead of calling load_world() (avoids heavy-map reload crashes on resource-limited servers)."),
        DeclareLaunchArgument('view', default_value='False', description='Launch the carla_manual_control pygame viewer (needs a DISPLAY and a spawned ego).'),
        DeclareLaunchArgument('launch_actuation', default_value='False', description='Run CARLA low-level actuation (carla_ackermann_control / carla_twist_to_control) in this container for the custom-controller path. Set launch_builtin_agent:=False when True so the built-in AD agent and the actuation node do not both publish vehicle_control_cmd.'),
        DeclareLaunchArgument('publish_twist', default_value='False', description='Use carla_twist_to_control (Twist input) instead of carla_ackermann_control (AckermannDrive input) for actuation.'),
        DeclareLaunchArgument('control_loop_rate', default_value='0.05', description='carla_ackermann_control loop rate in seconds.'),
        DeclareLaunchArgument('input_msg_is_stamped', default_value='True', description='Whether the actuation input control messages are stamped.'),
        DeclareLaunchArgument('teleoperate', default_value='False', description='When True, actuation subscribes to /ackermann_cmd_<role> (ackermann_mux output) instead of /drive.'),

        DeclareLaunchArgument('carla_waypoint_following_kp_lateral', default_value='0.9'),
        DeclareLaunchArgument('carla_waypoint_following_ki_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kd_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kp_longitudinal', default_value='0.206'),
        DeclareLaunchArgument('carla_waypoint_following_ki_longitudinal', default_value='0.0206'),
        DeclareLaunchArgument('carla_waypoint_following_kd_longitudinal', default_value='0.515'),

        OpaqueFunction(function=launch_setup)
    ])
