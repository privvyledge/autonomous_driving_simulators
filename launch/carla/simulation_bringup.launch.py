import os
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation
import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction
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

    carla_waypoint_following_kp_lateral = LaunchConfiguration('carla_waypoint_following_kp_lateral')
    carla_waypoint_following_ki_lateral = LaunchConfiguration('carla_waypoint_following_ki_lateral')
    carla_waypoint_following_kd_lateral = LaunchConfiguration('carla_waypoint_following_kd_lateral')
    carla_waypoint_following_kp_longitudinal = LaunchConfiguration('carla_waypoint_following_kp_longitudinal')
    carla_waypoint_following_ki_longitudinal = LaunchConfiguration('carla_waypoint_following_ki_longitudinal')
    carla_waypoint_following_kd_longitudinal = LaunchConfiguration('carla_waypoint_following_kd_longitudinal')

    # Resolve context strings
    role_name_string = role_name.perform(context)
    target_speed_string = target_speed.perform(context)
    goal_pose_string = goal_pose.perform(context)

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

    # 5. Built-in AD agent (local_planner/ad_agent) group
    carla_ad_agent_launch = Node(
        package='carla_ad_agent',
        executable='ad_agent',
        name=['carla_ad_agent_', role_name],
        output='log',
        parameters=[
            {
                'role_name': role_name,
                'avoid_risk': avoid_risk
            }
        ]
    )

    carla_waypoint_following_node = Node(
        package='carla_ad_agent',
        executable='local_planner',
        name=['carla_local_planner_', role_name],
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'role_name': role_name,
                'Kp_lateral': carla_waypoint_following_kp_lateral,
                'Ki_lateral': carla_waypoint_following_ki_lateral,
                'Kd_lateral': carla_waypoint_following_kd_lateral,
                'Kp_longitudinal': carla_waypoint_following_kp_longitudinal,
                'Ki_longitudinal': carla_waypoint_following_ki_longitudinal,
                'Kd_longitudinal': carla_waypoint_following_kd_longitudinal,
                'control_time_step': fixed_delta_seconds,
            }
        ]
    )

    if goal_pose_string.lower() != 'none':
        goal_pose_list = goal_pose_string.split(sep=',')
        goal_pose_orientation_quat = Rotation.from_euler('zyx', goal_pose_list[3:], degrees=True).as_quat().tolist()
        goal_pose_msg_string = (("{'header': {'stamp': 'now', 'frame_id': 'map'}, "
                                 "'pose': {position: {x: ") + str(goal_pose_list[0]) + ", y: " + str(goal_pose_list[1]) +
                                ", z: " + str(goal_pose_list[2]) + "}, orientation: {x: " + str(
                            goal_pose_orientation_quat[0]) +
                                ", y: " + str(goal_pose_orientation_quat[1]) + ", z: " + str(
                            goal_pose_orientation_quat[2]) +
                                ", w: " + str(goal_pose_orientation_quat[3]) + "}}}")
    else:
        goal_pose_msg_string = ""

    goal_pose_publisher_node = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                condition=LaunchConfigurationNotEquals('goal_pose', 'none'),
                output="log",
                cmd=[
                    "ros2", "topic", "pub", f"/carla/{role_name_string}/goal",
                    "geometry_msgs/msg/PoseStamped", goal_pose_msg_string, "--once",
                ],
                name='goal_pose_publisher'
            )
        ]
    )

    carla_goal_pose_relay_node = ExecuteProcess(
        output="log",
        cmd=["ros2", "run", "topic_tools", "relay", "/goal_pose", f"/carla/{role_name_string}/goal"]
    )

    built_in_driver_group = GroupAction(
        condition=IfCondition(launch_builtin_agent),
        actions=[
            carla_ad_agent_launch,
            carla_waypoint_following_node,
            goal_pose_publisher_node,
            carla_goal_pose_relay_node
        ]
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

    return [
        carla_ros_bridge_launch,
        carla_spawn_objects_launch,
        carla_target_speed_publisher_node,
        carla_waypoint_publisher_node,
        built_in_driver_group,
        carla_autoware_bridge_launch,
        carla_manual_control_launch
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

        DeclareLaunchArgument('carla_waypoint_following_kp_lateral', default_value='0.9'),
        DeclareLaunchArgument('carla_waypoint_following_ki_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kd_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kp_longitudinal', default_value='0.206'),
        DeclareLaunchArgument('carla_waypoint_following_ki_longitudinal', default_value='0.0206'),
        DeclareLaunchArgument('carla_waypoint_following_kd_longitudinal', default_value='0.515'),

        OpaqueFunction(function=launch_setup)
    ])
