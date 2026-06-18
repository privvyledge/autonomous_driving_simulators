import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    role_name = LaunchConfiguration('role_name')
    role_name_string = role_name.perform(context)
    
    record_waypoints = LaunchConfiguration('record_waypoints')
    publish_waypoints_from_csv = LaunchConfiguration('publish_waypoints_from_csv')
    waypoints_csv_file = LaunchConfiguration('waypoints_csv_file')
    
    launch_custom_controller = LaunchConfiguration('launch_custom_controller')
    custom_controller = LaunchConfiguration('custom_controller')
    mpc_config = LaunchConfiguration('mpc_config')
    mpc_build_directory = LaunchConfiguration('mpc_build_directory')
    target_speed = LaunchConfiguration('target_speed')
    teleoperate = LaunchConfiguration('teleoperate')
    publish_twist = LaunchConfiguration('publish_twist')

    # Get configuration file paths
    pkg_share = get_package_share_directory('autonomous_driving_simulators')
    joy_config_file = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    mux_config_file = os.path.join(pkg_share, 'config', 'mux.yaml')

    # 1. Waypoint Loader Node
    waypoint_loader_node = Node(
        condition=IfCondition(publish_waypoints_from_csv),
        package='trajectory_following_ros2',
        executable='waypoint_loader',
        name='waypoint_loader_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'file_path': waypoints_csv_file},
        ],
        remappings=[
            ('/waypoint_loader/path', '/trajectory/path'),
            ('/waypoint_loader/speed', '/trajectory/speed'),
        ]
    )

    # 2. Waypoint Recorder Node
    waypoint_recording_node = Node(
        condition=IfCondition(record_waypoints),
        package='trajectory_following_ros2',
        executable='waypoint_recorder',
        name='waypoint_recording_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'file_path': waypoints_csv_file,
                'save_interval': 1.0,
                'odom_topic': f'/carla/{role_name_string}/odometry',
                'target_frame_id': 'map',
                'save_if_transform_fails': True
            }
        ]
    )

    # 3. Joystick Teleoperation & Mux Group
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(teleoperate),
        parameters=[
            joy_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        condition=IfCondition(teleoperate),
        parameters=[
            joy_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        condition=IfCondition(teleoperate),
        parameters=[mux_config_file],
        remappings=[
            ('ackermann_cmd_out', 'ackermann_drive'),
            ('ackermann_cmd', f'/ackermann_cmd_{role_name_string}')
        ]
    )

    teleop_group = GroupAction(
        actions=[joy_node, joy_teleop_node, ackermann_mux_node]
    )

    # 4. Custom Trajectory Following Controller Group
    traj_track_settings = {
        'do_mpc': {
            'mpc_toolbox': 'do_mpc',
            'horizon': '50',
            'max_iterations': '30',
            'R_diagonal': '[10., 100.]',
            'Rd_diagonal': '[100., 1000.]',
            'Q_diagonal': '[1.0, 1.0, 10.0, 0.01]',
            'Qf_diagonal': '[0.002, 0.002, 0.0001, 0.00001]',
            'distance_tolerance': '2.5',
            'speed_tolerance': '5.0',
        },
        'acados': {
            'mpc_toolbox': 'acados',
            'horizon': '20',
            'max_iterations': '30',
            'R_diagonal': '[10., 100.]',
            'Rd_diagonal': '[100., 1000.]',
            'Q_diagonal': '[1.0, 1.0, 10.0, 0.01]',
            'Qf_diagonal': '[0.002, 0.002, 0.0001, 0.00001]',
            'distance_tolerance': '10.0',
            'speed_tolerance': '5.0',
        },
        'casadi': {
            'mpc_toolbox': 'casadi',
            'horizon': '20',
            'max_iterations': '30',
            'R_diagonal': '[10., 100.]',
            'Rd_diagonal': '[100., 1000.]',
            'Q_diagonal': '[1.0, 1.0, 10.0, 0.01]',
            'Qf_diagonal': '[0.002, 0.002, 0.0001, 0.00001]',
            'distance_tolerance': '2.5',
            'speed_tolerance': '5.0',
        }
    }
    
    custom_mpc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trajectory_following_ros2'), 'launch', 'mpc.launch.py')
        ),
        condition=LaunchConfigurationEquals('custom_controller', 'mpc'),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': mpc_config,
            'robot_frame': role_name,
            'global_frame': 'map',
            'generate_mpc_model': 'True',
            'build_with_cython': 'True',
            'model_directory': mpc_build_directory,
            'frequency': '20.0',
            'sample_time': '0.05',
            'prediction_time': '1.5',
            'publish_twist_topic': publish_twist,
            'wheelbase': '2.87528',
            'max_steer': '69.99999284118222',
            'min_steer': '-69.99999284118222',
            'max_steer_rate': '360.0',
            'max_speed': '10.5',
            'min_speed': '-10.5',
            'max_accel': '3.0',
            'max_decel': '-3.0',
            'scale_cost': 'True',
            'termination_condition': '0.0001',
            'stage_cost_type': 'NONLINEAR_LS',
            'load_waypoints': 'False',
            'waypoints_csv': waypoints_csv_file,
            'ode_type': 'continuous_kinematic_coupled',
            'desired_speed': target_speed,
            'odom_topic': f'/carla/{role_name_string}/odometry',
            'ackermann_cmd_topic': '/drive',
            'twist_topic': '/twist',
            'acceleration_topic': '/accel/local',
            'path_topic': '/trajectory/path',
            'speed_topic': '/trajectory/speed',
            **traj_track_settings.get('casadi')
        }.items()
    )

    custom_purepursuit_node = Node(
        package='trajectory_following_ros2',
        executable='purepursuit',
        name=f'purepursuit_{role_name_string}',
        output='screen',
        condition=LaunchConfigurationEquals('custom_controller', 'purepursuit'),
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_frame': role_name,
                'global_frame': 'map',
                'control_rate': 20.0,
                'goal_tolerance': 5.0,
                'lookahead_distance': 9.0,
                'min_lookahead': 2.0,
                'max_lookahead': 20.0,
                'adaptive_lookahead_gain': 0.3,
                'use_adaptive_lookahead': False,
                'wheelbase': 2.87528,
                'max_steer': 69.99999284118222,
                'min_steer': -69.99999284118222,
                'max_steer_rate': 352.9411764706,
                'max_speed': 10.0,
                'speed_Kp': 2.0,
                'speed_Ki': 0.2,
                'speed_Kd': 0.0,
                'desired_speed': target_speed,
                'odom_topic': f'/carla/{role_name_string}/odometry',
                'ackermann_cmd_topic': '/drive',
                'twist_topic': '/twist',
                'acceleration_topic': '/accel/local',
                'path_topic': '/trajectory/path',
                'speed_topic': '/trajectory/speed',
                'speedup_first_lookup': True,
            }
        ]
    )

    custom_controller_group = GroupAction(
        condition=IfCondition(launch_custom_controller),
        actions=[custom_mpc_node, custom_purepursuit_node]
    )

    # NOTE: CARLA low-level actuation (carla_ackermann_control / carla_twist_to_control)
    # now runs in the carla-ros-bridge container via simulation_bringup.launch.py
    # (launch_actuation:=True), where the ros-bridge that provides those packages is
    # built. This container is CARLA-package free and only hosts the controllers/teleop.

    return [
        waypoint_loader_node,
        waypoint_recording_node,
        teleop_group,
        custom_controller_group
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_driving_simulators')
    
    # Declare Default Paths
    default_waypoints_csv = os.path.join(pkg_share, 'data', 'waypoints.csv')
    default_mpc_config = os.path.join(pkg_share, 'config', 'mpc_parameters.yaml')
    default_mpc_build_directory = os.environ.get('MPC_MODEL_PATH', str(Path.home() / 'shared_dir' / 'mpc' / 'carla'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulated clock'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle', description='The role name for the ego vehicle'),
        DeclareLaunchArgument('record_waypoints', default_value='False', description='Record vehicle odometry path to CSV'),
        DeclareLaunchArgument('publish_waypoints_from_csv', default_value='False', description='Publish waypoint path/speed from CSV'),
        DeclareLaunchArgument('waypoints_csv_file', default_value=os.environ.get('WAYPOINTS_CSV', default_waypoints_csv), description='The path to the waypoints CSV file'),
        DeclareLaunchArgument('launch_custom_controller', default_value='False', description='Start custom trajectory controller'),
        DeclareLaunchArgument('custom_controller', default_value='mpc', description='Custom controller type (mpc or purepursuit)'),
        DeclareLaunchArgument('mpc_config', default_value=default_mpc_config, description='Path to MPC parameters yaml file'),
        DeclareLaunchArgument('mpc_build_directory', default_value=default_mpc_build_directory, description='Path to compiled Acados models'),
        DeclareLaunchArgument('target_speed', default_value='10.0', description='Target speed in m/s'),
        DeclareLaunchArgument('teleoperate', default_value='False', description='Enable joystick teleoperation and Ackermann command multiplexer'),
        DeclareLaunchArgument('publish_twist', default_value='False', description='Publish Twist messages instead of Ackermann drive commands'),


        OpaqueFunction(function=launch_setup)
    ])
