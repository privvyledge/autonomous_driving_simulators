"""
Records Pose and Velocity information from a Carla odometry pseudosensor.

Usage: ros2 launch autonomous_driving_simulators carla_bringup.launch.py

Steps:
    1. Launch Carla
        i. Simulator
        ii. Launch Carla ROS bridge
        iii. Spawn the ego_vehicle (and other objects)
        iv. Launch the waypoint publishing node, i.e go to goal (point stabilization). Do (1-3) using carla AD demo.
        v. Launch ackermann
        vi. Launch twist
        vii. (optional) spawn other traffic/objects not in objects.json
    6. Launch the waypoint recording node
    7. Move the car manually or Autonomously using one of the following
        i. Publish the goal point for go-to-goal for more reproducable results (or click 2D Goal Pose and draw a line on Rviz).
            Set teleop and manual control to false
        ii. Teleoperate using a joystick, i.e joy_teleop
        iii. Move the car using the GUI and manual control
    8. Ctrl-C to stop recording

Todo:
    * rename to carla_bringup since it does more than just record waypoints now
    
"""
import os
from pathlib import Path
from scipy.spatial.transform import Rotation
import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, \
    SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from nav2_common.launch import RewrittenYaml, ReplaceString


def carla_shutdown_callback(_launch_context):
    shutdown_process = ExecuteProcess(
            cmd=[[
                "kill -9 $ps aux | grep Unreal"
            ]],
            output="both",
            shell=True
    )
    return [shutdown_process]


def launch_setup(context, *args, **kwargs):
    # Get package directories

    # Get launch directories

    # Setup default directories.
    carla_simulator_script_path = os.path.join(
            get_package_prefix('autonomous_driving_simulators'),  # os.environ["HOME"])
            "lib",
            "autonomous_driving_simulators",
            "CarlaUE4.sh")

    joy_teleop_config_file = os.path.join(
            get_package_share_directory('autonomous_driving_simulators'),
            'config',  # config/vehicle
            'joy_teleop.yaml'
    )

    mux_config_file = os.path.join(
            get_package_share_directory('autonomous_driving_simulators'),
            'config',
            'mux.yaml'
    )

    mpc_parameters_file = os.path.join(
            get_package_share_directory('autonomous_driving_simulators'), 'config', 'mpc_parameters.yaml')

    # mpc_model_path = os.path.join(get_package_share_directory('autonomous_driving_simulators'), 'data', 'mpc')
    mpc_model_path = '/home/carla/shared_dir/mpc/carla'

    # Setup launch configuration variables
    ''' Unreal Engine Carla parameters '''
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    carla_simulator_script = LaunchConfiguration('carla_simulator_script', default=carla_simulator_script_path)
    simulation_tick_rate = LaunchConfiguration('simulation_tick_rate', default='20')
    hardware_acceleration_driver = LaunchConfiguration('hardware_acceleration_driver', default='cuda')
    audio_passthrough = LaunchConfiguration('audio_passthrough', default='False')
    headless_rendering = LaunchConfiguration('headless_rendering', default='True')  # True
    graphics_quality = LaunchConfiguration('graphics_quality', default='Epic')

    ''' Carla ROS Bridge parameters '''
    launch_simulator = LaunchConfiguration('launch_simulator', default='True')
    host = LaunchConfiguration('host', default='localhost')
    port = LaunchConfiguration('port', default='2000')
    timeout = LaunchConfiguration('timeout', default='10')
    passive = LaunchConfiguration('passive', default='False')
    synchronous_mode = LaunchConfiguration('synchronous_mode', default='True')
    synchronous_mode_wait_for_vehicle_control_command = LaunchConfiguration(
            'synchronous_mode_wait_for_vehicle_control_command', default='False')
    fixed_delta_seconds = LaunchConfiguration('fixed_delta_seconds', default='0.05')
    town = LaunchConfiguration('town', default='Town01')
    register_all_sensors = LaunchConfiguration('register_all_sensors', default='True')
    ego_vehicle_role_name = LaunchConfiguration('ego_vehicle_role_name',
                                                default=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                                                         "hero3", "hero4", "hero5", "hero6", "hero7",
                                                         "hero8", "hero9"])
    role_name = LaunchConfiguration('role_name', default='ego_vehicle')
    spawn_point = LaunchConfiguration('spawn_point')
    target_speed = LaunchConfiguration('target_speed', default='8.33')  # in m/s
    avoid_risk = LaunchConfiguration('avoid_risk', default='True')
    sigterm_timeout = LaunchConfiguration('sigterm_timeout', default='15')
    view = LaunchConfiguration('view', default='True')

    publish_fixed_goal_pose = LaunchConfiguration('publish_fixed_goal_pose', default='True')
    # goal pose: x, y, z, yaw, pitch, roll. Default: 0.0, 0.0, 0.0, 0.0, 0.0, 0.0'
    goal_pose = LaunchConfiguration('goal_pose', default='127.4,195.4,0.0,180.0,0,0')
    start_global_planner_carla = LaunchConfiguration('start_global_planner_carla',
                                                     default='True')  # todo: either use carla global planner or waypoint publisher from CSV
    control_loop_rate = LaunchConfiguration('control_loop_rate', default='0.05')
    input_msg_is_stamped = LaunchConfiguration('input_msg_is_stamped', default='True')

    joy_config = LaunchConfiguration('joy_config', default=joy_teleop_config_file)
    mux_config = LaunchConfiguration('mux_config', default=mux_config_file)

    record_waypoints = LaunchConfiguration('record_waypoints', default='True')
    launch_custom_controller = LaunchConfiguration('launch_custom_controller', default='False')
    custom_controller = LaunchConfiguration('custom_controller', default='mpc')
    mpc_config = LaunchConfiguration('mpc_config', default=mpc_parameters_file)
    mpc_build_directory = LaunchConfiguration('mpc_build_directory', default=mpc_model_path)
    publish_twist = LaunchConfiguration('publish_twist', default='False')

    carla_waypoint_following_kp_lateral = LaunchConfiguration('carla_waypoint_following_kp_lateral', default='0.9')
    carla_waypoint_following_ki_lateral = LaunchConfiguration('carla_waypoint_following_ki_lateral', default='0.0')
    carla_waypoint_following_kd_lateral = LaunchConfiguration('carla_waypoint_following_kd_lateral', default='0.0')
    carla_waypoint_following_kp_longitudinal = LaunchConfiguration('carla_waypoint_following_kp_longitudinal',
                                                                   default='0.206')
    carla_waypoint_following_ki_longitudinal = LaunchConfiguration('carla_waypoint_following_ki_longitudinal',
                                                                   default='0.0206')
    carla_waypoint_following_kd_longitudinal = LaunchConfiguration('carla_waypoint_following_kd_longitudinal',
                                                                   default='0.515')

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            name='use_sim_time',
            default_value=use_sim_time,
            description='Use simulated clock.'
    )

    launch_simulator_la = DeclareLaunchArgument(
            name='launch_simulator',
            default_value=launch_simulator,
            description='Whether to launch the simulator. If False, make sure to start up carla somewhere else.'
    )

    carla_simulator_script_la = DeclareLaunchArgument(
            name='carla_simulator_script',
            default_value=carla_simulator_script,
            description='The path to the carla launching script.'
    )

    simulation_tick_rate_la = DeclareLaunchArgument(
            name='simulation_tick_rate',
            default_value=simulation_tick_rate,
            description='Unreal engine graphics processing rate.'
    )

    hardware_acceleration_driver_la = DeclareLaunchArgument(
            name='hardware_acceleration_driver',
            default_value=hardware_acceleration_driver,
            description='The hardware acceleration driver to use. Options: CUDA (default), Vulkan, CPU'
    )

    audio_passthrough_la = DeclareLaunchArgument(
            name='audio_passthrough',
            default_value=audio_passthrough,
            description='Whether to pass through audio.'
    )

    headless_rendering_la = DeclareLaunchArgument(
            name='headless_rendering',
            default_value=headless_rendering,
            description='Whether to load the GUI to view global game.'
    )

    graphics_quality_la = DeclareLaunchArgument(
            name='graphics_quality',
            default_value=graphics_quality,
            description='Graphics rendering quality. Options: High/Epic/Ultra or Low'
    )

    host_la = DeclareLaunchArgument(
            name='host',
            default_value=host,
            description='IP of the CARLA server'
    )

    port_la = DeclareLaunchArgument(
            name='port',
            default_value=port,
            description='TCP port of the CARLA server'
    )

    timeout_la = DeclareLaunchArgument(
            name='timeout',
            default_value=timeout,
            description='Time to wait for a successful connection to the CARLA server'
    )

    passive_la = DeclareLaunchArgument(
            name='passive',
            default_value=passive,
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
    )

    synchronous_mode_la = DeclareLaunchArgument(
            name='synchronous_mode',
            default_value=synchronous_mode,
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
    )

    synchronous_mode_wait_for_vehicle_control_command_la = DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value=synchronous_mode_wait_for_vehicle_control_command,
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode).'
                        ' Set to true for slow controllers.'
    )

    fixed_delta_seconds_la = DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value=fixed_delta_seconds,
            description='Simulation time (delta seconds) between simulation steps. '
                        'Should be the same as or close to 1 / FPS that Carla is launched with.'
    )

    town_la = DeclareLaunchArgument(
            name='town',
            default_value=town,
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
    )

    register_all_sensors_la = DeclareLaunchArgument(
            name='register_all_sensors',
            default_value=register_all_sensors,
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
    )

    role_name_la = DeclareLaunchArgument(
            'role_name',
            default_value=role_name,
            description='The role name for the ego/default vehicle spawned.')

    ego_vehicle_role_name_la = DeclareLaunchArgument(
            'ego_vehicle_role_name',
            default_value=ego_vehicle_role_name,
            description='Role names to identify ego vehicles. ')

    spawn_point_la = DeclareLaunchArgument(
            name='spawn_point',
            default_value='0.8798897862434387, -1.6753101348876953,4.0,-0.035736084,0.0263918489,-88.118721',
            description='Where to spawn the car. Make sure Z is higher than 0 else the spawning'
                        ' will fail due to collision with the ground. Default: 127.4,-195.4,2,0,0,180')

    target_speed_la = DeclareLaunchArgument(
            'target_speed',
            default_value=target_speed,
            description='Target speed in m/s.')

    avoid_risk_la = DeclareLaunchArgument(
            'avoid_risk',
            default_value=avoid_risk,
            description='Whether the ego vehicle should avoid collision and obey traffic rules.')

    sigterm_timeout_la = DeclareLaunchArgument(
            name='sigterm_timeout',
            default_value=sigterm_timeout
    )

    view_la = DeclareLaunchArgument(
            name='view',
            default_value=view
    )

    publish_fixed_goal_pose_la = DeclareLaunchArgument(
            name='publish_fixed_goal_pose',
            default_value=publish_fixed_goal_pose,
            description='Whether to publish a fixed goal pose once.'
    )

    goal_pose_la = DeclareLaunchArgument(
            name='goal_pose',
            default_value=goal_pose,
            description='The target goal pose for the ego vehicle if not teleoperating or following waypoints. '
                        'In the order: x, y, z, yaw, pitch, roll. Angles should be specified in degrees.'
    )

    start_global_planner_carla_la = DeclareLaunchArgument(
            name='start_global_planner_carla',
            default_value=start_global_planner_carla,
            description='Whether to launch Carlas global path planner and publish waypoints to the desired goal.'
    )

    control_loop_rate_la = DeclareLaunchArgument(
            name='control_loop_rate',
            default_value=control_loop_rate
    )

    input_msg_is_stamped_la = DeclareLaunchArgument(
            name='input_msg_is_stamped',
            default_value=input_msg_is_stamped,
            description='Whether the input messages are stamped (True) or not (False)'
    )

    joy_la = DeclareLaunchArgument(
            'joy_config',
            default_value=joy_config,
            description='Descriptions for joy and joy_teleop configs')

    mux_la = DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config,
            description='Descriptions for ackermann mux configs')

    record_waypoints_la = DeclareLaunchArgument(
            'record_waypoints',
            default_value=record_waypoints,
            description='Whether to record the vehicles trajectory as waypoints.')

    launch_custom_controller_la = DeclareLaunchArgument(
            'launch_custom_controller',
            default_value=launch_custom_controller,
            description='Whether to launch one of my controllers.')

    custom_controller_la = DeclareLaunchArgument(
            'custom_controller',
            default_value=custom_controller,
            description='Which custom controller to launch, purepursuit, mpc, etc.')

    mpc_config_la = DeclareLaunchArgument(
            'mpc_config',
            default_value=mpc_config,
            description='Path to MPC parameters.')

    mpc_build_directory_la = DeclareLaunchArgument(
            'mpc_build_directory',
            default_value=mpc_build_directory,
            description='The path to build/compile the MPC optimization problem.')

    publish_twist_la = DeclareLaunchArgument(
            'publish_twist',
            default_value=publish_twist,
            description='True: publish twist message, False: publish ackermann message')

    carla_waypoint_following_kp_lateral_la = DeclareLaunchArgument(
            'carla_waypoint_following_kp_lateral',
            default_value=carla_waypoint_following_kp_lateral)
    carla_waypoint_following_ki_lateral_la = DeclareLaunchArgument(
            'carla_waypoint_following_ki_lateral',
            default_value=carla_waypoint_following_ki_lateral)
    carla_waypoint_following_kd_lateral_la = DeclareLaunchArgument(
            'carla_waypoint_following_kd_lateral',
            default_value=carla_waypoint_following_kd_lateral)
    carla_waypoint_following_kp_longitudinal_la = DeclareLaunchArgument(
            'carla_waypoint_following_kp_longitudinal',
            default_value=carla_waypoint_following_kp_longitudinal)
    carla_waypoint_following_ki_longitudinal_la = DeclareLaunchArgument(
            'carla_waypoint_following_ki_longitudinal',
            default_value=carla_waypoint_following_ki_longitudinal)
    carla_waypoint_following_kd_longitudinal_la = DeclareLaunchArgument(
            'carla_waypoint_following_kd_longitudinal',
            default_value=carla_waypoint_following_kd_longitudinal)

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        launch_simulator_la,
        carla_simulator_script_la,
        simulation_tick_rate_la,
        hardware_acceleration_driver_la,
        audio_passthrough_la,
        headless_rendering_la,
        graphics_quality_la,
        host_la,
        port_la,
        timeout_la,
        passive_la,
        role_name_la,
        synchronous_mode_la,
        synchronous_mode_wait_for_vehicle_control_command_la,
        fixed_delta_seconds_la,
        town_la,
        register_all_sensors_la,
        ego_vehicle_role_name_la,
        spawn_point_la,
        target_speed_la,
        avoid_risk_la,
        sigterm_timeout_la,
        view_la,
        publish_fixed_goal_pose_la,
        goal_pose_la,
        start_global_planner_carla_la,
        control_loop_rate_la,
        input_msg_is_stamped_la,
        joy_la,
        mux_la,
        record_waypoints_la,
        mpc_config_la,
        mpc_build_directory_la,
        launch_custom_controller_la,
        custom_controller_la,
        publish_twist_la,
        carla_waypoint_following_kp_lateral_la,
        carla_waypoint_following_ki_lateral_la,
        carla_waypoint_following_kd_lateral_la,
        carla_waypoint_following_kp_longitudinal_la,
        carla_waypoint_following_ki_longitudinal_la,
        carla_waypoint_following_kd_longitudinal_la,
    ]

    """ Get launch context """
    carla_simulator_script_string = carla_simulator_script.perform(context)
    role_name_string = role_name.perform(context)
    target_speed_string = target_speed.perform(context)
    simulation_tick_rate_string = simulation_tick_rate.perform(context)
    hardware_acceleration_driver_string = hardware_acceleration_driver.perform(context)
    audio_passthrough_string = audio_passthrough.perform(context)
    headless_rendering_string = headless_rendering.perform(context)
    graphics_quality_string = graphics_quality.perform(context)
    publish_fixed_goal_pose_string = publish_fixed_goal_pose.perform(context)
    goal_pose_string = goal_pose.perform(context)
    goal_pose_list = goal_pose_string.split(sep=',')
    goal_pose_orientation_quat = Rotation.from_euler('zyx', goal_pose_list[3:], degrees=True).as_quat().tolist()

    spawn_point_param_name = 'spawn_point_' + role_name_string
    topic_name = "/carla/" + role_name_string + "/target_speed"
    data_string = "{'data': " + target_speed_string + "}"
    if goal_pose_string.lower() is not None:
        goal_pose_string = (("{'header': {'stamp': 'now', 'frame_id': 'map'}, "
                             "'pose': {position: {x: ") + str(goal_pose_list[0]) + ", y: " + str(goal_pose_list[1]) +
                            ", z: " + str(goal_pose_list[2]) + "}, orientation: {x: " + str(
                        goal_pose_orientation_quat[0]) +
                            ", y: " + str(goal_pose_orientation_quat[1]) + ", z: " + str(
                        goal_pose_orientation_quat[2]) +
                            ", w: " + str(goal_pose_orientation_quat[3]) + "}}}")
    # else:
    #     goal_pose_string = ("{'header': {'stamp': 'now', 'frame_id': 'map'}, "
    #                         "'pose': {position: {x: 0.0, y: 0.0, z: 0.0}, "
    #                         "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}")

    if hardware_acceleration_driver_string.lower() == 'cuda':
        hardware_acceleration_driver_string = "-prefernvidia"
    elif hardware_acceleration_driver_string.lower() == 'vulkan':
        hardware_acceleration_driver_string = "-vulkan"
    elif hardware_acceleration_driver_string.lower() == 'cpu':
        hardware_acceleration_driver_string = ""
    else:
        hardware_acceleration_driver_string = "-vulkan"

    if audio_passthrough_string.lower() == 'false':
        audio_passthrough_string = '-nosound'
        audio_environment_variable = SetEnvironmentVariable(name='SDL_AUDIODRIVER', value='dsp')
        launch_args.append(audio_environment_variable)
    else:
        audio_passthrough_string = ''
        # audio_environment_variable = SetEnvironmentVariable(name='SDL_AUDIODRIVER', value='alsa')
        # launch_args.append(audio_environment_variable)

    if headless_rendering_string.lower() == 'true':
        headless_rendering_string = '-RenderOffScreen'
    else:
        headless_rendering_string = ''

    if graphics_quality_string.lower() == 'low':
        graphics_quality_string = '-quality-level=Low'
    elif graphics_quality_string.lower() in ('epic', 'high', 'ultra'):
        graphics_quality_string = '-quality-level=Epic'
    else:
        graphics_quality_string = ''

    """ Launch Nodes """
    # Todo: launch as a python node using subprocess for better error handling and shutdown.
    # todo: implement https://github.com/carla-simulator/carla/issues/4230
    # carla_launch = ExecuteProcess(
    #         cmd=[[
    #             "bash ",
    #             carla_simulator_script_string,
    #             f" {hardware_acceleration_driver_string}",
    #             f" {audio_passthrough_string}",
    #             f" -benchmark -fps={simulation_tick_rate_string}",
    #             f" {headless_rendering_string}",
    #             f" {graphics_quality_string}",
    #         ]],
    #         name="carla_simulator",
    #         output="both",
    #         on_exit=[
    #             OpaqueFunction(function=carla_shutdown_callback)
    #         ],
    #         shell=True,
    #         condition=IfCondition(launch_simulator),
    #         emulate_tty=True
    # )

    carla_launch = Node(
            package='autonomous_driving_simulators',
            executable='CarlaUE4.sh',
            name='carla_simulator',
            output='screen',
            condition=IfCondition(launch_simulator),
            arguments=[
                f"{hardware_acceleration_driver_string}",
                f"{audio_passthrough_string}",
                f"-benchmark -fps={simulation_tick_rate_string}",
                f"{headless_rendering_string}",
                f"{graphics_quality_string}"
            ],
            on_exit=OpaqueFunction(function=carla_shutdown_callback),
            emulate_tty=True
    )

    carla_ros_bridge = Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'host': host,
                    'port': port,
                    'timeout': timeout,
                    'passive': passive,
                    'synchronous_mode': synchronous_mode,
                    'synchronous_mode_wait_for_vehicle_control_command': synchronous_mode_wait_for_vehicle_control_command,
                    'fixed_delta_seconds': fixed_delta_seconds,
                    'town': town,
                    'register_all_sensors': register_all_sensors,
                    'ego_vehicle_role_name': ego_vehicle_role_name
                }
            ],
            # remappings=[
            #     (f'/carla/{role_name_string}/rgb_front/camera_info', '/sensing/camera/traffic_light/camera_info'),
            #     (f'/carla/{role_name_string}/rgb_front/image', '/sensing/camera/traffic_light/image_raw'),
            #     (f'/carla/{role_name_string}/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
            #     (f'/carla/{role_name_string}/tamagawa/imu_link', f'/carla/{role_name_string}/imu'),
            #     (f'/carla/{role_name_string}/imu', '/sensing/imu/tamagawa/imu_raw'),
            #     (f'/carla/{role_name_string}/velodyne_top', f'/carla/{role_name_string}/lidar'),
            #     (f'/carla/{role_name_string}/lidar', '/sensing/lidar/top/pointcloud_raw'),
            # ]  # todo: test
    )

    carla_spawn_objects_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory(
                            'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': get_package_share_directory(
                        'autonomous_driving_simulators') + '/config/objects.json',  # todo: make this a launch argument
                spawn_point_param_name: spawn_point
            }.items()
    )

    carla_target_speed_publisher_node = ExecuteProcess(
            output="log",
            cmd=["ros2", "topic", "pub", topic_name,
                 "std_msgs/msg/Float64", data_string, "--qos-durability", "transient_local"],
            name='topic_pub_target_speed')

    # carla_ad_agent_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #                 os.path.join(get_package_share_directory(
    #                         'carla_ad_agent'), 'carla_ad_agent.launch.py')
    #         ),
    #         launch_arguments={
    #             'role_name': role_name,
    #             'avoid_risk': avoid_risk
    #         }.items()
    # )

    carla_ad_agent_launch = Node(
            package='carla_ad_agent',
            executable='ad_agent',
            name=['carla_ad_agent_', role_name],
            output='screen',
            parameters=[
                {
                    'role_name': role_name,
                    'avoid_risk': avoid_risk
                }
            ]
    )

    goal_pose_publisher_node = TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                        output="screen",
                        condition=IfCondition(publish_fixed_goal_pose),
                        cmd=[
                            # "/goal_pose" or f"/carla/{role_name_string}/goal"
                            "ros2", "topic", "pub", f"/carla/{role_name_string}/goal",
                            "geometry_msgs/msg/PoseStamped", goal_pose_string, "--once",
                        ],
                        name='goal_pose_publisher')
            ]
    )

    carla_goal_pose_relay_node = ExecuteProcess(
            output="screen",
            cmd=["ros2", "run", "topic_tools", "relay", "/goal_pose", f"/carla/{role_name_string}/goal"],
            # name='carla_goal_pose_relay'
    )

    carla_waypoint_publisher_node = Node(
            package='carla_waypoint_publisher',
            executable='carla_waypoint_publisher',
            name='carla_waypoint_publisher',
            condition=IfCondition(start_global_planner_carla),
            # todo: either use carla global planner or waypoint publisher from CSV
            output='screen',
            emulate_tty='True',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'host': host,
                    'port': port,
                    'timeout': timeout,
                    'role_name': role_name
                }
            ],
            remappings=[
                # (f'/carla/{role_name_string}/goal', '/goal_pose')
            ]
    )

    carla_manual_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory(
                            'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': role_name
            }.items(),
            condition=IfCondition(view)
    )

    ''' Launches a low level PID to convert speed, acceleration and steering commands to actuation commands.
    When tuning the cascade from MPC to PID use this PID.
     '''
    carla_ackermann_control_node = Node(
            package='carla_ackermann_control',
            executable='carla_ackermann_control_node',
            name=f'carla_ackermann_control_{role_name_string}',
            output='screen',
            condition=UnlessCondition(publish_twist),
            parameters=[
                Path(get_package_share_directory('autonomous_driving_simulators'), "PID_low_level.yaml"),
                {
                    'use_sim_time': use_sim_time,
                    'role_name': role_name,
                    'control_loop_rate': control_loop_rate,
                    'input_msg_is_stamped': input_msg_is_stamped
                }
            ],
            remappings=[
                # todo: add launch context to set to /drive if either joy teleop or ackermann mux is off
                (f'/carla/{role_name_string}/ackermann_cmd', f'/ackermann_cmd_{role_name_string}')
            ]
    )

    carla_twist_to_control_node = Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name=f"carla_twist_to_control_{role_name_string}",
            output='screen',
            emulate_tty='True',
            condition=IfCondition(publish_twist),
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                    'input_msg_is_stamped': input_msg_is_stamped
                }
            ],
            remappings=[
                (f'/carla/{role_name_string}/twist', '/twist')
            ]
    )

    # todo: add teleop ifcondition
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                joy_config,
                {
                    'use_sim_time': use_sim_time,
                }
            ]
    )

    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[
                joy_config,
                {
                    'use_sim_time': use_sim_time,
                    # 'human_control.deadman_buttons': deadman_buttons,
                    # 'human_control.axis_mappings.drive-speed.scale': max_speed,  # max speed in m/s
                    # 'human_control.axis_mappings.drive-steering_angle.scale': max_steering,  # max steering in rads
                }
            ]
    )

    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[mux_config],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', f'/ackermann_cmd_{role_name_string}')]
    )

    waypoint_recording_node = Node(
            package='trajectory_following_ros2',
            executable='waypoint_recorder',
            name='waypoint_recording_node',
            output='screen',
            condition=IfCondition(record_waypoints),
            parameters=[
                {
                    'use_sim_time': True,
                    'file_path': '/home/carla/shared_dir/waypoints/carla/waypoints.csv',  # todo
                    'save_interval': 1,
                    'odom_topic': f'/carla/{role_name_string}/odometry',
                    'target_frame_id': 'map',
                    'save_if_transform_fails': True
                }
            ]
    )

    # low level PID used to follow waypoints. Note that this is different from the low-level Cascaded PID of
    # carla ackermann control even though they are doing the same thing.
    # Therefore, when tuning the cascade from MPC to PID use the ackermann PID instead
    carla_waypoint_following_node = Node(
            package='carla_ad_agent',
            executable='local_planner',
            name=['carla_local_planner_', role_name],
            output='screen',
            condition=UnlessCondition(launch_custom_controller),
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
                    'control_time_step': control_loop_rate,
                }
            ]
    )

    custom_mpc_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(
                            get_package_share_directory('trajectory_following_ros2'),
                            'launch', 'mpc.launch.py'
                    )
            ),
            condition=LaunchConfigurationEquals('custom_controller', 'mpc'),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': mpc_config,
                'generate_mpc_model': 'True',
                'build_with_cython': 'True',
                'model_directory': mpc_build_directory,
                'horizon': '25',
                'frequency': '20.0',
                'sample_time': '0.05',  # control_loop_rate
                'prediction_time': '1.5',
                'publish_twist_topic': publish_twist,
                'wheelbase': '2.87528',
                'max_steer': '69.99999284118222',
                'min_steer': '-69.99999284118222',
                'max_steer_rate': '120.0',
                'max_speed': '10.5',
                'min_speed': '-10.5',
                'max_accel': '3.0',
                'max_decel': '-3.0',
                'R_diagonal': '[0.01, 0.01]',
                'Rd_diagonal': '[10., 100.]',
                'Q_diagonal': '[1.0, 1.0, 1.0, 0.01]',
                'Qf_diagonal': '[0.04, 0.04, 0.1, 0.01]',
                'max_iterations': '15',
                'termination_condition': '0.001',
                'stage_cost_type': 'NONLINEAR_LS',
                'distance_tolerance': '5.0',
                'speed_tolerance': '5.0',
                'load_waypoints': 'True',  # todo: either use carla global planner or waypoint publisher from CSV
                'waypoints_csv': '/home/carla/shared_dir/waypoints/carla/waypoints.csv',  # todo
                'mpc_toolbox': 'acados',
                'ode_type': 'continuous_kinematic_coupled',
                # 'desired_speed': target_speed,
                'odom_topic': f'/carla/{role_name_string}/odometry',
                'ackermann_cmd_topic': '/drive',
                # f'/ackermann_cmd_{role_name_string}' or f'/carla/{role_name_string}/ackermann_cmd'
                'twist_topic': '/twist',  # f'/carla/{role_name_string}/twist'
                'acceleration_topic': '/accel/local',  # f'/carla/{role_name_string}/twist'
                'path_topic': '/trajectory/path',
                'speed_topic': '/trajectory/speed',
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
                    'file_path': '/home/carla/shared_dir/waypoints/carla/waypoints.csv',  # todo
                    'path_source': 'file',
                    'control_rate': 20.0,
                    'goal_tolerance': 5.0,
                    'lookahead_distance': 7.0,
                    'min_lookahead': 4.35,
                    'max_lookahead': 15.0,
                    'adaptive_lookahead_gain': 4.0,
                    'use_adaptive_lookahead': False,
                    'wheelbase': 2.87528,
                    'max_steer': 69.99999284118222,
                    'min_steer': -69.99999284118222,
                    'max_steer_rate': 352.9411764706,
                    'max_speed': 15.0,
                    # 'min_speed': '-10.5',
                    'speed_Kp': 1.0,
                    'speed_Ki': 0.0,
                    'speed_Kd': 0.0,
                    'desired_speed': target_speed,
                    'odom_topic': f'/carla/{role_name_string}/odometry',
                    'ackermann_cmd_topic': '/drive',
                    # f'/ackermann_cmd_{role_name_string}' or f'/carla/{role_name_string}/ackermann_cmd'
                    'twist_topic': '/twist',  # f'/carla/{role_name_string}/twist'
                    'acceleration_topic': '/accel/local',  # f'/carla/{role_name_string}/twist'
                    'path_topic': '/trajectory/path',
                    'speed_topic': '/trajectory/speed',
                    'speedup_first_lookup': True,
                }
            ]
    )

    custom_controller_group = GroupAction(
            condition=IfCondition(launch_custom_controller),
            actions=[
                custom_mpc_node,
                custom_purepursuit_node
            ]
    )

    ld = launch_args + [
        carla_launch,
        carla_ros_bridge,
        carla_spawn_objects_launch,
        carla_target_speed_publisher_node,
        carla_ad_agent_launch,
        goal_pose_publisher_node,
        carla_goal_pose_relay_node,
        carla_waypoint_publisher_node,
        carla_waypoint_following_node,
        carla_manual_control,
        carla_ackermann_control_node,
        carla_twist_to_control_node,
        joy_node,
        joy_teleop_node,
        ackermann_mux_node,
        waypoint_recording_node,
        custom_controller_group
    ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
