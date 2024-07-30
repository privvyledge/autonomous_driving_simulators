"""
Todo:
    * add IMU support [done]
    * add RGB camera [done]
    * add depth camera [done]
    * setup rtabmap 2D mapping [done]
    * test GNSS [done]
    * switch to Rtabmap launch
    * setup config arguments for either RGBD mapping or ICP/Pointcloud mapping
    * rename rtabmap topics
    * add rtabmap database path
    * save rtabmap rviz configuration
    * add rtabmap arguments
    * include carla_bringup.launch.py
    * setup initial pose
    * setup launching rviz
    * add support for pointcloud fusion and multiple LIDARs
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


def launch_setup(context, *args, **kwargs):
    # Setup default directories.

    # Setup launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    mapping_2d = LaunchConfiguration('mapping_2d', default='False')
    deskewing = LaunchConfiguration('deskewing', default='False')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/carla/ego_vehicle/lidar')

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            name='use_sim_time',
            default_value=use_sim_time,
            description='Use simulated clock.'
    )

    mapping_2d_la = DeclareLaunchArgument(
            name='mapping_2d',
            default_value=mapping_2d,
            description='True: uses slam toolbox, False: Uses RTABMap.'
    )

    deskewing_la = DeclareLaunchArgument(
            'deskewing',
            default_value=deskewing,
            description='Enable lidar deskewing')

    pointcloud_topic_la = DeclareLaunchArgument(
            'pointcloud_topic',
            default_value=pointcloud_topic,
            description='The pointcloud topic to use.')

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        mapping_2d_la,
        deskewing_la,
        pointcloud_topic_la,
    ]

    """ Get launch context """

    """ Launch Nodes """
    lidar_icp_odometry = Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
                'frame_id': 'ego_vehicle',  # frame to publish in
                'odom_frame_id': 'map',
                'wait_for_transform': 0.2,
                # 'wait_imu_to_init': False,
                # 'queue_size': 10,
                # 'approx_sync': True,
                'guess_frame_id': 'map',  # otherwise will start at 0.0
                # 'initial_pose': 'x y z roll pitch yaw',  # todo: setup and use instead of guess_frame_id
                # 'ground_truth_frame_id': 'map',  # todo: setup and use instead of guess_frame_id
                # 'guess_min_translation': 0.05,  # m
                # 'guess_min_rotation': 0.005,  # rad
                'publish_tf': False,
                'publish_null_when_lost': True,
                # 'expected_update_rate': 10.0,  # set higher than expected, e.g for 10/20 Hz (actual) use 30 Hz
                'deskewing': deskewing,
                'use_sim_time': use_sim_time,
                # RTAB-Map's internal parameters are strings:
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/VoxelSize': '0.1',
                'Icp/Epsilon': '0.001',
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/MaxTranslation': '2',
                'Icp/MaxCorrespondenceDistance': '1',
                'Icp/Strategy': '1',
                'Icp/OutlierRatio': '0.7',
                'Icp/CorrespondenceRatio': '0.01',
                'Odom/ScanKeyFrameThr': '0.4',
                'OdomF2M/ScanSubtractRadius': '0.1',
                'OdomF2M/ScanMaxSize': '15000',
                'OdomF2M/BundleAdjustment': 'false'
            }],
            remappings=[
                ('scan_cloud', pointcloud_topic),
                # ('odom', '/carla/ego_vehicle/odometry/icp'),
                # ('odom_last_frame', '/carla/ego_vehicle/rtabmap/icp/points'),
                # ('odom_filtered_input_scan', '/carla/ego_vehicle/lidar_filtered'),
                # ('imu', '/carla/ego_vehicle/imu'),  # imu must have orientation. todo: test
            ])

    rgbd_odometry = Node(
            package='rtabmap_odom', executable='rgbd_odometry', name="rgbd_odometry", output="screen",
            parameters=[{
                "frame_id": 'ego_vehicle',
                "odom_frame_id": 'map',
                "publish_tf": 'False',
                "wait_for_transform": 0.2,
                "wait_imu_to_init": 'False',
                "approx_sync": 'False',
                "guess_frame_id": 'map'}],
            remappings=[
                ('/rgb/image', '/carla/ego_vehicle/rgb_front/image'),
                ('/rgb/camera_info', '/carla/ego_vehicle/rgb_front/camera_info'),
                ('/depth/image', '/carla/ego_vehicle/depth_front/image'),
                # ("odom", '/carla/ego_vehicle/odometry/rgbd'),
                ('imu', '/carla/ego_vehicle/imu'),
            ]
    )

    pointcloud_assembler_node = Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'max_clouds': 10,
                'fixed_frame_id': '',  # map, ego_vehicle
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan'),
                # ('odom', '/carla/ego_vehicle/odometry/icp'),
                # ('assembled_cloud', '/carla/ego_vehicle/lidar_assembled')
            ])

    # todo: switch to the launch file to do odometry + slam + rtabviz
    slam_node = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': 'ego_vehicle',
                # subscribe_depth should be true but performs worse.
                # Also, must modify the depth image size to match RGB in objects.json file.
                'subscribe_depth': False,
                'subscribe_rgb': True,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'wait_for_transform': 0.2,
                'wait_imu_to_init': True,
                'use_sim_time': use_sim_time,
                # 'scan_cloud_topic': 'assembled_cloud',
                #'odom_topic': '/carla/ego_vehicle/odometry',  # ego_vehicle, /odom or icp_odometry topic
                #'imu_topic': '/carla/ego_vehicle/imu',
                # 'vo_frame_id': 'map',
                'map_frame_id': 'rtabmap',  # remap to something else since Carla does not allow odom frame
                'publish_tf_map': 'True',
                'publish_tf_odom': 'False',
                # RTAB-Map's internal parameters are strings:
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/ProximityPathMaxNeighbors': '1',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/CreateOccupancyGrid': 'true',  # tested: false, todo: test true
                'Mem/NotLinkedNodesKept': 'false',
                'Mem/STMSize': '30',
                'Mem/LaserScanNormalK': '20',
                'Reg/Strategy': '1',
                'Icp/VoxelSize': '0.1',
                'Icp/PointToPlaneK': '20',
                'Icp/PointToPlaneRadius': '0',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '10',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '3',
                'Icp/MaxCorrespondenceDistance': '1',
                'Icp/Strategy': '1',
                'Icp/OutlierRatio': '0.7',
                'Icp/CorrespondenceRatio': '0.2',
                'map_always_update': True,
                'Grid/FromDepth': 'False',
                'Grid/Sensor': '0',  # 0=laser scan, 1=depth image(s), 2=both
                'Grid/RangeMax': '0',  # 0=Inf
                # A 3D occupancy grid is required if you want an OctoMap (3D ray tracing).
                # Set to false if you want only a 2D map, the cloud will be projected on xy plane.
                # A 2D map can be still generated if checked, but it requires more memory and time to generate it.
                # Ignored if laser scan is 2D and Grid/Sensor is 0.
                'Grid/3D': 'true',  # todo: test true
                # Ray tracing is done for each occupied cell, filling unknown space between
                # the sensor and occupied cells. If true, RTAB-Map should be built with OctoMap support,
                # otherwise 3D ray tracing is ignored.
                'Grid/RayTracing': 'true',
                'Reg/Force3DoF': 'true',
                'Optimizer/Slam2D': 'true',
                'Optimizer/GravitySigma': '0',
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud'),
                ('imu', '/carla/ego_vehicle/imu'),
                ('odom', '/carla/ego_vehicle/odometry'),
                ('/rgb/camera_info', '/carla/ego_vehicle/rgb_front/camera_info'),
                ('/rgb/image', '/carla/ego_vehicle/rgb_front/image'),
                ('/depth/image', '/carla/ego_vehicle/depth_front/image'),
                ('/gps/fix', '/carla/ego_vehicle/gnss'),
            ],
            arguments=[
                '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
            ])

    rtab_viz_node = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                'frame_id': 'ego_vehicle',
                'odom_frame_id': 'map',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_odom_info': True,
                'subscribe_scan_cloud': True,
                'approx_sync': False,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan'),
                ('/rgb/camera_info', '/carla/ego_vehicle/rgb_front/camera_info'),
                ('/rgb/image', '/carla/ego_vehicle/rgb_front/image'),
                ('/depth/image', '/carla/ego_vehicle/depth_front/image'),
            ])

    ld = launch_args + [
        lidar_icp_odometry,
        rgbd_odometry,
        pointcloud_assembler_node,
        slam_node,
        rtab_viz_node
    ]

    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
