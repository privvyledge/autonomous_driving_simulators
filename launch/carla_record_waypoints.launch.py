"""
Records Pose and Velocity information from a Carla odometry pseudosensor.

Steps:
    1. Launch Carla simulation
    2. Launch Carla ROS bridge
    3. Spawn the ego_vehicle
    4. Launch the waypoint publishing node, i.e go to goal (point stabilization). Do (1-3) using carla AD demo.
    5. (optional) spawn other traffic
    6. Launch the waypoint recording node
    6. Move the car manually or Autonomously using one of the following
        1. Publish the goal point for go-to-goal (or click 2D Goal Pose and draw a line on Rviz)
        2. Teleoperate using a joystick, i.e joy_teleop
        3. Move the car using the GUI and manual control
    7. Ctrl-C to stop recording
    6. Publish the goal point
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction, \
    SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    # Get package directories

    # Get launch directories

    # Setup default directories.

    # Setup launch configuration variables

    # Declare launch arguments

    # Add launch arguments to a list

    ''' Launch Nodes '''

    ld = launch_args + [

    ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
