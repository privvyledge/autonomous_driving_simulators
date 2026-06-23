"""Standalone built-in CARLA AD agent (ad_agent + local_planner).

Extracted from simulation_bringup.launch.py so the agent can be started on its
OWN, after the rest of the simulation (bridge + ego spawn + global planner) and
the waypoint recorder are already up and confirmed. This restores the manual
start-ordering of the pre-Compose workflow: bring the world up stationary, arm
the recorder, then launch THIS file to release the ego to autonomy so the
recorder captures the path from the very first motion.

simulation_bringup.launch.py includes this file (gated by launch_builtin_agent)
so the bundled behaviour is unchanged; this just makes the agent independently
launchable.
"""
import os
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    role_name = LaunchConfiguration('role_name')
    avoid_risk = LaunchConfiguration('avoid_risk')
    goal_pose = LaunchConfiguration('goal_pose')
    fixed_delta_seconds = LaunchConfiguration('fixed_delta_seconds')

    carla_waypoint_following_kp_lateral = LaunchConfiguration('carla_waypoint_following_kp_lateral')
    carla_waypoint_following_ki_lateral = LaunchConfiguration('carla_waypoint_following_ki_lateral')
    carla_waypoint_following_kd_lateral = LaunchConfiguration('carla_waypoint_following_kd_lateral')
    carla_waypoint_following_kp_longitudinal = LaunchConfiguration('carla_waypoint_following_kp_longitudinal')
    carla_waypoint_following_ki_longitudinal = LaunchConfiguration('carla_waypoint_following_ki_longitudinal')
    carla_waypoint_following_kd_longitudinal = LaunchConfiguration('carla_waypoint_following_kd_longitudinal')

    role_name_string = role_name.perform(context)
    goal_pose_string = goal_pose.perform(context)

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

    return [
        carla_ad_agent_launch,
        carla_waypoint_following_node,
        goal_pose_publisher_node,
        carla_goal_pose_relay_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulated clock.'),
        DeclareLaunchArgument('role_name', default_value='ego_vehicle', description='Ego vehicle role name.'),
        DeclareLaunchArgument('avoid_risk', default_value='True', description='Avoid risk and obey traffic rules.'),
        DeclareLaunchArgument('goal_pose', default_value='127.4,195.4,0.0,180.0,0,0', description='Target goal pose "x,y,z,yaw,pitch,roll" (deg) or "none".'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05', description='Simulation step size (local_planner control_time_step).'),

        DeclareLaunchArgument('carla_waypoint_following_kp_lateral', default_value='0.9'),
        DeclareLaunchArgument('carla_waypoint_following_ki_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kd_lateral', default_value='0.0'),
        DeclareLaunchArgument('carla_waypoint_following_kp_longitudinal', default_value='0.206'),
        DeclareLaunchArgument('carla_waypoint_following_ki_longitudinal', default_value='0.0206'),
        DeclareLaunchArgument('carla_waypoint_following_kd_longitudinal', default_value='0.515'),

        OpaqueFunction(function=launch_setup)
    ])
