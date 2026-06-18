#   MIT License
#
#   Copyright (c) 2023 Robotics010
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulated clock'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_steer_map',
            default_value=get_package_share_directory(
                'carla_autoware_bridge') + '/data/carla_tesla_model3/steer_map.csv'
        ),
        launch_ros.actions.Node(
            package='carla_autoware_bridge',
            executable='carla_autoware_bridge',
            name='carla_autoware_bridge',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                },
                {
                    'csv_path_steer_map': launch.substitutions.LaunchConfiguration(
                        'csv_path_steer_map')
                }
            ],
            remappings=[
                ('~/input/odometry', '/carla/ego_vehicle/odometry'),
                ('~/input/status', '/carla/ego_vehicle/vehicle_status'),
                ('~/input/steering', '/carla/ego_vehicle/vehicle_steering'),
                ('~/input/actuation', '/control/command/actuation_cmd'),
                ('~/input/lidar', '/sensing/lidar/top/pointcloud_raw'),
                ('~/output/velocity_status', '/vehicle/status/velocity_status'),
                ('~/output/steering_status', '/vehicle/status/steering_status'),
                ('~/output/actuation_status', '/vehicle/status/actuation_status'),
                ('~/output/control', '/carla/ego_vehicle/vehicle_control_cmd'),
                ('~/output/lidar_ex', '/sensing/lidar/top/pointcloud_raw_ex'),
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
