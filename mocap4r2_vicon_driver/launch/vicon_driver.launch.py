# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mocap4r2_vicon_driver')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'vicon_driver.yaml'),
            description='Parameters of the Vicon driver'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'autostart', default_value='configure',
            description='none, configure (connect and wait for START) or activate'),
        Node(
            package='mocap4r2_vicon_driver',
            executable='vicon_driver',
            name='vicon_driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file'),
                        {'autostart': LaunchConfiguration('autostart')}],
            output='screen'),
    ])
