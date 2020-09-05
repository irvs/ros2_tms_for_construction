# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('tms_rc_qurin_support'),
            'maps',
            'map_w2_9f.yaml'))

    param_file_name = 'guidebot_params.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('tms_rc_qurin_support'),
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('tms_rc_qurin_support'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    share_dir_path = os.path.join(get_package_share_directory('tms_rc_qurin_support'))
    #urdf_path = os.path.join(share_dir_path, 'urdf', 'collidor_928_and_957.urdf')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'model.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/foxy_quriana_bringup.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),

        # TODO: remap /cmd_vel -> /hapirobo/cmd_vel
        # this node re-publish /cmd_vel with rename
        Node(
            package='tms_rc_qurin_support',
            executable='convert',
            output='screen'
        ),

        Node(
            package='tms_rc_qurin_support',
            executable='guidebot_odometry',
            output='screen'
        ),

        Node(
            package='tms_rc_tfnode',
            executable='odom_tf_node',
            output='screen',
        ),

        # Node(
        #     package='tms_rc_qurin_support',
        #     executable='pozyx_local',
        #     output='screen'
        # ),

        # TODO: change robot_base_frame /base_link -> /base_footprint
        # this node publish /base_link = /base_footprint, this is not elegant.
        Node(package='tf2_ros',
            node_executable='static_transform_publisher',
            output='both',
            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]
        ),

        # static_transform_publisher: map -> origin_position
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='both',
            arguments=["34.6", "2.15", "0", "0", "0", "0", "map", "origin_position"]
        ),

        # static_transform_publisher: base_footprint -> pozyx
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='both',
            arguments=["-0.09", "-0.165", "0.5", "0", "0", "0", "base_footprint", "pozyx"]
        ),


        # collidor model: (frame: origin_position)
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='both',
            # argumentsでURDFを出力したパスを指定
            arguments=[urdf_path]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])