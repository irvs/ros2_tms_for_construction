# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_terrain_static_pointcloud2 = DeclareLaunchArgument(
        'input/terrain/static/pointcloud2', default_value='input/terrain/static/pointcloud2'
    )
    input_terrain_dynamic_pointcloud2 = DeclareLaunchArgument(
        'input/terrain/dynamic/pointcloud2', default_value='input/terrain/dynamic/pointcloud2'
    )
    terrain_name = DeclareLaunchArgument(
        'terrain_name', default_value='terrain'
    )

    tms_sd_terrain_static_node = Node(
        package='tms_sd_terrain',
        executable='tms_sd_terrain_static',
        output='screen',
        remappings=[
            ('~/input/terrain/static/pointcloud2', LaunchConfiguration('input/terrain/static/pointcloud2')),
        ],
    )
    tms_sd_terrain_dynamic_node = Node(
        package='tms_sd_terrain',
        executable='tms_sd_terrain_dynamic',
        output='screen',
        remappings=[
            ('~/input/terrain/dynamic/pointcloud2', LaunchConfiguration('input/terrain/dynamic/pointcloud2')),
        ],
        parameters=[{
            'terrain_name': LaunchConfiguration('terrain_name'),
        }]
    )

    return LaunchDescription([
        input_terrain_static_pointcloud2,
        input_terrain_dynamic_pointcloud2,
        terrain_name,
        tms_sd_terrain_static_node,
        tms_sd_terrain_dynamic_node,
    ])
