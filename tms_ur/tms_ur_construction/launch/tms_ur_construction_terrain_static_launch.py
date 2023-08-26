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
    # Arguments
    output_terrain_static_pointcloud2 = DeclareLaunchArgument(
        'output/terrain/static/pointcloud2', default_value='output/terrain/static/pointcloud2'
    )

    filename = DeclareLaunchArgument(
        'filename', default_value='filename'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
    )

    # Nodes
    tms_ur_construction_terrain_static_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_static',
        output='screen',
        remappings=[
            ('~/output/terrain/static/pointcloud2', LaunchConfiguration('output/terrain/static/pointcloud2')),
        ],
        parameters=[{
            'latest': LaunchConfiguration('latest'),
        }]
    )

    return LaunchDescription([
        output_terrain_static_pointcloud2,
        filename,
        voxel_size,
        tms_ur_construction_terrain_static_node,
    ])