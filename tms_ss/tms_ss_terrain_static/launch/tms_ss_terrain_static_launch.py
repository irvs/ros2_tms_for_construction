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
    filename = DeclareLaunchArgument(
        'filename', default_value='filename.pcd'
    )
    filename_mesh = DeclareLaunchArgument(
        'filename_mesh', default_value='filename_mesh.ply'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
    )
    octree_depth = DeclareLaunchArgument(
        'octree_depth', default_value='2'
    )
    density_th = DeclareLaunchArgument(
        'density_th', default_value='0.1'
    )

    # Nodes
    tms_ss_terrain_static_node = Node(
        package='tms_ss_terrain_static',
        executable='tms_ss_terrain_static',
        output='screen',
        parameters=[{
            'filename': LaunchConfiguration('filename'),
        }]
    )
    tms_ss_terrain_static_mesh_node = Node(
        package='tms_ss_terrain_static',
        executable='tms_ss_terrain_static_mesh',
        output='screen',
        parameters=[{
            'filename_mesh': LaunchConfiguration('filename_mesh'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'octree_depth': LaunchConfiguration('octree_depth'),
            'density_th': LaunchConfiguration('density_th'),
        }]
    )

    return LaunchDescription([
        filename,
        filename_mesh,
        voxel_size,
        octree_depth,
        density_th,
        tms_ss_terrain_static_node,
        tms_ss_terrain_static_mesh_node,
    ])
