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
    output_mesh = DeclareLaunchArgument(
        'output/terrain/mesh_srv', default_value='output/terrain/mesh_srv'
    )
    output_texture = DeclareLaunchArgument(
        'output/terrain/texture', default_value='output/terrain/texture'
    )
    filename_mesh = DeclareLaunchArgument(
        'filename_mesh', default_value='filename_mesh'
    )
    DATA_TYPE = DeclareLaunchArgument(
        'DATA_TYPE_TERRAIN', default_value='heightmap', description='Type of the data (heightmap or other)'
    )
    DATA_TYPE_TEXTURE = DeclareLaunchArgument(
        'DATA_TYPE_TEXTURE', default_value='texture', description='Type of the data (heightmap or other)'
    )

    # Nodes
    #tms_ur_construction_terrain_mesh_node = Node(
    tms_ur_construction_terrain_heightmap_node = Node(
        package='tms_ur_construction',
      #  executable='tms_ur_construction_terrain_mesh',
        executable='tms_ur_construction_terrain_heightmap',
        output='screen',
        remappings=[
            ('~/output/terrain/mesh_srv', LaunchConfiguration('output/terrain/mesh_srv')),
        ],
        parameters=[{
            'filename_mesh': LaunchConfiguration('filename_mesh'),
            'DATA_TYPE': LaunchConfiguration('DATA_TYPE_TERRAIN'),
        }]
    )

    # Nodes
    #tms_ur_construction_terrain_mesh_node = Node(
    tms_ur_construction_terrain_texture_node = Node(
        package='tms_ur_construction',
      #  executable='tms_ur_construction_terrain_mesh',
        executable='tms_ur_construction_terrain_heightmap',
        output='screen',
        remappings=[
            ('~/output/terrain/mesh_srv', LaunchConfiguration('output/terrain/mesh_srv')),
        ],
        parameters=[{
            'filename_mesh': LaunchConfiguration('filename_mesh'),
            'DATA_TYPE': LaunchConfiguration('DATA_TYPE_TEXTURE'),
        }]
    )


    return LaunchDescription([
        output_mesh,
        filename_mesh,
        DATA_TYPE,
        DATA_TYPE_TEXTURE,
        tms_ur_construction_terrain_heightmap_node,
        tms_ur_construction_terrain_texture_node,
        #tms_ur_construction_terrain_mesh_node,
    ])
