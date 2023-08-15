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
    # Nodes
    terrain_mesh_cli_pub_node = Node(
        package='tms_ur_test',
        executable='tms_ur_construction_terrain_mesh_test',
        output='screen',
    )
    terrain_static_cli_pub_node = Node(
        package='tms_ur_test',
        executable='tms_ur_construction_terrain_static_test',
        output='screen',
    )

    return LaunchDescription([
        terrain_mesh_cli_pub_node,
        terrain_static_cli_pub_node,
    ])