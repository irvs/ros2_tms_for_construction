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
    output_ground_mesh = DeclareLaunchArgument(
        "output/ground_mesh", default_value="output/ground_mesh"
    )
    timer_period = DeclareLaunchArgument("timer_period", default_value="10")

    # Nodes
    tms_ur_ground_mesh_node = Node(
        package="tms_ur_construction",
        executable="tms_ur_ground_mesh",
        output="screen",
        remappings=[
            ("~/output/ground_mesh", LaunchConfiguration("output/ground_mesh")),
        ],
        parameters=[
            {
                "timer_period": LaunchConfiguration("timer_period"),
            }
        ],
    )

    return LaunchDescription(
        [
            output_ground_mesh,
            timer_period,
            tms_ur_ground_mesh_node,
        ]
    )
