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
    input_occupancy_grid = DeclareLaunchArgument(
        "input/occupancy_grid", default_value="input/occupancy_grid"
    )
    ground_name = DeclareLaunchArgument("ground_name", default_value="ground_name")
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    tms_sd_ground_node = Node(
        package="tms_sd_ground",
        executable="tms_sd_ground",
        output="screen",
        remappings=[
            ("~/input/occupancy_grid", LaunchConfiguration("input/occupancy_grid")),
        ],
        parameters=[
            {
                "ground_name": LaunchConfiguration("ground_name"),
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            input_occupancy_grid,
            ground_name,
            to_frame,
            tms_sd_ground_node,
        ]
    )
