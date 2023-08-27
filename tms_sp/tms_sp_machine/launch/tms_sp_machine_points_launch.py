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
    input_points = DeclareLaunchArgument(
        "input/machine_points", default_value="/input/machine_points"
    )
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    # Nodes
    machine_points = Node(
        package="tms_sp_machine",
        executable="tms_sp_machine_points",
        output="screen",
        remappings=[
            ("~/input/machine_points", LaunchConfiguration("input/machine_points")),
        ],
        parameters=[
            {"to_frame": LaunchConfiguration("to_frame")},
        ],
    )

    return LaunchDescription([input_points, to_frame, machine_points])
