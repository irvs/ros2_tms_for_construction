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
    input_theta_compressed = DeclareLaunchArgument(
        "input/theta/compressed", default_value="input/theta/compressed"
    )
    theta_name = DeclareLaunchArgument("theta_name", default_value="theta_name")

    tms_sd_theta_compressed_node = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", LaunchConfiguration("input/theta/compressed")),
        ],
        parameters=[
            {
                "theta_name": LaunchConfiguration("theta_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            input_theta_compressed,
            theta_name,
            tms_sd_theta_compressed_node,
        ]
    )
