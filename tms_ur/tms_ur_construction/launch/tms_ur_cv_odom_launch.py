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
    output_odom = DeclareLaunchArgument("output/odom", default_value="output/odom")
    latest = DeclareLaunchArgument("latest", default_value="False")
    machine_name = DeclareLaunchArgument("machine_name", default_value="machine_name")

    # Nodes
    tms_ur_cv_odom_node = Node(
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", LaunchConfiguration("output/odom")),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": LaunchConfiguration("machine_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            output_odom,
            latest,
            machine_name,
            tms_ur_cv_odom_node,
        ]
    )
