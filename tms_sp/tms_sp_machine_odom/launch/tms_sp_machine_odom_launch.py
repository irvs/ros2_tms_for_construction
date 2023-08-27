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
    input_odom = DeclareLaunchArgument("input/odom", default_value="input/odom")
    machine_name = DeclareLaunchArgument("machine_name", default_value="machine_name")
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    tms_sp_machine_odom_node = Node(
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", LaunchConfiguration("input/odom")),
        ],
        parameters=[
            {
                "machine_name": LaunchConfiguration("machine_name"),
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            input_odom,
            machine_name,
            to_frame,
            tms_sp_machine_odom_node,
        ]
    )
