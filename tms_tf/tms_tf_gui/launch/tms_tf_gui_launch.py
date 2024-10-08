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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    params = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "params.yaml"
    )
    default_config_file = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "tf_config.json"
    )
    config_file = DeclareLaunchArgument(
        "config_file", default_value=default_config_file
    )

    # Nodes
    ground_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="ground_tf",
        executable="ground_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}, params],
    )

    machine_odom_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="machine_odom_tf",
        executable="machine_odom_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}, params],
    )

    return LaunchDescription(
        [
            config_file,
            ground_tf_broadcaster_node,
            machine_odom_tf_broadcaster_node,
        ]
    )
