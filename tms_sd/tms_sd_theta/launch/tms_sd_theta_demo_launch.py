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
from launch_ros.actions import Node


def generate_launch_description():
    tms_sd_theta_compressed_node1 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client1/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod1",
            },
        ],
    )
    tms_sd_theta_compressed_node2 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client2/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod2",
            },
        ],
    )
    tms_sd_theta_compressed_node3 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client3/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod3",
            },
        ],
    )
    tms_sd_theta_compressed_node4 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client4/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod4",
            },
        ],
    )

    return LaunchDescription(
        [
            tms_sd_theta_compressed_node1,
            tms_sd_theta_compressed_node2,
            tms_sd_theta_compressed_node3,
            tms_sd_theta_compressed_node4,
        ]
    )
