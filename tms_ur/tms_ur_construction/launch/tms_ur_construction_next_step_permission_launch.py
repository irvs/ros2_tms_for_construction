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
    latest = DeclareLaunchArgument("latest", default_value="True")

    # Nodes
    tms_ur_next_step_permission1 = Node(
    package='tms_ur_construction',
    executable='tms_ur_construction_next_step_permission',
    name='wait_ur_node',
    namespace='mst110cr',
    remappings=[
        ('urpermission', '/urpermission'),
        ('permissionrequest', '/permissionrequest'),
    ],
    output='screen'
    )

    tms_ur_next_step_permission2 = Node(
    package='tms_ur_construction',
    executable='tms_ur_construction_next_step_permission',
    name='wait_ur_node',
    namespace='zx200',
    remappings=[
        ('urpermission', '/urpermission'),
        ('permissionrequest', '/permissionrequest'),
    ],
    output='screen'
    )

    
    tms_ur_plan_reader = Node(
        name="plan_reader",
        package="tms_ur_construction",
        executable="tms_ur_plan_reader",
        output="screen",
        remappings=[
            ("planwritten", "/planwritten"),
            ("~/output/plan", "/output/mst110cr_plan"),
            ("~/output/joint_plan", "/output/zx200_joint_plan"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
        ],
    )


    return LaunchDescription(
        [
            latest,
            tms_ur_next_step_permission1,
            tms_ur_next_step_permission2,
            tms_ur_plan_reader
        ]
    )
