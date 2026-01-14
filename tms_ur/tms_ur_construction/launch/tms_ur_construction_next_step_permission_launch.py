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
    name='st_wait_ur_node',
    namespace='mst110cr',
    remappings=[
        ('urpermission', '/urpermission'),
        ('permisionrequest', '/permisionrequest'),
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
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
        ],
    )

    tms_ur_cv_joints_node1 = Node(
        name="tms_ur_cv_joints1",
        package="tms_ur_construction",
        executable="tms_ur_cv_joints",
        output="screen",
        remappings=[
            ("~/output/joint", "/output/zx200/Joins_states"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "zx200_1",
            },
        ],
    )

    tms_ur_cv_joints_node2 = Node(
        name="tms_ur_cv_joints2",
        package="tms_ur_construction",
        executable="tms_ur_cv_joints",
        output="screen",
        remappings=[
            ("~/output/joint", "/output/ic120/joint_states"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "ic120_1",
            },
        ],
    )
    '''
    tms_ur_cv_odom_node3 = Node(
        name="tms_ur_cv_odom3",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump1/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump1",
            },
        ],
    )
    tms_ur_cv_odom_node4 = Node(
        name="tms_ur_cv_odom4",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump2/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump2",
            },
        ],
    )
    tms_ur_cv_odom_node5 = Node(
        name="tms_ur_cv_odom5",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump3/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump3",
            },
        ],
    )
    '''

    return LaunchDescription(
        [
            latest,
            tms_ur_next_step_permission1,
            tms_ur_plan_reader
        #    tms_ur_paramwriter_odom_node1
        #    tms_ur_cv_odom_node1,
         #   tms_ur_cv_odom_node2,
          #  tms_ur_cv_joints_node1,
           # tms_ur_cv_joints_node2,
        #    tms_ur_cv_odom_node3,
         #   tms_ur_cv_odom_node4,
          #  tms_ur_cv_odom_node5,
        ]
    )
