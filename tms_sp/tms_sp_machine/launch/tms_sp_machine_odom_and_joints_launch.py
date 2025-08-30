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
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    # Nodes
    tms_sp_machine_odom_node1 = Node(
        name="tms_sp_machine_odom1",
        package="tms_sp_machine",
        executable="tms_sp_machine_odom.py",
        output="screen",
        remappings=[
            ("~/input/odom", "/Vehicle01/odom"),
          # ("~/input/odom", "/ic120/global_pose"),
        ],
        parameters=[
            {
                "machine_name": "kumagayagumi_crawler",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )
    
    tms_sp_machine_odom_node2 = Node(
        name="tms_sp_machine_odom2",
        package="tms_sp_machine",
        executable="tms_sp_machine_odom.py",
        output="screen",
        remappings=[
            ("~/input/odom", "/global_pose"),
           #("~/input/odom", "/ic120/odometry/global"),
        ],
        parameters=[
            {
                "machine_name": "kumagayagumi_backhoe",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    tms_sp_machine_joints_node1 = Node(
        name="tms_sp_machine_joints1",
        package="tms_sp_machine",
        executable="tms_sp_machine_joints.py",
        output="screen",
        remappings=[
            ("~/input/joint", "/joint_state"),
        ],
        parameters=[
            {
                "machine_name": "kumagayagumi_backhoe",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    tms_sp_machine_odom_node3 = Node(
        name="tms_sp_machine_odom3",
        package="tms_sp_machine",
        executable="tms_sp_machine_odom_yammer.py",
        output="screen",
        remappings=[
            ("~/input/odom", "/cafe/odom"),
          # ("~/input/odom", "/ic120/global_pose"),
        ],
        parameters=[
            {
                "machine_name": "yammer_cafe",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    # tms_sp_machine_joints_node2 = Node(
    #     name="tms_sp_machine_joints2",
    #     package="tms_sp_machine",
    #     executable="tms_sp_machine_joints.py",
    #     output="screen",
    #     remappings=[
    #         ("~/input/joint", "/ic120/joint_states"),
    #     ],
    #     parameters=[
    #         {
    #             "machine_name": "ic120_1",
    #         },
    #         {
    #             "to_frame": LaunchConfiguration("to_frame"),
    #         },
    #     ],
    # )
    
    
    # tms_sp_machine_odom_node3 = Node(
    #     name="tms_sp_machine_odom3",
    #     package="tms_sp_machine",
    #     executable="tms_sp_machine_odom.py",
    #     output="screen",
    #     remappings=[
    #         ("~/input/odom", "/cd_0/status/odom"),
    #     ],
    #     parameters=[
    #         {
    #             "machine_name": "crawler_dump1",
    #         },
    #         {
    #             "to_frame": LaunchConfiguration("to_frame"),
    #         },
    #     ],
    # )
    # tms_sp_machine_odom_node4 = Node(
    #     name="tms_sp_machine_odom4",
    #     package="tms_sp_machine",
    #     executable="tms_sp_machine_odom.py",
    #     output="screen",
    #     remappings=[
    #         ("~/input/odom", "/cd_1/status/odom"),
    #     ],
    #     parameters=[
    #         {
    #             "machine_name": "crawler_dump2",
    #         },
    #         {
    #             "to_frame": LaunchConfiguration("to_frame"),
    #         },
    #     ],
    # )
    # tms_sp_machine_odom_node5 = Node(
    #     name="tms_sp_machine_odom5",
    #     package="tms_sp_machine",
    #     executable="tms_sp_machine_odom.py",
    #     output="screen",
    #     remappings=[
    #         ("~/input/odom", "/cd_2/status/odom"),
    #     ],
    #     parameters=[
    #         {
    #             "machine_name": "crawler_dump3",
    #         },
    #         {
    #             "to_frame": LaunchConfiguration("to_frame"),
    #         },
    #     ],
    # )
    

    return LaunchDescription(
        [
            to_frame,
            tms_sp_machine_odom_node1,
            tms_sp_machine_odom_node2,
            tms_sp_machine_odom_node3,
            tms_sp_machine_joints_node1,
            # tms_sp_machine_joints_node2,
            # tms_sp_machine_odom_node3,
            # tms_sp_machine_odom_node4,
            # tms_sp_machine_odom_node5,
        ]
    )