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
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", "/cd_0/odom"),
        ],
        parameters=[
            {
                "machine_name": "backhow1",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )
    tms_sp_machine_odom_node2 = Node(
        name="tms_sp_machine_odom2",
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", "/cd_1/odom"),
        ],
        parameters=[
            {
                "machine_name": "wheel_loader1",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )
    tms_sp_machine_odom_node3 = Node(
        name="tms_sp_machine_odom3",
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", "/cd_2/odom"),
        ],
        parameters=[
            {
                "machine_name": "crawler_dump1",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )
    tms_sp_machine_odom_node4 = Node(
        name="tms_sp_machine_odom4",
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", "/cd_3/odom"),
        ],
        parameters=[
            {
                "machine_name": "crawler_dump2",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )
    tms_sp_machine_odom_node5 = Node(
        name="tms_sp_machine_odom5",
        package="tms_sp_machine_odom",
        executable="tms_sp_machine_odom",
        output="screen",
        remappings=[
            ("~/input/odom", "/cd_4/odom"),
        ],
        parameters=[
            {
                "machine_name": "crawler_dump3",
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            to_frame,
            tms_sp_machine_odom_node1,
            tms_sp_machine_odom_node2,
            tms_sp_machine_odom_node3,
            tms_sp_machine_odom_node4,
            tms_sp_machine_odom_node5,
        ]
    )
