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
