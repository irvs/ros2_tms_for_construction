from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    input_points = DeclareLaunchArgument(
        "input/machine_points", default_value="/input/machine_points"
    )
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    # Nodes
    machine_points = Node(
        package="tms_sp_machine_points",
        executable="tms_sp_machine_points",
        output="screen",
        remappings=[
            ("~/input/machine_points", LaunchConfiguration("input/machine_points")),
        ],
        parameters=[
            {"to_frame": LaunchConfiguration("to_frame")},
        ],
    )

    return LaunchDescription([input_points, to_frame, machine_points])
