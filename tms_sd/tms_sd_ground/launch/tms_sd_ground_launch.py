from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_occupancy_grid = DeclareLaunchArgument(
        "input/occupancy_grid", default_value="input/occupancy_grid"
    )
    ground_name = DeclareLaunchArgument("ground_name", default_value="ground_name")
    to_frame = DeclareLaunchArgument("to_frame", default_value="world")

    tms_sd_ground_node = Node(
        package="tms_sd_ground",
        executable="tms_sd_ground",
        output="screen",
        remappings=[
            ("~/input/occupancy_grid", LaunchConfiguration("input/occupancy_grid")),
        ],
        parameters=[
            {
                "ground_name": LaunchConfiguration("ground_name"),
            },
            {
                "to_frame": LaunchConfiguration("to_frame"),
            },
        ],
    )

    return LaunchDescription(
        [
            input_occupancy_grid,
            ground_name,
            to_frame,
            tms_sd_ground_node,
        ]
    )
