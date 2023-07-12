from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_theta_compressed = DeclareLaunchArgument(
        "input/theta/compressed", default_value="input/theta/compressed"
    )
    theta_name = DeclareLaunchArgument("theta_name", default_value="theta_name")

    tms_sd_theta_compressed_node = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", LaunchConfiguration("input/theta/compressed")),
        ],
        parameters=[
            {
                "theta_name": LaunchConfiguration("theta_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            input_theta_compressed,
            theta_name,
            tms_sd_theta_compressed_node,
        ]
    )
