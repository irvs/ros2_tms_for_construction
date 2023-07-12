from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    output_theta_compressed = DeclareLaunchArgument(
        "output/theta/compressed", default_value="output/theta/compressed"
    )
    latest = DeclareLaunchArgument("latest", default_value="False")
    theta_name = DeclareLaunchArgument("theta_name", default_value="theta_name")

    # Nodes
    tms_ur_construction_theta_compressed = Node(
        package="tms_ur_construction",
        executable="tms_ur_construction_theta_compressed",
        output="screen",
        remappings=[
            (
                "~/output/theta/compressed",
                LaunchConfiguration("output/theta/compressed"),
            ),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "theta_name": LaunchConfiguration("theta_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            output_theta_compressed,
            latest,
            theta_name,
            tms_ur_construction_theta_compressed,
        ]
    )
