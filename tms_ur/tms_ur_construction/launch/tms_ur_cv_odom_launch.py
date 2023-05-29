from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    output_odom = DeclareLaunchArgument("output/odom", default_value="output/odom")
    latest = DeclareLaunchArgument("latest", default_value="False")
    machine_name = DeclareLaunchArgument("machine_name", default_value="machine_name")

    # Nodes
    tms_ur_cv_odom_node = Node(
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", LaunchConfiguration("output/odom")),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": LaunchConfiguration("machine_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            output_odom,
            latest,
            machine_name,
            tms_ur_cv_odom_node,
        ]
    )
