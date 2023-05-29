from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    output_ground_mesh = DeclareLaunchArgument(
        "output/ground_mesh", default_value="output/ground_mesh"
    )
    timer_period = DeclareLaunchArgument("timer_period", default_value="10")

    # Nodes
    tms_ur_ground_mesh_node = Node(
        package="tms_ur_construction",
        executable="tms_ur_ground_mesh",
        output="screen",
        remappings=[
            ("~/output/ground_mesh", LaunchConfiguration("output/ground_mesh")),
        ],
        parameters=[
            {
                "timer_period": LaunchConfiguration("timer_period"),
            }
        ],
    )

    return LaunchDescription(
        [
            output_ground_mesh,
            timer_period,
            tms_ur_ground_mesh_node,
        ]
    )
