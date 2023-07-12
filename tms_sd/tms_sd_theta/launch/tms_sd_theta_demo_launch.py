from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tms_sd_theta_compressed_node1 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client1/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod1",
            },
        ],
    )
    tms_sd_theta_compressed_node2 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client2/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod2",
            },
        ],
    )
    tms_sd_theta_compressed_node3 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client3/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod3",
            },
        ],
    )
    tms_sd_theta_compressed_node4 = Node(
        package="tms_sd_theta",
        executable="tms_sd_theta_compressed",
        output="screen",
        remappings=[
            ("~/input/theta/compressed", "/client4/compressed"),
        ],
        parameters=[
            {
                "theta_name": "sensorpod4",
            },
        ],
    )

    return LaunchDescription(
        [
            tms_sd_theta_compressed_node1,
            tms_sd_theta_compressed_node2,
            tms_sd_theta_compressed_node3,
            tms_sd_theta_compressed_node4,
        ]
    )
