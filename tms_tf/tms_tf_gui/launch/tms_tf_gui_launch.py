import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    params = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "params.yaml"
    )
    default_config_file = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "tf_config.json"
    )
    config_file = DeclareLaunchArgument(
        "config_file", default_value=default_config_file
    )

    # Nodes
    ground_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="ground_tf",
        executable="ground_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}, params],
    )

    machine_odom_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="machine_odom_tf",
        executable="machine_odom_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}, params],
    )

    return LaunchDescription(
        [
            config_file,
            ground_tf_broadcaster_node,
            machine_odom_tf_broadcaster_node,
        ]
    )
