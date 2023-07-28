import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    params = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "params.yaml"
    )
    config_file = os.path.join(
        get_package_share_directory("tms_tf_gui"), "config", "tf_config.json"
    )

    # Nodes
    ground_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="ground_tf",
        executable="ground_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": config_file}, params],
    )

    machine_odom_tf_broadcaster_node = Node(
        package="tms_tf_gui",
        namespace="machine_odom_tf",
        executable="machine_odom_tf_broadcaster",
        output="screen",
        parameters=[{"config_file": config_file}, params],
    )

    return LaunchDescription(
        [
            ground_tf_broadcaster_node,
            machine_odom_tf_broadcaster_node,
        ]
    )
