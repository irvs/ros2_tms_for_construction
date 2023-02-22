from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes
    terrain_mesh_cli_pub_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_mesh_test',
        output='screen',
    )
    terrain_static_cli_pub_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_static_test',
        output='screen',
    )

    return LaunchDescription([
        terrain_mesh_cli_pub_node,
        terrain_static_cli_pub_node,
    ])