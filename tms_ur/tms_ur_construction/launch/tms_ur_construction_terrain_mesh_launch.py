from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    output_mesh = DeclareLaunchArgument(
        'output/terrain/mesh_srv', default_value='output/terrain/mesh_srv'
    )
    filename_mesh = DeclareLaunchArgument(
        'filename_mesh', default_value='filename_mesh'
    )

    # Nodes
    tms_ur_construction_terrain_mesh_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_mesh',
        output='screen',
        remappings=[
            ('~/output/terrain/mesh_srv', LaunchConfiguration('output/terrain/mesh_srv')),
        ],
        parameters=[{
            'filename_mesh': LaunchConfiguration('filename_mesh'),
        }]
    )

    return LaunchDescription([
        output_mesh,
        filename_mesh,
        tms_ur_construction_terrain_mesh_node,
    ])
