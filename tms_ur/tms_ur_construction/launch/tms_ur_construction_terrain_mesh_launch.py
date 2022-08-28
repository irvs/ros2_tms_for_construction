from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    output_mesh = DeclareLaunchArgument(
        'output/mesh', default_value='output/mesh'
    )
    filename = DeclareLaunchArgument(
        'filename', default_value='filename'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
    )
    alpha = DeclareLaunchArgument(
        'alpha', default_value='1.0'
    )

    # Nodes
    tms_ur_construction_terrain_mesh_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_mesh',
        output='screen',
        remappings=[
            ('~/output/mesh', LaunchConfiguration('output/mesh')),
        ],
        parameters=[{
            'filename': LaunchConfiguration('filename'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'alpha': LaunchConfiguration('alpha'),
        }]
    )

    return LaunchDescription([
        output_mesh,
        filename,
        voxel_size,
        alpha,
        tms_ur_construction_terrain_mesh_node,
    ])
