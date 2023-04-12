from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    output_terrain_static_pointcloud2 = DeclareLaunchArgument(
        'output/terrain/static/pointcloud2', default_value='output/terrain/static/pointcloud2'
    )

    filename = DeclareLaunchArgument(
        'filename', default_value='filename'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
    )

    # Nodes
    tms_ur_construction_terrain_static_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_static',
        output='screen',
        remappings=[
            ('~/output/terrain/static/pointcloud2', LaunchConfiguration('output/terrain/static/pointcloud2')),
        ],
        parameters=[{
            'latest': LaunchConfiguration('latest'),
        }]
    )

    return LaunchDescription([
        output_terrain_static_pointcloud2,
        filename,
        voxel_size,
        tms_ur_construction_terrain_static_node,
    ])