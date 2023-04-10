from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    output_dem = DeclareLaunchArgument(
        'output/terrain/dem_srv', default_value='output/terrain/dem_srv'
    )
    filename_dem = DeclareLaunchArgument(
        'filename_dem', default_value='filename_dem.npy'
    )
    resolution = DeclareLaunchArgument(
        'resolution', default_value='0.1'
    )

    # Nodes
    tms_ur_construction_terrain_dem_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_dem',
        output='screen',
        remappings=[
            ('~/output/terrain/dem_srv', LaunchConfiguration('output/terrain/dem_srv')),
        ],
        parameters=[{
            'filename_dem': LaunchConfiguration('filename_dem'),
            'resolution': LaunchConfiguration('resolution')
        }]
    )

    return LaunchDescription([
        output_dem,
        filename_dem,
        resolution,
        tms_ur_construction_terrain_dem_node,
    ])
