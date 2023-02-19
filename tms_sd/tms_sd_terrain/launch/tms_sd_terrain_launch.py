from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_terrain_static_pointcloud2 = DeclareLaunchArgument(
        'input/terrain/static/pointcloud2', default_value='input/terrain/static/pointcloud2'
    )
    input_terrain_dynamic_pointcloud2 = DeclareLaunchArgument(
        'input/terrain/dynamic/pointcloud2', default_value='input/terrain/dynamic/pointcloud2'
    )
    terrain_name = DeclareLaunchArgument(
        'terrain_name', default_value='terrain'
    )

    tms_sd_terrain_static_node = Node(
        package='tms_sd_terrain',
        executable='tms_sd_terrain_static',
        output='screen',
        remappings=[
            ('~/input/terrain/static/pointcloud2', LaunchConfiguration('input/terrain/static/pointcloud2')),
        ],
    )
    tms_sd_terrain_dynamic_node = Node(
        package='tms_sd_terrain',
        executable='tms_sd_terrain_dynamic',
        output='screen',
        remappings=[
            ('~/input/terrain/dynamic/pointcloud2', LaunchConfiguration('input/terrain/dynamic/pointcloud2')),
        ],
        parameters=[{
            'terrain_name': LaunchConfiguration('terrain_name'),
        }]
    )

    return LaunchDescription([
        input_terrain_static_pointcloud2,
        input_terrain_dynamic_pointcloud2,
        terrain_name,
        tms_sd_terrain_static_node,
        tms_sd_terrain_dynamic_node,
    ])
