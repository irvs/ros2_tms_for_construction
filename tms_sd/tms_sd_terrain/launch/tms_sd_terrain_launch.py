from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_terrain_static_pointcloud2 = DeclareLaunchArgument(
        'input/terrain/static/pointcloud2', default_value='input/terrain/static/pointcloud2'
    )
    filename = DeclareLaunchArgument(
        'filename', default_value='filename'
    )

    tms_sd_terrain_node = Node(
        package='tms_sd_terrain',
        executable='tms_sd_static_terrain',
        output='screen',
        remappings=[
            ('~/input/terrain/static/pointcloud2', LaunchConfiguration('input/terrain/static/pointcloud2')),
        ],
        parameters=[{
            'filename': LaunchConfiguration('filename'),
        }]
    )

    return LaunchDescription([
        input_terrain_static_pointcloud2,
        filename,
        tms_sd_terrain_node,
    ])
