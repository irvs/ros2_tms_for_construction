from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_pointcloud2 = DeclareLaunchArgument(
        'input/pointcloud2', default_value='input/pointcloud2'
    )
    file_name = DeclareLaunchArgument(
        'file_name', default_value='file_name'
    )

    tms_ss_terrain_node = Node(
        package='tms_ss_terrain',
        executable='tms_ss_terrain',
        output='screen',
        remappings=[
            ('~/input/pointcloud2', LaunchConfiguration('input/pointcloud2')),
        ],
        parameters=[{
            'file_name': LaunchConfiguration('file_name'),
        }]
    )

    return LaunchDescription([
        input_pointcloud2,
        file_name,
        tms_ss_terrain_node,
    ])
