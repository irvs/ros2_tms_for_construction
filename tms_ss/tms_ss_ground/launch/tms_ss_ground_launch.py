from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_occupancy_grid = DeclareLaunchArgument(
        'input/occupancy_grid', default_value='input/occupancy_grid'
    )
    ground_name = DeclareLaunchArgument(
        'ground_name', default_value='ground_name'
    )

    tms_ss_ground_node = Node(
        package='tms_ss_ground',
        executable='tms_ss_ground',
        output='screen',
        remappings=[
            ('~/input/occupancy_grid', LaunchConfiguration('input/occupancy_grid')),
        ],
        parameters=[{
            'ground_name': LaunchConfiguration('ground_name'),
        }]
    )

    return LaunchDescription([
        input_occupancy_grid,
        ground_name,
        tms_ss_ground_node,
    ])
