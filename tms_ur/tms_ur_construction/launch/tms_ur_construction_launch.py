from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    output_occupancy_grid = DeclareLaunchArgument(
        'output/occupancy_grid', default_value='output/occupancy_grid'
    )
    output_odom = DeclareLaunchArgument(
        'output/odom', default_value='output/odom'
    )
    output_terrain_dynamic_pointcloud2 = DeclareLaunchArgument(
        'output/terrain/dynamic/pointcloud2', default_value='output/terrain/dynamic/pointcloud2'
    )
    
    latest = DeclareLaunchArgument(
        'latest', default_value='False'
    )

    # Nodes
    tms_ur_construction_ground_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_ground',
        output='screen',
        remappings=[
            ('~/output/occupancy_grid', LaunchConfiguration('output/occupancy_grid')),
        ],
        parameters=[{
            'latest': LaunchConfiguration('latest'),
        }]
    )
    tms_ur_cv_odom_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_cv_odom',
        output='screen',
        remappings=[
            ('~/output/odom', LaunchConfiguration('output/odom')),
        ],
        parameters=[{
            'latest': LaunchConfiguration('latest'),
        }]
    )
    tms_ur_construction_terrain_dynamic_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain_dynamic',
        output='screen',
        remappings=[
            ('~/output/terrain/dynamic/pointcloud2', LaunchConfiguration('output/terrain/dynamic/pointcloud2')),
        ],
         parameters=[{
            'latest': LaunchConfiguration('latest'),
         }]
    )

    return LaunchDescription([
        output_occupancy_grid,
        output_odom,
        output_terrain_dynamic_pointcloud2,
        latest,
        tms_ur_construction_ground_node,
        tms_ur_cv_odom_node,
        tms_ur_construction_terrain_dynamic_node,
    ])
