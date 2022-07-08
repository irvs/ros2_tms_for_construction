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
    output_pointcloud2 = DeclareLaunchArgument(
        'output/pointcloud2', default_value='output/pointcloud2'
    )
    
    latest = DeclareLaunchArgument(
        'latest', default_value='False'
    )
    file_name = DeclareLaunchArgument(
        'file_name', default_value='file_name'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
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
    tms_ur_construction_terrain_node = Node(
        package='tms_ur_construction',
        executable='tms_ur_construction_terrain',
        output='screen',
        remappings=[
            ('~/output/pointcloud2', LaunchConfiguration('output/pointcloud2')),
        ],
        parameters=[{
            'file_name': LaunchConfiguration('file_name'),
            'voxel_size': LaunchConfiguration('voxel_size'),
        }]
    )

    return LaunchDescription([
        output_occupancy_grid,
        output_odom,
        output_pointcloud2,
        latest,
        file_name,
        voxel_size,
        tms_ur_construction_ground_node,
        tms_ur_cv_odom_node,
        tms_ur_construction_terrain_node,
    ])
