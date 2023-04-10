from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    filename = DeclareLaunchArgument(
        'filename', default_value='filename.pcd'
    )
    filename_mesh = DeclareLaunchArgument(
        'filename_mesh', default_value='filename_mesh.ply'
    )
    filename_dem = DeclareLaunchArgument(
        'filename_dem', default_value='filename_dem.npy'
    )
    voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.0'
    )
    octree_depth = DeclareLaunchArgument(
        'octree_depth', default_value='2'
    )
    density_th = DeclareLaunchArgument(
        'density_th', default_value='0.1'
    )
    fill_nan_type = DeclareLaunchArgument(
        'fill_nan_type', default_value=''
    )
    resolution = DeclareLaunchArgument(
        'resolution', default_value='0.1'
    )

    # Nodes
    tms_ss_terrain_static_node = Node(
        package='tms_ss_terrain_static',
        executable='tms_ss_terrain_static',
        output='screen',
        parameters=[{
            'filename': LaunchConfiguration('filename'),
        }]
    )
    tms_ss_terrain_static_mesh_node = Node(
        package='tms_ss_terrain_static',
        executable='tms_ss_terrain_static_mesh',
        output='screen',
        parameters=[{
            'filename_mesh': LaunchConfiguration('filename_mesh'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'octree_depth': LaunchConfiguration('octree_depth'),
            'density_th': LaunchConfiguration('density_th'),
        }]
    )
    tms_ss_terrain_static_dem_node = Node(
        package='tms_ss_terrain_static',
        executable='tms_ss_terrain_static_dem',
        output='screen',
        parameters=[{
            'filename_dem': LaunchConfiguration('filename_dem'),
            'fill_nan_type': LaunchConfiguration('fill_nan_type'),
            'resolution': LaunchConfiguration('resolution'),
        }]
    )

    return LaunchDescription([
        filename,
        filename_mesh,
        filename_dem,
        voxel_size,
        octree_depth,
        density_th,
        fill_nan_type,
        resolution,
        tms_ss_terrain_static_node,
        tms_ss_terrain_static_mesh_node,
        tms_ss_terrain_static_dem_node,
    ])
