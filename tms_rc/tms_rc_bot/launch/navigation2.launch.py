import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('tms_rc_bot'),
            'maps',
            'map.yaml'))
    
    params_file = 'guidebot_params.yaml'
    params_file_dir = LaunchConfiguration(
        'params', 
        default=os.path.join(get_package_share_directory('tms_rc_bot'), 'params', params_file))

    # param_file_name = TURTLEBOT3_MODEL + '.yaml'
    # params_file_dir = LaunchConfiguration(
    #     'params',
    #     default=os.path.join(
    #         get_package_share_directory('turtlebot3_navigation2'),
    #         'param',
    #         param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('tms_rc_bot'), 'launch')
    # nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('tms_rc_bot'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=params_file_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/guidebot_bringup.launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params': params_file_dir}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params': params_file_dir}.items(),
        # ),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
