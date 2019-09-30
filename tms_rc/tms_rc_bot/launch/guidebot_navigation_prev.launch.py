import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration('map', 
                                default=os.path.join(get_package_share_directory('tms_rc_bot'), 'map', 'map.yaml'))

    map_yaml_file = LaunchConfiguration('map')

    # bt_navigator_xml=os.path.join(get_package_share_directory('tms_rc_bot'), 'behavior_trees', 'navigate_w_recovery_retry.xml')

    params_file = 'guidebot_params.yaml'
    params_file_dir = LaunchConfiguration('params', 
                                default=os.path.join(get_package_share_directory('tms_rc_bot'), 'param', params_file))
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    # bt_navigator_install_path = get_package_prefix('nav2_bt_navigator')
    # bt_navigator_xml = os.path.join(bt_navigator_install_path,
    #                                 'behavior_trees',
    #                                 'navigate_w_recovery_retry.xml') # TODO(mkhansen): change to an input parameter

    rviz_config_dir = os.path.join(get_package_share_directory('tms_rc_bot'), 'rviz', 'tb3_navigation2.rviz')

    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

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

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_1st_launch.py']),
        #     launch_arguments={'map': map_dir, 'use_sim_time': use_sim_time}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_2nd_launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time, 'params': param_dir}.items(),
        # ),

        # Node(
        #     package='tf2_ros', node_executable='static_transform_publisher',node_name='tf_imu', output='screen',                       
        #     arguments=['0', '-0.3', '0.52', '-1.570796327', '0', '1.570796327', 'base_link', 'imu_link']            		
        #     ),
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{' use_sim_time': use_sim_time}],
            arguments=[urdf]
            ),

        # Node(
        #     package='guidebot_node',
        #     node_executable='guidebot_odom',
        #     node_name='guidebot_node',
        #     output='screen'
        #     ),

        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_dir}]
            ),

        # Node(
        #     package='robot_localization', 
        #     node_executable='se_node', 
        #     node_name='ekf_localization_node',
        #     output='screen',
        #     parameters=[params_file_dir],
        #     remappings=[('/set_pose', '/initialpose')]
        #    ),       

        # Node(
        #     package='nav2_amcl',
        #     node_executable='amcl',
        #     node_name='amcl',
        #     output='screen',
        #     parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[params_file_dir]
            ),

        Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[params_file_dir],
             remappings=[('/cmd_vel', '/vel')]
            ),

        Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/amcl_pose', '/initialpose'), ('/cmd_vel', '/vel')]
            ),

        Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]
            ),

        Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]
            ),

        # Node(
        #     package='nav2_motion_primitives',
        #     node_executable='motion_primitives_node',
        #     node_name='motion_primitives',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     remappings=[('/amcl_pose', '/odom'), ('/cmd_vel', '/hapirobo/cmd_vel')]
        #     ),

        # Node(
        #     package='nav2_bt_navigator',
        #     node_executable='bt_navigator',
        #     node_name='bt_navigator',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}, {'bt_xml_filename': bt_navigator_xml}]
        #     ),
    
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
            ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/world_model', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/global_costmap/global_costmap', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/local_costmap/local_costmap', 'use_sim_time', use_sim_time],
            output='screen')
    ])