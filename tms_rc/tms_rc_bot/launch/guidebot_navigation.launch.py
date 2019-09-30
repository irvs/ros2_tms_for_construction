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
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('tms_rc_bot'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration('map', 
                                default=os.path.join(get_package_share_directory('tms_rc_bot'), 'maps', 'map.yaml'))

    # map_yaml_file = LaunchConfiguration('map')

    # bt_navigator_xml=os.path.join(get_package_share_directory('tms_rc_bot'), 'behavior_trees', 'navigate_w_recovery_retry.xml')

    params_file = 'guidebot_params.yaml'
    params_file_dir = LaunchConfiguration(
        'params', 
        default=os.path.join(get_package_share_directory('tms_rc_bot'), 'params', params_file))
    
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    autostart = LaunchConfiguration('autostart')

    # nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


    # bt_navigator_install_path = get_package_prefix('nav2_bt_navigator')
    # bt_navigator_xml = os.path.join(bt_navigator_install_path,
    #                                 'behavior_trees',
    #                                 'navigate_w_recovery_retry.xml') # TODO(mkhansen): change to an input parameter

    # rviz_config_dir = os.path.join(get_package_share_directory('tms_rc_bot'), 'rviz', 'tb3_navigation2.rviz')

    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

    stdout_linebuf_envvar = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_dir,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file_dir, rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(get_package_share_directory('tms_rc_bot'), 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params', 
        default_value=os.path.join(get_package_share_directory('tms_rc_bot'), 'params', params_file),
        description='Full path to param file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # declare_bt_xml_cmd = DeclareLaunchArgument(
    #     'bt_xml_file',
    #     default_value=os.path.join(get_package_prefix('nav2_bt_navigator'),
    #         'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
    #     description='Full path to the behavior tree xml file to use')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(get_package_share_directory('tms_rc_bot'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{' use_sim_time': use_sim_time}],
        arguments=[urdf]
        )

    # start_guidebot_node_cmd = Node(
    #     package='guidebot_node',
    #     node_executable='guidebot_odom',
    #     node_name='guidebot_node',
    #     output='screen'
    #     )

    # start_map_server_cmd = Node(
    #     package='nav2_map_server',
    #     node_executable='map_server',
    #     node_name='map_server',
    #     output='screen',
    #     parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_dir}]
    #     )

    start_map_server_cmd = Node(
        package='nav2_map_server',
        node_executable='map_server',
        node_name='map_server',
        output='screen',
        parameters=[configured_params])
    
    # start_localizer_cmd = Node(
    #     package='nav2_amcl',
    #     node_executable='amcl',
    #     node_name='amcl',
    #     output='screen',
    #     parameters=[configured_params])

    # start_localizer_cmd = Node(
        #     package='robot_localization', 
        #     node_executable='se_node', 
        #     node_name='ekf_localization_node',
        #     output='screen',
        #     parameters=[params_file_dir],
        #     remappings=[('/set_pose', '/initialpose')]
        #    )
    
    start_world_model_cmd = Node(
        package='nav2_world_model',
        node_executable='world_model',
        output='screen',
        parameters=[configured_params]
        )

    start_dwb_cmd = Node(
        package='dwb_controller',
        node_executable='dwb_controller',
        output='screen',
        parameters=[configured_params],
        remappings=[('/cmd_vel', '/vel')]
        ) 

    # start_planner_cmd = Node(
    #     package='nav2_navfn_planner',
    #     node_executable='navfn_planner',
    #     node_name='navfn_planner',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/amcl_pose', '/initialpose'), ('/cmd_vel', '/hapirobo/cmd_vel')]
    #     )
    
    start_planner_cmd = Node(
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        node_name='navfn_planner',
        output='screen',
        parameters=[configured_params]
        # remappings=[('/amcl_pose', '/initialpose'), ('/cmd_vel', '/hapirobo/cmd_vel')]
        )

    start_recovery_cmd = Node(
        package='nav2_recoveries',
        node_executable='recoveries_node',
        node_name='recoveries',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/amcl_pose', '/initialpose'), ('/cmd_vel', '/cmd2vel')]
        )

    start_navigator_cmd = Node(
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        node_name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[configured_params])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(start_guidebot_node_cmd)
    ld.add_action(start_map_server_cmd)
    # ld.add_action(start_localizer_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_recovery_cmd)
    ld.add_action(start_navigator_cmd)

    ld.add_action(start_robot_state_publisher_cmd)

    return ld
