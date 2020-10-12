import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import launch
import launch_ros.actions
#import xacro

share_dir_path = os.path.join(get_package_share_directory('tms_rc_qurin_support'))
#urdf_path = os.path.join(share_dir_path, 'urdf', 'collidor_928_and_957.urdf')
urdf_path = os.path.join(share_dir_path, 'urdf', 'model.urdf')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration('map', 
                                default=os.path.join(
                                    get_package_share_directory('tms_rc_qurin_support'), 
                                    'maps', 'map_w2_9f.yaml'))    
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    autostart = LaunchConfiguration('autostart')

    params_file = 'guidebot_params.yaml'
    params_file_dir = LaunchConfiguration(
        'params', 
        default=os.path.join(get_package_share_directory('tms_rc_qurin_support'), 'params', params_file))
    
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

    declare_params_file_cmd = DeclareLaunchArgument(
        'params', 
        default_value=os.path.join(get_package_share_directory('tms_rc_qurin_support'), 'params', params_file),
        description='Full path to param file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(get_package_share_directory('tms_rc_qurin_support'), 'maps', 'map_w2_9f.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(get_package_share_directory('tms_rc_qurin_support'),
            'behavior_trees', 'navigate_w_replanning_without_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[configured_params])

    start_map_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        node_executable='map_server',
        node_name='map_server',
        output='both',
        parameters=[configured_params])
    

    print(params_file_dir)
    start_localizer_cmd = launch_ros.actions.Node(
        package='robot_localization', 
        node_executable='se_node', 
        # node_name='se_node',
        output='screen',
        parameters=[params_file_dir],
        remappings=[('/set_pose', '/initialpose')]
       )

    start_guidebot_odometry_cmd = launch_ros.actions.Node(
        package='tms_rc_qurin_support',
        node_executable='guidebot_odometry',
        output='screen'
    )

    start_tfnode_cmd = launch_ros.actions.Node(
        package='tms_rc_tfnode',
        node_executable='odom_tf_node',
        output='screen',
    )

    start_pozyx_local_cmd = launch_ros.actions.Node(
        package='tms_rc_qurin_support',
        node_executable='pozyx_local',
        output='screen'
    )

    # TODO: remap /cmd_vel -> /hapirobo/cmd_vel
    # this node re-publish /cmd_vel with rename
    start_convert = launch_ros.actions.Node(
        package='tms_rc_qurin_support',
        node_executable='convert',
        output='screen'
    )

    # TODO: change robot_base_frame /base_link -> /base_footprint
    # this node publish /base_link = /base_footprint, this is not elegant.
    stf_footprint_baselink = launch_ros.actions.Node(package='tf2_ros',
                                node_executable='static_transform_publisher',
                                output='both',
                                arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"])

    # static_transform_publisher: map -> origin_position
    stf_map_originpos = launch_ros.actions.Node(package='tf2_ros',
                                node_executable='static_transform_publisher',
                                output='both',
                                arguments=["34.6", "2.15", "0", "0", "0", "0", "map", "origin_position"])

    # static_transform_publisher: base_footprint -> pozyx
    # stf_base_footprint_pozyx = launch_ros.actions.Node(package='tf2_ros',
    #                             node_executable='static_transform_publisher',
    #                             output='both',
    #                             arguments=["-0.09", "-0.165", "0.5", "0", "0", "0", "base_footprint", "pozyx"])
    # static_transform_publisher: map -> pozyx
    stf_base_footprint_pozyx = launch_ros.actions.Node(package='tf2_ros',
                                node_executable='static_transform_publisher',
                                output='both',
                                arguments=["26.1", "26.2", "0", "0.092", "0", "0", "map", "pozyx"])

    # static_transform_publisher: base_footprint -> laser
    stf_base_footprint_laser = launch_ros.actions.Node(package='tf2_ros',
                                node_executable='static_transform_publisher',
                                output='both',
                                arguments=["0", "0", "0", "0", "0", "3.14", "base_footprint", "laser"])

    # collidor model: (frame: origin_position)
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  node_executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])

    start_rviz = launch_ros.actions.Node(package='rviz2',
                                    node_executable='rviz2',
                                    output='both',
                                    arguments=['-d',os.path.join(share_dir_path, 'rviz2', 'pozyx_test.rviz') ])

    start_world_model_cmd = launch_ros.actions.Node(
        package='nav2_world_model',
        node_executable='world_model',
        output='screen',
        parameters=[configured_params]
        )

    start_dwb_cmd = launch_ros.actions.Node(
        package='dwb_controller',
        node_executable='dwb_controller',
        output='screen',
        parameters=[configured_params],
        remappings=[('/odom', '/odometry/filtered')]
        ) 
    
    start_planner_cmd = launch_ros.actions.Node(
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        node_name='navfn_planner',
        output='screen',
        parameters=[configured_params],
        remappings=[('/amcl_pose', '/initialpose'), ('/cmd_vel', '/hapirobo/cmd_vel')]
        )

    start_recovery_cmd = launch_ros.actions.Node(
        package='nav2_recoveries',
        node_executable='recoveries_node',
        node_name='recoveries',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/amcl_pose', '/initialpose'), ('/odom', '/odometry/filtered')]
        )

    start_navigator_cmd = launch_ros.actions.Node(
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        node_name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    # start_ros1_bridge_cmd = Node(
    #     package='ros1_bridge',
    #     node_executable='dynamic_bridge',
    #     node_name='dynamic_bridge',
    #     output='screen',
    #     parameters=[configured_params])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(stf_map_originpos)
    ld.add_action(stf_footprint_baselink)
    ld.add_action(stf_base_footprint_pozyx)
    ld.add_action(stf_base_footprint_laser)
    ld.add_action(rsp)
    ld.add_action(start_rviz)
    ld.add_action(start_localizer_cmd)
    ld.add_action(start_guidebot_odometry_cmd)
    ld.add_action(start_tfnode_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_recovery_cmd)
    ld.add_action(start_navigator_cmd)
    # ld.add_action(start_pozyx_local_cmd)
    ld.add_action(start_convert)
    # ld.add_action(start_ros1_bridge_cmd)


    return ld
