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
    
    # static_transform_publisher: map -> origin_position
    stf_map_originpos = launch_ros.actions.Node(package='tf2_ros',
                                node_executable='static_transform_publisher',
                                output='both',
                                arguments=["34.6", "2.15", "0", "0", "0", "0", "map", "origin_position"])

    # collidor model: (frame: origin_position)
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  node_executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(stf_map_originpos)
    ld.add_action(rsp)

    return ld
