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

#urdf_path = os.path.join(share_dir_path, 'urdf', 'collidor_928_and_957.urdf')
# urdf_path = os.path.join(share_dir_path, 'urdf', 'model.urdf')

def generate_launch_description():
    params_file = 'room928.yaml'
    params_file_dir = LaunchConfiguration(
        'params', 
        default=os.path.join(get_package_share_directory('pozyx_range_debugger'), 'params', params_file))
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    map_dir = LaunchConfiguration('map', 
        default=os.path.join(
            get_package_share_directory('tms_rc_qurin_support'), 
            'maps', 'map_w2_9f.yaml'))

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(get_package_share_directory('tms_rc_qurin_support'), 'maps', 'map_w2_9f.yaml'),
        description='Full path to map file to load')

    param_substitutions = {
        # 'use_sim_time': use_sim_time,
        'yaml_filename': map_dir,
        # 'bt_xml_filename': bt_xml_file,
        # 'autostart': autostart
    }
    configured_params = RewrittenYaml(
        source_file=params_file_dir, rewrites=param_substitutions,
        convert_types=True)

    start_range_debugger = launch_ros.actions.Node(
        package='pozyx_range_debugger',
        node_executable='debugger',
        output='both',
    )

    stf_map_pozyx = launch_ros.actions.Node(package='tf2_ros',
                            node_executable='static_transform_publisher',
                            output='both',
                            arguments=["26.1", "26.2", "0", "0.092", "0", "0", "map", "pozyx"])

    share_dir_path = os.path.join(get_package_share_directory('pozyx_range_debugger'))
    start_rviz = launch_ros.actions.Node(
        package='rviz2',
        node_executable='rviz2',
        output='both',
        arguments=['-d',os.path.join(share_dir_path, 'rviz2', 'range_debugger_room928.rviz') ])


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
    stf_map_originpos = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='both',
        arguments=["34.6", "2.15", "0", "0", "0", "0", "map", "origin_position"])

    # collidor model: (frame: origin_position)
    share_dir_path = os.path.join(get_package_share_directory('tms_rc_qurin_support'))
    urdf_path = os.path.join(share_dir_path, 'urdf', 'model.urdf')
    rsp = launch_ros.actions.Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='both',
        # argumentsでURDFを出力したパスを指定
        arguments=[urdf_path])
    
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_map_yaml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(stf_map_pozyx)
    ld.add_action(start_range_debugger)
    ld.add_action(start_rviz)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(stf_map_originpos)
    ld.add_action(rsp)


    return ld