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

share_dir_path = os.path.join(get_package_share_directory('pozyx_range_debugger'))
#urdf_path = os.path.join(share_dir_path, 'urdf', 'collidor_928_and_957.urdf')
urdf_path = os.path.join(share_dir_path, 'urdf', 'model.urdf')

def generate_launch_description():
    params_file = 'guidebot_params.yaml'
    params_file_dir = LaunchConfiguration(
        'params', 
        default=os.path.join(get_package_share_directory('pozyx_range_debugger'), 'params', params_file))
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'yaml_filename': map_dir,
    #     'bt_xml_filename': bt_xml_file,
    #     'autostart': autostart
    # }
    # configured_params = RewrittenYaml(
    #     source_file=params_file_dir, rewrites=param_substitutions,
    #     convert_types=True)

    start_range_debugger = launch_ros.actions.Node(
        package='pozyx_range_debugger',
        node_executable='debugger',
        output='both',
    )

    stf_map_originpos = launch_ros.actions.Node(package='tf2_ros',
                            node_executable='static_transform_publisher',
                            output='both',
                            arguments=["0", "0", "0", "0", "0", "0", "map", "pozyx"])

    start_rviz = launch_ros.actions.Node(
        package='rviz2',
        node_executable='rviz2',
        output='both',
        arguments=['-d',os.path.join(share_dir_path, 'rviz2', 'range_debugger.rviz') ])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(stf_map_originpos)
    ld.add_action(start_range_debugger)
    ld.add_action(start_rviz)


    return ld