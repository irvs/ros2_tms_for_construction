import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch
import launch_ros.actions
import xacro

share_dir_path = os.path.join(get_package_share_directory('tms_rc_qurin_support'))
urdf_path = os.path.join(share_dir_path, 'urdf', 'collidor_928_and_957.urdf')

def generate_launch_description():
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  node_executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])

    return launch.LaunchDescription([rsp])
