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

def generate_launch_description():
  
  start_tms_db_writer_cmd = Node(
      package='tms_db_manager',
      node_executable='tms_db_writer',
      node_name='tms_db_writer',
      output='screen'
      )

  start_tms_db_reader_cmd = Node(
      package='tms_db_manager',
      node_executable='tms_db_writer',
      node_name='tms_db_writer',
      output='screen'
      )


# Create the launch description and populate
ld = LaunchDescription()
ld.add_action(start_tms_db_wtiter_cmd)
ld.add_action(start_tms_db_reader_cmd)
return ld