# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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