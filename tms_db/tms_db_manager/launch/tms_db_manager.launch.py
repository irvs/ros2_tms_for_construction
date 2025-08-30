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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
      db_host = DeclareLaunchArgument(
            'db_host', default_value='localhost'
      )
      db_port = DeclareLaunchArgument(
            'db_port', default_value='27017'
      )
      init_db = DeclareLaunchArgument(
            'init_db', default_value='False'
      )

      tms_db_writer_node = Node(
            package='tms_db_manager',
            executable='tms_db_writer',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
                  'init_db': LaunchConfiguration('init_db'),
            }]
      )
      # tms_db_writer_gridfs_node = Node(
      #       package='tms_db_manager',
      #       executable='tms_db_writer_gridfs',
      #       output='screen',
      #       parameters=[{
      #             'db_host': LaunchConfiguration('db_host'),
      #             'db_port': LaunchConfiguration('db_port'),
      #       }]
      # )
      tms_db_param_writer_node = Node(
            package='tms_db_manager',
            executable='tms_db_writer_param',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
                  'init_db': LaunchConfiguration('init_db'),
            }]
      )
      tms_db_param_writer_node = Node(
            package='tms_db_manager',
            executable='tms_db_writer_param',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
                  'init_db': LaunchConfiguration('init_db'),
            }]
      )
      tms_db_reader_node = Node(
            package='tms_db_manager',
            executable='tms_db_reader',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
     # tms_db_reader_gridfs_node = Node(
      tms_db_reader_heightmap_node = Node(
            package='tms_db_manager',
         #   executable='tms_db_reader_gridfs',
            executable='tms_db_reader_heightmap',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
      tms_db_param_reader_node = Node(
            package='tms_db_manager',
            executable='tms_db_reader_param',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
      return LaunchDescription([
            db_host,
            db_port,
            init_db,
            tms_db_writer_node,
            tms_db_writer_gridfs_node,
            tms_db_param_writer_node,
            tms_db_reader_node,
           # tms_db_reader_gridfs_node,
            tms_db_reader_heightmap_node,
            tms_db_param_reader_node,
      ])
