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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
      
      opera_modules = get_package_share_directory("opera_modules")
      opera_modules_launch_file_path=os.path.join(opera_modules, "launch","opera_modules.launch.py")

      tms_if_for_opera = get_package_share_directory("tms_if_for_opera")
      tms_if_for_opera_launch_file_path = os.path.join(tms_if_for_opera, "launch","tms_if_for_opera.launch.py")

      ic120_unity = get_package_share_directory("ic120_unity")
      ic120_standby_ekf_launch_file_path = os.path.join(ic120_unity, "launch","ic120_standby_ekf.launch.py")

      zx200_bringup = get_package_share_directory("zx200_bringup")
      vehicle_launch_file_path = os.path.join(zx200_bringup, "launch","vehicle.launch.py")

      zx200_unity = get_package_share_directory("zx200_unity")
      zx200_standby_ekf_launch_file_path = os.path.join(zx200_unity, "launch", "zx200_standby_ekf.launch.py")

      declare_cmd_iface = DeclareLaunchArgument(
            'command_interface_name',
            default_value='velocity',
            description='The parameter to define control interface for ZX200 motion planning.'
      )

      declare_use_rviz = DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='The parameter to use RViz visualization for ZX200 motion planning.'
      )

      declare_robot_name_zx200 = DeclareLaunchArgument(
            'robot_name_zx200',
            default_value='zx200',
      )


      cmd_iface = LaunchConfiguration('command_interface_name')
      use_rviz = LaunchConfiguration('use_rviz')
      robot_name_zx200 = LaunchConfiguration('robot_name_zx200')

      return LaunchDescription([
            
            declare_cmd_iface,
            declare_use_rviz,
            declare_robot_name_zx200,

            DeclareLaunchArgument('task_id1', default_value="3"), # The task to control zx200 prepared for demo(2025/8) for FRL 
            DeclareLaunchArgument('task_id2', default_value="4"), # The task to control ic120 prepared for demo(2025/8) for FRL

            # Node(
            #       package='tms_ts_manager',
            #       executable='task_schedular_manager1_202412',
            #       output='screen'),

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager2_202412',
                  output='screen'),

            Node(
                  package='tms_ur_button_input', 
                  executable='tms_ur_demo202412',
                  output='screen', 
                  parameters=[{"task_id1": LaunchConfiguration('task_id1'), "task_id2": LaunchConfiguration('task_id2')}]), 
            
            # subtasks
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_change_pose',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_excavate_simple',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_excavate_simple_plan',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_release_simple',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_follow_waypoints_deg',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_follow_waypoints',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_navigate_anywhere_deg',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_navigate_anywhere',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_navigate_through_poses_deg',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_zx200_navigate_through_poses',
                  output='screen',
                  namespace = 'zx200'),
            
            # sample ###
            #ic120用
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_follow_waypoints_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_follow_waypoints',
                  output='screen',
                  namespace='ic120'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere_deg',
                  output='screen',
                  namespace='ic120'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere',
                  output='screen',
                  namespace='ic120'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses_deg',
                  output='screen',
                  namespace='ic120'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses',
                  output='screen',
                  namespace='ic120'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_release_soil',
                  output='screen',
                  namespace='ic120'),
            

            # # mst2200用
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_follow_waypoints_deg',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_follow_waypoints',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_navigate_anywhere_deg',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_navigate_anywhere',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_navigate_through_poses_deg',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_navigate_through_poses',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst2200_release_soil',
            #       output='screen',
            #       namespace='mst2200'),

            # #MST110CR
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_follow_waypoints_deg',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_follow_waypoints',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_navigate_anywhere_deg',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_navigate_anywhere',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_navigate_through_poses_deg',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_navigate_through_poses',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_mst110cr_release_soil',
            #       output='screen',
            #       namespace='mst110cr_2'),      
            # Node(
            #       package = 'tms_ts_subtask',
            #       executable='subtask_mst110cr_swing',
            #       output='screen',
            #       namespace='mst110cr_2'), 

            
            # #D37PXI
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_follow_waypoints_deg',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_follow_waypoints',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_navigate_anywhere_deg',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_navigate_anywhere',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_navigate_through_poses_deg',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_ts_subtask', 
            #       executable='subtask_d37pxi_navigate_through_poses',
            #       output='screen',
            #       namespace='d37pxi_24'),     

            

            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_anywhere',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_anywhere_deg',
            #       output='screen',
            # ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_follow_waypoints',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_release_soil',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_flgs',
                  output='screen',
            ),            
            # Node(_launch_file_path
            #       executable='tms_sp_navigate_through_poses_deg',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_through_poses_deg',
            #       output='screen',
            # ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_change_pose',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_excavate_simple',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_release_simple',
                  output='screen',
            ),

            # TMS_DB

            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_task",
                  output='screen'
                  ),
            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_parameter",
                  output='screen'
                  ),

            # OPERA modules

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(opera_modules_launch_file_path),
            ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(tms_if_for_opera_launch_file_path),
            # ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(ic120_standby_ekf_launch_file_path),
            # ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(vehicle_launch_file_path),
            #     launch_arguments={
            #         'command_interface_name': cmd_iface,
            #         'use_rviz': use_rviz,
            #         'robot_name': robot_name_zx200,
            #     }.items(),
            # ),

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(zx200_standby_ekf_launch_file_path),
            # ),

      ])
