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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

      tms_if_for_opera_dir = get_package_share_directory("tms_if_for_opera")

      tms_if_for_opera_excavator_path = os.path.join(
        tms_if_for_opera_dir, "launch", "tms_if_for_opera_excavator.launch.py"
      )

      tms_if_for_opera_crawlerdump_path = os.path.join(
        tms_if_for_opera_dir, "launch", "tms_if_for_opera_crawlerdump.launch.py"
      )

      tms_if_for_opera_bulldozer_path = os.path.join(
        tms_if_for_opera_dir, "launch", "tms_if_for_opera_bulldozer.launch.py"
      )

      declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
      
      declare_task_id_arg = DeclareLaunchArgument(
        'task_id',
        default_value='44')

      return LaunchDescription([

            declare_use_sim_time_arg,
            declare_task_id_arg,

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  output='screen'),
            Node(
                  package='tms_ur_button_input', 
                  executable='tms_ur_button',
                  output='screen', 
                  parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time'), 
                               "task_id": LaunchConfiguration('task_id')}]), # You must define task_id that you want to execute. Default task_id is 2.
            
            
            # primitives
            Node(
                  package='tms_ts_primitive', 
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),
            
            Node(
                  package='tms_ts_primitive', 
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='zx200'),

            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_excavator_change_pose_plan',
                  output='screen',
                  namespace = 'zx200'),   
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_change_pose_execute_from_plan',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_change_pose_execute_from_plan_retime',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='excavator_scene_manager',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"model_name": "zx200", "root_record_name": "collision_objects_shimiz", "planning_frame": "base_link", "visualization_record_name": "target_excavate_area"}]),
            
            # sample ###
            #ic120用
            # Node(
            #       package='tms_ts_primitive',
            #       executable='primitive_crawlerdump_follow_waypoints',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='ic120'),
            # Node(
            #       package='tms_ts_primitive',
            #       executable='primitive_crawlerdump_navigate_anywhere',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='ic120'),
            # Node(
            #       package='tms_ts_primitive',
            #       executable='primitive_crawlerdump_navigate_through_poses',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='ic120'),
            # Node(
            #       package='tms_ts_primitive',
            #       executable='primitive_crawlerdump_release_soil',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='ic120'),
            

            # mst2200用
            # Node(
            #       package='tms_ts_primitive', 
            #       executable='primitive_crawlerdump_follow_waypoints',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_primitive', 
            #       executable='primitive_crawlerdump_navigate_anywhere',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_primitive', 
            #       executable='primitive_crawlerdump_navigate_through_poses',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='mst2200'),
            # Node(
            #       package='tms_ts_primitive', 
            #       executable='primitive_crawlerdump_release_soil',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='mst2200'),
            # Node(
            #       package = 'tms_ts_primitive',
            #       executable='primitive_crawlerdump_swing_align_to_heading',
            #       output='screen',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       namespace='mst2200'), 

            #MST110CR
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_crawlerdump_follow_path',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_crawlerdump_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),       
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_crawlerdump_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_crawlerdump_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_crawlerdump_release_soil',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'),      
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'), 
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing_align_to_heading',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='mst110cr'), 
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_compute_path_to_pose',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                 'output_model_name': 'mst110cr',
                                 'output_record_name': 'computed_path_autogenerated'}],
                  namespace='mst110cr',
                  remappings=[('planwritten', '/planwritten')]
                  ), 
            
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(tms_if_for_opera_crawlerdump_path),
            #     launch_arguments={
            #         'robot_name': 'mst110cr',
            #         'use_sim_time': LaunchConfiguration('use_sim_time')
            #     }.items(),
            # ),

            
            #D37PXI
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_blade_control',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='d37pxi_24'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='d37pxi_24'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='d37pxi_24'),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace='d37pxi_24'),     

            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(tms_if_for_opera_bulldozer_path),
            #     launch_arguments={
            #         'robot_name': 'd37pxi_24',
            #         'use_sim_time': LaunchConfiguration('use_sim_time')
            #     }.items(),
            # ),            

            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_anywhere',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_follow_waypoints',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_through_poses',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_change_pose',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_excavate_simple',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_release_simple',
            #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            #       output='screen',
            # ),

            # TMS_DB

            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_task",
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  output='screen'
                  ),
            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_subtask",
                  output='screen'
                  ),
            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_param",
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  output='screen'
                  ),
      ])
