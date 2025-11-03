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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

      return LaunchDescription([

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager_NextTS',
                  output='screen',
                  namespace='ic120'),

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager_NextTS',
                  output='screen',
                  namespace='zx200'),
            
            
            # subtasks
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_excavator_change_pose',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_excavator_excavate_simple',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_excavator_excavate_simple_plan',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_excavator_release_simple',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_excavator_follow_waypoints',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_excavator_navigate_anywhere',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_excavator_navigate_through_poses',
                  output='screen',
                  namespace = 'zx200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            
            # sample ###
            #ic120用
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_crawlerdump_follow_waypoints',
                  output='screen',
                  namespace='ic120',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_crawlerdump_navigate_anywhere',
                  output='screen',
                  namespace='ic120',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_crawlerdump_navigate_through_poses',
                  output='screen',
                  namespace='ic120',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_crawlerdump_release_soil',
                  output='screen',
                  namespace='ic120',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),

            # tms_ifを仲介する場合はコメントアウトを解除
            # Subtask Nodeのaction client名を変更すること

            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_follow_waypoints',
            #       output='screen',
            #       namespace='ic120'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_follow_waypoints',
            #       output='screen',
            #       namespace='ic120'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_through_poses',
            #       output='screen',
            #       namespace='ic120'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_anywhere',
            #       output='screen',
            #       namespace='ic120'),


            # mst2200用
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_follow_waypoints',
                  output='screen',
                  namespace='mst2200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_navigate_anywhere',
                  output='screen',
                  namespace='mst2200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_navigate_through_poses',
                  output='screen',
                  namespace='mst2200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_release_soil',
                  output='screen',
                  namespace='mst2200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package = 'tms_ts_subtask',
                  executable='subtask_crawlerdump_swing_align_to_heading',
                  output='screen',
                  namespace='mst2200',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ), 

            # tms_ifを仲介する場合はコメントアウトを解除
            # Subtask Nodeのaction client名を変更すること

            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_follow_waypoints',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_through_poses',
            #       output='screen',
            #       namespace='mst2200'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_anywhere',
            #       output='screen',
                  # namespace='mst2200'),


            #MST110CR
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_follow_waypoints',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_navigate_anywhere',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_navigate_through_poses',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_crawlerdump_release_soil',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),      
            Node(
                  package = 'tms_ts_subtask',
                  executable='subtask_crawlerdump_swing',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ), 
            Node(
                  package = 'tms_ts_subtask',
                  executable='subtask_crawlerdump_swing_align_to_heading',
                  output='screen',
                  namespace='mst110cr_2',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ), 
            
            # tms_ifを仲介する場合はコメントアウトを解除
            # Subtask Nodeのaction client名を変更すること

            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_follow_waypoints',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_through_poses',
            #       output='screen',
            #       namespace='mst110cr_2'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_anywhere',
            #       output='screen',
            #       namespace='mst110cr_2'),

            
            #D37PXI
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_bulldozer_follow_waypoints',
                  output='screen',
                  namespace='d37pxi_24',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_bulldozer_navigate_anywhere',
                  output='screen',
                  namespace='d37pxi_24',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_bulldozer_navigate_through_poses',
                  output='screen',
                  namespace='d37pxi_24',
                  parameters=[{"db_parameter": "official_subtask_parameters"}] ), 
            
            # tms_ifを仲介する場合はコメントアウトを解除
            # Subtask Nodeのaction client名を変更すること

            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_follow_waypoints',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_through_poses',
            #       output='screen',
            #       namespace='d37pxi_24'),
            # Node(
            #       package='tms_if_for_opera',
            #       executable='navigation2_navigate_anywhere',
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
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_follow_waypoints',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_follow_waypoints_deg',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_through_poses',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_through_poses_deg',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_navigate_through_poses_deg',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_change_pose',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_excavate_simple',
            #       output='screen',
            # ),
            # Node(
            #       package='tms_sp_sensing', 
            #       executable='tms_sp_release_simple',
            #       output='screen',
            # ),

            # TMS_DB

            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_task",
                  output='screen',
                  parameters=[{"db_collection": "official_subtasks"}]
                  ),
            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_param",
                  output='screen'
                  ),
      ])
