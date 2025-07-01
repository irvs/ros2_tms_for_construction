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

            DeclareLaunchArgument('task_id', default_value="44"),

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager',
                  output='screen'),
            Node(
                  package='tms_ur_button_input', 
                  executable='tms_ur_button',
                  output='screen', 
                  parameters=[{"task_id": LaunchConfiguration('task_id')}]), # You must define task_id that you want to execute. Default task_id is 2.
            
            
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
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_release_soil',
                  output='screen'),
            

            # mst2200用
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_follow_waypoints_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_follow_waypoints',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_navigate_anywhere_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_navigate_anywhere',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_navigate_through_poses_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_navigate_through_poses',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_release_soil',
                  output='screen'),

            #MST110CR
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_follow_waypoints_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_follow_waypoints',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_navigate_anywhere_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_navigate_anywhere',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_navigate_through_poses_deg',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_navigate_through_poses',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst110cr_release_soil',
                  output='screen'),            

            

            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_navigate_anywhere',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_navigate_anywhere_deg',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_follow_waypoints',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_follow_waypoints_deg',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_navigate_through_poses',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_navigate_through_poses_deg',
                  output='screen',
            ),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_navigate_through_poses_deg',
                  output='screen',
            ),
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
      ])
