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

            DeclareLaunchArgument('task_id1', default_value="8"),
            DeclareLaunchArgument('task_id2', default_value="9"),

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager1_202412',
                  output='screen'),
            Node(
                  package='tms_ur_button_input', 
                  executable='tms_ur_demo202412',
                  output='screen', 
                  parameters=[{"task_id1": LaunchConfiguration('task_id1'), "task_id2": LaunchConfiguration('task_id2')}]), 

            Node(
                  package='tms_ts_manager',
                  executable='task_schedular_manager2_202412',
                  output='screen'),
            
            
            # subtasks
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_change_pose',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_excavate_simple',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='subtask_zx200_release_simple',
                  output='screen'),
            
            # sample ###
            #ic120用
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_follow_waypoints_deg_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_follow_waypoints_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere_deg_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_anywhere_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses_deg_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_navigate_through_poses_server',
                  output='screen'),
            Node(
                  package='tms_ts_subtask',
                  executable='subtask_ic120_release_soil_server',
                  output='screen'),
            
            # zx120用
            Node(
                  package='tms_ts_subtask', 
                  executable='zx120_sample_boom_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx120_sample_swing_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx120_sample_arm_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx120_sample_bucket_subtask',
                  output='screen'),
            
            # zx200用
            Node(
                  package='tms_ts_subtask', 
                  executable='zx200_sample_boom_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx200_sample_swing_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx200_sample_arm_subtask',
                  output='screen'),
            Node(
                  package='tms_ts_subtask', 
                  executable='zx200_sample_bucket_subtask',
                  output='screen'),
            
            # センシング処?��?後�??��?��?ータをデータベ�??��スに取り込むためのノ�??��ド�?
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_zx200_end_effector',
                  output='screen'),
            Node(
                  package='tms_sp_sensing', 
                  executable='sample',
                  output='screen'),
            Node(
                  package='tms_sp_sensing', 
                  executable='tms_sp_flgs',
                  output='screen'
            ),
            Node(
                  package="tms_db_manager", 
                  executable="tms_db_reader_task",
                  output='screen'),
      ])
