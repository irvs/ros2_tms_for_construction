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
import launch_ros.actions

def generate_launch_description():
      return LaunchDescription([
            launch_ros.actions.Node(
            package='tms_ts_manager', executable='tms_ts_manager_action',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_ts_subtask', executable='subtask_nodes_sample_construction',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_ur_text_recognizer', executable='tms_ur_text_recognizer_action',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_ur_button_input', executable='tms_ur_button',
            output='screen'),
            launch_ros.actions.Node(
            package="tms_db_manager", executable="tms_db_reader_task",
            output='screen'),
      ])
