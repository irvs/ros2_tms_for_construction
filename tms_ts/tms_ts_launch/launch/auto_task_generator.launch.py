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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tms_ts_subtask_package_dir = get_package_share_directory('tms_ts_subtask')

    return LaunchDescription([

        Node(
            package="tms_ts_manager",
            executable="task_generator.py",
            name="task_generator_from_task_sequence",
            output='screen',
            parameters=[{
                'bt_tree_xml_file_name':'sample_construction_tree_zx120_zx200', # You must put the source xml file in tms_ts/tms_ts_subtask/config.
                "output_text_file_directory_path": tms_ts_subtask_package_dir + '/config',
                "output_file_name":'task_sequence',
                'output_txt_file':False,
                'model_name':'zx120, zx200', # You must define construction machinery model name(zx120, zx200, ic120).
                'description': 'No commented ...'} # Please define task description.
            ],
        ),
    
    ])