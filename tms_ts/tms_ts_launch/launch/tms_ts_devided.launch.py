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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import ast


def generate_launch_description():

      declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


      # 掘削積込の情報管理ノード (tms_sp)を起動するか
      declare_launch_sp_arg = DeclareLaunchArgument(
        'launch_sp',
        default_value='true')

      use_sim_time = LaunchConfiguration('use_sim_time')
      launch_sp = IfCondition(LaunchConfiguration('launch_sp'))

      return LaunchDescription([

            declare_use_sim_time_arg,
            declare_launch_sp_arg,



            # 建機制御用PC - LiDAR用PC 間のDB同期用ブリッジ (config/excavation_loading_sensor_bridge.yaml)
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_sensor_bridge',
                  parameters = [{'use_sim_time': use_sim_time}],
                  output='screen',
                  condition=launch_sp),

            # 積込中のダンプ (LOADING_DUMP)の荷台を掘削機に向ける旋回角 -> loading_pose_vessel
            # 90°固定にするためコメントアウト
            # Node(
            #       package='tms_sp_sensing',
            #       executable='tms_sp_dump_swing_angle',
            #       parameters = [{'use_sim_time': use_sim_time}],
            #       output='screen',
            #       condition=launch_sp),

            # TMS_DB
            Node(
                  package="tms_db_manager",
                  executable="tms_db_reader_task",
                  parameters = [{'use_sim_time': use_sim_time}],
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
                  parameters = [{'use_sim_time': use_sim_time}],
                  output='screen'
                  ),
            

            #for preview
            Node(
                  package="tms_ur_construction",
                  executable="tms_ur_plan_reader",
                  output="screen",
                  remappings=[("planwritten", "/planwritten"),
                              ("~/output/plan", "/output/mst110cr_plan"),
                              ("~/output/joint_plan", "/output/zx200_joint_plan"),
                  ],
                  ),
            Node(
                  package="tms_db_manager",
                  executable="tms_db_reader",
                  output='screen'
                  ),
            Node(
                  package="tms_ur_construction",
                  executable="tms_ur_parameter_viz",
                  output="screen",
                  ),
            Node(
                  package='tms_db_manager',
                  executable='tms_db_writer',
                  output='screen',
                  ),

      ])
