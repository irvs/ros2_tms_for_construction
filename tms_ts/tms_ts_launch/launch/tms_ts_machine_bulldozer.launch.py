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

# 2026年11月 実機実験：掘削積込 (excavation_loading)
#   バックホウ zx200 1台と、交互に来るクローラダンプ 2台 (mst110cr, mst2200vd)。
#   建機制御用 PC (MongoDB・BT)で起動する。excavate_pose / release_pose は LiDAR 用 PC で別に起動する。
#   設計: excavation_loading_design.md

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import ast


def launch_setup(context, *args, **kwargs):
      # task_ids を int の配列に変換 ('[44, 45]' / '44' のどちらも受け付ける)
      task_ids = ast.literal_eval(LaunchConfiguration('task_ids').perform(context))
      if isinstance(task_ids, int):
            task_ids = [task_ids]
      task_ids = [int(task_id) for task_id in task_ids]

      use_sim_time = LaunchConfiguration('use_sim_time')
      zmq_server_port_base = int(LaunchConfiguration('zmq_server_port_base').perform(context))
      zmq_publisher_port_base = int(LaunchConfiguration('zmq_publisher_port_base').perform(context))

      nodes = []
      # task_id ごとに task_schedular_manager を起動 (ZMQ のポートは task ごとに 10 ずつずらす)
      for i, task_id in enumerate(task_ids):
            nodes.append(
                  Node(
                        package='tms_ts_manager',
                        executable='task_schedular_manager',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time,
                                     'task_id': task_id,
                                     'zmq_server_port': zmq_server_port_base + (i * 10),
                                     'zmq_publisher_port': zmq_publisher_port_base + (i * 10),
                                     'heartbeat_topic': '/tms_heartbeat',
                                     'heartbeat_rate': 10.0,
                                     }]))

      nodes.append(
            Node(
                  package='tms_ur_button_input',
                  executable='tms_ur_button',
                  output='screen',
                  parameters=[{'use_sim_time': use_sim_time,
                               'task_ids': task_ids}]))

      return nodes


def generate_launch_description():

      declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

      # 実行するタスクの ID の配列。複数指定するとタスクを複数立てる (例: task_ids:='[44, 45]')
      declare_task_ids_arg = DeclareLaunchArgument(
        'task_ids',
        default_value='[44]')

      # 複数台の PC で動かすときの ZMQ のポート (task_id ごとに 10 ずつ増やす)
      declare_zmq_server_port_base_arg = DeclareLaunchArgument(
        'zmq_server_port_base',
        default_value='1666',
        description='Base port number for ZMQ server (will be incremented for each task_id)')

      declare_zmq_publisher_port_base_arg = DeclareLaunchArgument(
        'zmq_publisher_port_base',
        default_value='1777',
        description='Base port number for ZMQ publisher (will be incremented for each task_id)')

      # robot_name の ROS のネームスペース (primitive のアクション名、TF、load_volume に使う)。
      # DB のレコード名は SAMPLE_BLACKBOARD_robot_name (掘削積込の中では robot_name に統一)
      declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='d37pxi')

      use_sim_time = LaunchConfiguration('use_sim_time')
      robot_name = LaunchConfiguration('robot_name')

      return LaunchDescription([

            declare_use_sim_time_arg,
            declare_task_ids_arg,
            declare_zmq_server_port_base_arg,
            declare_zmq_publisher_port_base_arg,
            declare_robot_name_arg,

            # task_schedular_manager (task_id ごと)と tms_ur_button
            OpaqueFunction(function=launch_setup),


            # primitives
            Node(
                  package='tms_ts_primitive',
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            #D37PXI
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_blade_control',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive', 
                  executable='primitive_bulldozer_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                  namespace=robot_name),  

      ])
