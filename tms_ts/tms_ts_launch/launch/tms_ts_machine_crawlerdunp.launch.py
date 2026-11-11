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
from launch.substitutions import PythonExpression


def launch_setup(context, *args, **kwargs):

    # task_idsをintの配列に変換
    task_ids = ast.literal_eval(
        LaunchConfiguration('task_ids').perform(context)
    )

    if isinstance(task_ids, int):
        task_ids = [task_ids]

    task_ids = [int(task_id) for task_id in task_ids]

    use_sim_time = LaunchConfiguration('use_sim_time')

    nodes = []

    # task_idごとにtask_schedular_managerを起動
    for task_id in task_ids:

        # task_idからポート番号を決定
        zmq_server_port = 1665 + task_id
        zmq_publisher_port = 1776 + task_id

        nodes.append(
            Node(
                package='tms_ts_manager',
                executable='task_schedular_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'task_id': task_id,
                    'zmq_server_port': zmq_server_port,
                    'zmq_publisher_port': zmq_publisher_port,
                    'heartbeat_topic': '/tms_heartbeat',
                    'heartbeat_rate': 10.0,
                }]
            )
        )

    nodes.append(
        Node(
            package='tms_ur_button_input',
            executable='tms_ur_button',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'task_ids': task_ids
            }]
        )
    )

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

      # robot_name の ROS のネームスペース (primitive のアクション名、TF、load_volume に使う)。
      # DB のレコード名は SAMPLE_BLACKBOARD_robot_name (掘削積込の中では robot_name に統一)
      declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mst110cr_2')

      # 掘削積込の情報管理ノード (tms_sp)を起動するか
      declare_launch_sp_arg = DeclareLaunchArgument(
        'launch_sp',
        default_value='true')

      use_sim_time = LaunchConfiguration('use_sim_time')
      robot_name = LaunchConfiguration('robot_name')
      launch_sp = IfCondition(LaunchConfiguration('launch_sp'))

      return LaunchDescription([

            declare_use_sim_time_arg,
            declare_task_ids_arg,
            declare_robot_name_arg,
            declare_launch_sp_arg,

            # task_schedular_manager (task_id ごと)と tms_ur_button
            OpaqueFunction(function=launch_setup),


            # primitives
            Node(
                  package='tms_ts_primitive',
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            #MST110CR
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_path',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_release_soil',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing_align_to_heading',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=robot_name),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_compute_path_to_pose',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time,
                                 'output_model_name': robot_name,
                                 'output_record_name': 'computed_path_autogenerated'}],
                  namespace=robot_name,
                  remappings=[('planwritten', '/planwritten')]
                  ),

            # 積載量 -> 各ダンプのレコードの SENSING_LOADED_FLG
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_loadsense',
                  name=PythonExpression(["'tms_sp_loadsense_' + '", robot_name, "'"]),
                  parameters = [{'use_sim_time': use_sim_time,
                                 'machine_name': robot_name,
                                 'record_name': PythonExpression(["'SAMPLE_BLACKBOARD_' + '", robot_name, "'"])
                  }],
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


            #for odom
            Node(
                  package="tms_sp_machine",
                  executable="tms_sp_machine_odom.py",
                  output="screen",
                  remappings=[
                        ("~/input/odom", PythonExpression(["'/' + '", robot_name, "' + '/odometry/global'"])),
                  ],
                  parameters=[
                        {
                        "machine_name": robot_name,
                        },
                  ],
                  ),
      ])
