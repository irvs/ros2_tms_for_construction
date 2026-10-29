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
                                     'zmq_publisher_port': zmq_publisher_port_base + (i * 10)}]))

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

      # mst2200vd の ROS のネームスペース (primitive のアクション名、TF、load_volume に使う)。
      # DB のレコード名は SAMPLE_BLACKBOARD_mst2200vd (掘削積込の中では mst2200vd に統一)
      declare_mst2200vd_ns_arg = DeclareLaunchArgument(
        'mst2200vd_ns',
        default_value='mst2200vd')

      # 掘削積込の情報管理ノード (tms_sp)を起動するか
      declare_launch_sp_arg = DeclareLaunchArgument(
        'launch_sp',
        default_value='true')

      use_sim_time = LaunchConfiguration('use_sim_time')
      mst2200vd_ns = LaunchConfiguration('mst2200vd_ns')
      launch_sp = IfCondition(LaunchConfiguration('launch_sp'))

      return LaunchDescription([

            declare_use_sim_time_arg,
            declare_task_ids_arg,
            declare_zmq_server_port_base_arg,
            declare_zmq_publisher_port_base_arg,
            declare_mst2200vd_ns_arg,
            declare_launch_sp_arg,

            # task_schedular_manager (task_id ごと)と tms_ur_button
            OpaqueFunction(function=launch_setup),


            # primitives
            Node(
                  package='tms_ts_primitive',
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive',
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package='tms_ts_primitive',
                  executable='wait_for_ur',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='zx200'),

            #ZX200
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_change_pose_plan',
                  output='screen',
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_change_pose_execute_from_plan',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_change_pose_execute_from_plan_retime',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace = 'zx200'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_excavator_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace = 'zx200'),

            #MST110CR
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_path',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_release_soil',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing_align_to_heading',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace='mst110cr'),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_compute_path_to_pose',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time,
                                 'output_model_name': 'mst110cr',
                                 'output_record_name': 'computed_path_autogenerated'}],
                  namespace='mst110cr',
                  remappings=[('planwritten', '/planwritten')]
                  ),

            #MST2200VD
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_path',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_follow_waypoints',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_anywhere',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_navigate_through_poses',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package='tms_ts_primitive',
                  executable='primitive_crawlerdump_release_soil',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),
            Node(
                  package = 'tms_ts_primitive',
                  executable='primitive_crawlerdump_swing_align_to_heading',
                  output='screen',
                  parameters = [{'use_sim_time': use_sim_time}],
                  namespace=mst2200vd_ns),

            # 掘削積込の情報管理 (tms_sp)

            # 掘削・放土・かき集めの刃先位置を/zx200/end_effectorトピックで受け取ってDBに書き込むノード (/zx200/end_effector -> target_*_pose)
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_zx200_end_effector',
                  parameters = [{'use_sim_time': use_sim_time}],
                  output='screen',
                  condition=launch_sp),
            # excavation_loadingサブタスク用 掘削ブロックの切り替えノード (NEXT_BLOCK_FLG -> current_block_index, excavate_position, loading_position)
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_block_manager',
                  parameters = [{'use_sim_time': use_sim_time}],
                  output='screen',
                  condition=launch_sp),
            # 建機制御用PC - LiDAR用PC 間のDB同期用ブリッジ (config/excavation_loading_sensor_bridge.yaml)
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_sensor_bridge',
                  parameters = [{'use_sim_time': use_sim_time}],
                  output='screen',
                  condition=launch_sp),
            # 積載量 -> 各ダンプのレコードの SENSING_LOADED_FLG
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_loadsense',
                  name='tms_sp_loadsense_mst110cr',
                  parameters = [{'use_sim_time': use_sim_time,
                                 'machine_name': 'mst110cr',
                                 'record_name': 'SAMPLE_BLACKBOARD_mst110cr'}],
                  output='screen',
                  condition=launch_sp),
            Node(
                  package='tms_sp_sensing',
                  executable='tms_sp_loadsense',
                  name='tms_sp_loadsense_mst2200vd',
                  parameters = [{'use_sim_time': use_sim_time,
                                 'machine_name': mst2200vd_ns,
                                 'record_name': 'SAMPLE_BLACKBOARD_mst2200vd'}],
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
            
            #for odom
            Node(
                  package="tms_ur_construction",
                  executable="tms_ur_parameter_viz",
                  output="screen",
                  ),
            Node(
                  package="tms_sp_machine",
                  executable="tms_sp_machine_odom.py",
                  output="screen",
                  remappings=[
                        ("~/input/odom", "/mst110cr_2/odometry/global"),
                  ],
                  parameters=[
                        {
                        "machine_name": "mst110cr_2",
                        },
                  ],
                  ),
            Node(
                  package="tms_sp_machine",
                  executable="tms_sp_machine_posest.py",
                  output="screen",
                  remappings=[
                        ("~/input/pose", "/mst2200vd/robot/pose"),
                  ],
                  parameters=[
                        {
                        "machine_name": "mst2200vd",
                        },
                  ],
                  ),
            Node(
                  package='tms_db_manager',
                  executable='tms_db_writer',
                  output='screen',
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
      ])
