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
from launch.substitutions import LaunchConfiguration
import yaml
import os
from pymongo import MongoClient

def initialize_mongodb_from_yaml():
    """
    initial_values.yamlファイルからMongoDBを初期化する関数
    """
    try:
        yaml_path = os.path.join(os.path.dirname(__file__), "../config", "initial_values.yaml")
        
        # YAMLファイルを読み込み
        with open(yaml_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
        
        # MongoDB接続設定を取得
        mongodb_config = config.get('mongodb', {})
        ip_address = mongodb_config.get('ip_address', 'localhost')
        port = mongodb_config.get('port', 27017)
        database_name = mongodb_config.get('database', 'rostmsdb')
        collection_name = mongodb_config.get('collection', 'parameter')
        
        # MongoDBに接続
        client = MongoClient(ip_address, port)
        db = client[database_name]
        collection = db[collection_name]
        
        # 初期パラメータを取得して挿入
        initial_params = config.get('initial_parameters', {})
        
        for param in initial_params:
            record_name = param.get('record_name')
            values = param.get('values', {})
            
            if not record_name:
                print(f"Error: record_name is missing in parameter config: {param}")
                continue
            
            if not values:
                print(f"Warning: No values specified for record_name: {record_name}")
                
            # 既存のドキュメントを検索
            query = {"type": "dynamic", "record_name": record_name}
            existing_doc = collection.find_one(query)
            
            # $setで更新する形式で値を準備
            update_data = {"$set": values}
            
            if existing_doc:
                # 既存のドキュメントを更新
                collection.update_one(query, update_data)
                print(f"Updated parameter: {record_name}")
            else:
                # 既存のドキュメントが見つからない場合はエラー
                print(f"Error: Document not found for record_name: {record_name}, type: {param_type}")
                continue
        
        print("MongoDB initialization completed successfully")
        
    except Exception as e:
        print(f"Failed to initialize MongoDB: {str(e)}")

def generate_launch_description():
    # Define the task IDs for the demo
    declare_task_ids = DeclareLaunchArgument(
        'task_ids',
        default_value='[1, 2, 3]',
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # Define ZMQ port parameters for multiple machine deployment
    declare_zmq_server_port_base = DeclareLaunchArgument(
        'zmq_server_port_base',
        default_value='1666',
        description='Base port number for ZMQ server (will be incremented for each task_id)'
    )
    
    declare_zmq_publisher_port_base = DeclareLaunchArgument(
        'zmq_publisher_port_base', 
        default_value='1777',
        description='Base port number for ZMQ publisher (will be incremented for each task_id)'
    )
    
    return LaunchDescription([
        declare_task_ids,
        declare_use_sim_time_arg,
        declare_zmq_server_port_base,
        declare_zmq_publisher_port_base,
        OpaqueFunction(function=launch_setup)
    ])

    


def launch_setup(context, *args, **kwargs):
    initialize_mongodb_from_yaml()

    # Get the task_ids parameter and convert to int array
    task_ids_str = LaunchConfiguration('task_ids').perform(context)
    zmq_server_port_base = int(LaunchConfiguration('zmq_server_port_base').perform(context))
    zmq_publisher_port_base = int(LaunchConfiguration('zmq_publisher_port_base').perform(context))
    
    # Convert string to int array using eval with default fallback
    task_ids = eval(task_ids_str)
    
    nodes = []
    for i, task_id in enumerate(task_ids):
        # Calculate unique port numbers for each task_id
        zmq_server_port = zmq_server_port_base + (i * 10)  # Increment by 10 for each task
        zmq_publisher_port = zmq_publisher_port_base + (i * 10)
        
        # Create unique node name for each task_id
        node_name = f"task_schedular_manager_{task_id}"
        
        nodes.append(
            Node(
                package='tms_ts_manager',
                executable='task_schedular_manager',
                # name=node_name,  # コメントアウト（元の名前のまま）
                output='screen',
                parameters=[{
                    "task_id": task_id,
                    "zmq_server_port": zmq_server_port,
                    "zmq_publisher_port": zmq_publisher_port
                }],
                arguments=['--ros-args', '--log-level', 'ERROR']  # WARNINGを非表示
            )
        )

    nodes.extend([
        Node(
            package='tms_ur_button_input',
            executable='tms_ur_button',
            output='screen',
            parameters=[{"task_ids": task_ids}]),
        
        # primitives
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
      # Node(
      #       package='tms_ts_primitive',
      #       executable='primitive_excavator_change_pose_execute_from_plan_retime',
      #       output='screen',
      #       parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
      #       namespace = 'zx200'),
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
              parameters=[{"model_name": "zx200", "root_record_name": "collision_objects_shimiz", "planning_frame": "base_link"}]),      
        Node(
              package='tms_ts_primitive',
              executable='primitive_excavator_follow_waypoints',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_excavator_navigate_anywhere',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_excavator_navigate_through_poses',
              output='screen',
              namespace = 'zx200'),
        
        
        # sample ###
        #ic120用
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_follow_waypoints_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_follow_waypoints_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_anywhere_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_anywhere_server',
      #         output='screen',
      #         namespace='ic120_1'),

      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_through_poses_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_through_poses_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_release_soil_server',
      #         output='screen',
      #         namespace='ic120_1'),

      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_follow_waypoints_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_follow_waypoints_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_anywhere_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_anywhere_server',
      #         output='screen',
      #         namespace='ic120_2'),

      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_through_poses_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_navigate_through_poses_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_primitive',
      #         executable='primitive_ic120_release_soil_server',
      #         output='screen',
      #         namespace='ic120_2'),
        
        # zx120用
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='zx120_sample_boom_primitive',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='zx120_sample_swing_primitive',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='zx120_sample_arm_primitive',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='zx120_sample_bucket_primitive',
      #         output='screen'),
        
        # zx200用
        # Node(
        #       package='tms_ts_primitive', 
        #       executable='zx200_sample_boom_primitive',
        #       output='screen'),
        # Node(
        #       package='tms_ts_primitive', 
        #       executable='zx200_sample_swing_primitive',
        #       output='screen'),
        # Node(
        #       package='tms_ts_primitive', 
        #       executable='zx200_sample_arm_primitive',
        #       output='screen'),
        # Node(
        #       package='tms_ts_primitive', 
        #       executable='zx200_sample_bucket_primitive',
        #       output='screen'),

        # mst2200用
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_follow_waypoints_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_follow_waypoints_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_navigate_anywhere_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_navigate_anywhere_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_navigate_through_poses_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_navigate_through_poses_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_primitive', 
      #         executable='primitive_mst2200_release_soil_server',
      #         output='screen'),

        # mst110cr用
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_follow_waypoints',
              output='screen',
              namespace='mst110cr'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_anywhere',
              output='screen',
              namespace='mst110cr'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_through_poses',
              output='screen',
              namespace='mst110cr'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_release_soil',
              output='screen',
              namespace='mst110cr'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing_align_to_heading',
              output='screen',
              namespace='mst110cr'),   
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing',
              output='screen',
              namespace='mst110cr'),
        # mst110cr用
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_follow_waypoints',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_anywhere',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_through_poses',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_release_soil',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing_align_to_heading',
              output='screen',
              namespace='mst110cr_2'),   
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing',
              output='screen',
              namespace='mst110cr_2'),
        # mst2200vd用
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_follow_waypoints',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_anywhere',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_navigate_through_poses',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_release_soil',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing_align_to_heading',
              output='screen',
              namespace='mst2200vd'),   
        Node(
              package='tms_ts_primitive', 
              executable='primitive_crawlerdump_swing',
              output='screen',
              namespace='mst2200vd'),
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
       
        
        # センシング処�?後�?��?ータをデータベ�?�スに取り込むためのノ�?�ド�?
        Node(
              package='tms_sp_sensing', 
              executable='tms_sp_zx200_end_effector',
              output='screen',
        ),
        Node(
              package='tms_sp_sensing', 
              executable='sample',
              output='screen'
        ),
        Node(
              package='tms_sp_sensing', 
              executable='tms_sp_flgs_202508',
              output='screen'
        ),
        Node(
              package="tms_db_manager", 
              executable="tms_db_reader_task",
              output='screen'
              ),
        Node(
              package="tms_db_manager", 
              executable="tms_db_reader_param",
              output='screen'
              ),
        Node(
              package="tms_db_manager", 
              executable="tms_db_reader_subtask",
              output='screen'
              ),
      #   Node(
      #         package='tms_sp_sensing', 
      #         executable='tms_sp_dump_swing_angle',
      #         output='screen',
      #         parameters=[{"base_frame": "base_link"},
      #                     {"robot_names": ["mst110cr_2", "mst2200vd", "ic120_tf"]},
      #                     {"record_names": ["SAMPLE_BLACKBOARD_mst110cr_2", "SAMPLE_BLACKBOARD_mst2200", "SAMPLE_BLACKBOARD_ic120"]}]
      #   ),
      #  Node(
      #         package='tms_sp_sensing', 
      #         executable='tms_sp_zx200_collision_objects_from_tf',
      #         output='screen',
      #         parameters=[{"robot_name": "ic120"}]
      #   ),
      #   Node(
      #         package='tms_sp_sensing', 
      #         executable='tms_sp_zx200_collision_objects_from_tf',
      #         output='screen',
      #         parameters=[{"robot_name": "mst110cr"}]
      #   ),
    ])
    
    return nodes