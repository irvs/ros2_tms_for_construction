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
        declare_zmq_server_port_base,
        declare_zmq_publisher_port_base,
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    # MongoDB初期化を最初に実行
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
            #     arguments=['--ros-args', '--log-level', 'ERROR']  # WARNINGを非表示
            )
        )

    nodes.extend([
        Node(
            package='tms_ur_button_input',
            executable='tms_ur_button',
            output='screen',
            parameters=[{"task_ids": task_ids}]),
        
        # primitives for zx200
        Node(
              package='tms_ts_primitive', 
              executable='primitive_excavator_change_pose',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_excavator_change_pose_plan',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_excavator_excavate_simple',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_excavator_excavate_simple_plan',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_primitive', 
              executable='primitive_excavator_release_simple',
              output='screen',
              namespace = 'zx200'),
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
        
        # primitives for ic120
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_follow_waypoints',
              output='screen',
              namespace = 'ic120'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_navigate_anywhere',
              output='screen',
              namespace = 'ic120'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_navigate_through_poses',
              output='screen',
              namespace = 'ic120'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_release_soil',
              output='screen',
              namespace = 'ic120'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_swing_align_to_heading',
              output='screen',
              namespace = 'ic120'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_swing',
              output='screen',
              namespace = 'ic120'),

        # primitives for mst110cr
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_follow_waypoints',
              output='screen',
              namespace = 'mst110cr'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_navigate_anywhere',
              output='screen',
              namespace = 'mst110cr'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_navigate_through_poses',
              output='screen',
              namespace = 'mst110cr'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_release_soil',
              output='screen',
              namespace = 'mst110cr'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_swing_align_to_heading',
              output='screen',
              namespace = 'mst110cr'),
        Node(
              package='tms_ts_primitive',
              executable='primitive_crawlerdump_swing',
              output='screen',
              namespace = 'mst110cr'),
       
        
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
    ])
    
    return nodes