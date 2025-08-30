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
        
        # subtasks
        Node(
              package='tms_ts_subtask', 
              executable='subtask_zx200_change_pose',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_zx200_change_pose_plan',
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
              executable='subtask_zx200_follow_waypoints_server',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_subtask',
              executable='subtask_zx200_navigate_anywhere_server',
              output='screen',
              namespace = 'zx200'),
        Node(
              package='tms_ts_subtask',
              executable='subtask_zx200_navigate_through_poses_server',
              output='screen',
              namespace = 'zx200'),
        
        
        # sample ###
        #ic120用
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_follow_waypoints_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_follow_waypoints_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_anywhere_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_anywhere_server',
      #         output='screen',
      #         namespace='ic120_1'),

      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_through_poses_deg_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_through_poses_server',
      #         output='screen',
      #         namespace='ic120_1'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_release_soil_server',
      #         output='screen',
      #         namespace='ic120_1'),

      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_follow_waypoints_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_follow_waypoints_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_anywhere_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_anywhere_server',
      #         output='screen',
      #         namespace='ic120_2'),

      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_through_poses_deg_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_navigate_through_poses_server',
      #         output='screen',
      #         namespace='ic120_2'),
      #   Node(
      #         package='tms_ts_subtask',
      #         executable='subtask_ic120_release_soil_server',
      #         output='screen',
      #         namespace='ic120_2'),
        
        # zx120用
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='zx120_sample_boom_subtask',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='zx120_sample_swing_subtask',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='zx120_sample_arm_subtask',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='zx120_sample_bucket_subtask',
      #         output='screen'),
        
        # zx200用
        # Node(
        #       package='tms_ts_subtask', 
        #       executable='zx200_sample_boom_subtask',
        #       output='screen'),
        # Node(
        #       package='tms_ts_subtask', 
        #       executable='zx200_sample_swing_subtask',
        #       output='screen'),
        # Node(
        #       package='tms_ts_subtask', 
        #       executable='zx200_sample_arm_subtask',
        #       output='screen'),
        # Node(
        #       package='tms_ts_subtask', 
        #       executable='zx200_sample_bucket_subtask',
        #       output='screen'),

        # mst2200用
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_follow_waypoints_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_follow_waypoints_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_navigate_anywhere_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_navigate_anywhere_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_navigate_through_poses_deg_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_navigate_through_poses_server',
      #         output='screen'),
      #   Node(
      #         package='tms_ts_subtask', 
      #         executable='subtask_mst2200_release_soil_server',
      #         output='screen'),

            # mst110cr用
      Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_follow_waypoints_deg',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_follow_waypoints',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_navigate_anywhere_deg',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_navigate_anywhere',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_navigate_through_poses_deg',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_navigate_through_poses',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_release_soil',
              output='screen',
              namespace='mst110cr_2'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_swing_align_to_heading',
              output='screen',
              namespace='mst110cr_2'),   
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst110cr_swing',
              output='screen',
              namespace='mst110cr_2'),
        # mst2200vd用
        Node(
                  package='tms_ts_subtask', 
                  executable='subtask_mst2200_follow_waypoints_deg',
                  output='screen',
                  namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_follow_waypoints',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_navigate_anywhere_deg',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_navigate_anywhere',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_navigate_through_poses_deg',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_navigate_through_poses',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_release_soil',
              output='screen',
              namespace='mst2200vd'),
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_swing_align_to_heading',
              output='screen',
              namespace='mst2200vd'),   
        Node(
              package='tms_ts_subtask', 
              executable='subtask_mst2200_swing',
              output='screen',
              namespace='mst2200vd'),
       
        
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