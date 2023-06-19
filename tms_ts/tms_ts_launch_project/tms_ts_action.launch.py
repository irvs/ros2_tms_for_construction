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
      ])
