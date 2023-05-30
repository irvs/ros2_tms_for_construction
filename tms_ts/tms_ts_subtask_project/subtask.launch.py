from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
      return LaunchDescription([
            launch_ros.actions.Node(
            package='tms_ts_subtask', node_executable='subtask_nodes',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_ts_subtask', node_executable='subtask_nodes_bed',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_ts_subtask', node_executable='subtask_nodes_roomlight',
            output='screen'),
      ])
