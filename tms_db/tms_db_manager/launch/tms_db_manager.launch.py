from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
      return LaunchDescription([
            launch_ros.actions.Node(
            package='tms_db_manager', node_executable='tms_db_reader',
            output='screen'),
            launch_ros.actions.Node(
            package='tms_db_manager', node_executable='tms_db_writer',
            output='screen'),
      ])
