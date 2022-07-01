from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
      db_host = DeclareLaunchArgument(
            'db_host', default_value='localhost'
      )
      db_port = DeclareLaunchArgument(
            'db_port', default_value='27017'
      )

      tms_db_writer_node = Node(
            package='tms_db_manager',
            executable='tms_db_writer',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
      tms_db_writer_gridfs_node = Node(
            package='tms_db_manager',
            executable='tms_db_writer_gridfs',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
      tms_db_reader_node = Node(
            package='tms_db_manager',
            executable='tms_db_reader',
            output='screen',
            parameters=[{
                  'db_host': LaunchConfiguration('db_host'),
                  'db_port': LaunchConfiguration('db_port'),
            }]
      )
      # tms_db_reader_gridfs_node = Node(
      #       package='tms_db_manager',
      #       executable='tms_db_reader_gridfs',
      #       output='screen',
      #       parameters=[{
      #             'db_host': LaunchConfiguration('db_host'),
      #             'db_port': LaunchConfiguration('db_port'),
      #       }]
      # )
      return LaunchDescription([
            db_host,
            db_port,
            tms_db_writer_node,
            tms_db_writer_gridfs_node,
            tms_db_reader_node,
            # tms_db_reader_gridfs_node,
      ])
