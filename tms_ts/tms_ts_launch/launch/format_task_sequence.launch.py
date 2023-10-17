import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    tms_ts_subtask_package_dir = get_package_share_directory('tms_ts_subtask')

    return LaunchDescription([


        Node(
            package="tms_ts_subtask",
            executable="format_task_sequence.py",
            name="convert_from_xml_to_string",
            parameters=[{'bt_tree_xml_file_name':'sample_construction_tree_parallel', # You must put the source xml file in tms_ts/tms_ts_subtask/config.
                            "output_text_file_directory_path": tms_ts_subtask_package_dir + '/config',
                            "output_file_name":'task_sequence',
                            'output_txt_file':True}],
        ),
    
    ])