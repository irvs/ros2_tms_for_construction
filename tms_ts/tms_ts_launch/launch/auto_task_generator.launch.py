from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tms_ts_subtask_package_dir = get_package_share_directory('tms_ts_subtask')

    return LaunchDescription([

        Node(
            package="tms_ts_subtask",
            executable="task_generator_from_task_sequence.py",
            name="task_generator_from_task_sequence",
            output='screen',
            parameters=[{
                'bt_tree_xml_file_name':'sample_construction_tree_parallel', # You must put the source xml file in tms_ts/tms_ts_subtask/config.
                "output_text_file_directory_path": tms_ts_subtask_package_dir + '/config',
                "output_file_name":'task_sequence',
                'output_txt_file':True,
                'model_name':'zx120', # You must define construction machinery model name(zx120, zx200, ic120).
                'description': 'No commented ...'} # Please define task description.
            ],
        ),
    
    ])