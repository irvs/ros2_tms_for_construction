import os
import sys
import launch
import launch.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate a launch description for a single serial driver."""
    config_file = os.path.join(get_package_share_directory("tms_ss_gnss"), "config", "nmea_serial_reader.yaml")
    print(config_file)
    serial_reader_node = Node(
        package='tms_ss_gnss', 
        executable='nmea_serial_reader', 
        output='screen',
        parameters=[config_file])

    return launch.LaunchDescription([serial_reader_node])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = launch.LaunchService()
    # ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)