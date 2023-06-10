from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    latest = DeclareLaunchArgument("latest", default_value="True")

    # Nodes
    tms_ur_cv_odom_node1 = Node(
        name="tms_ur_cv_odom1",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/machine1/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "machine1",
            },
        ],
    )
    tms_ur_cv_odom_node2 = Node(
        name="tms_ur_cv_odom2",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/machine2/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "machine2",
            },
        ],
    )
    tms_ur_cv_odom_node3 = Node(
        name="tms_ur_cv_odom3",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/machine3/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "machine3",
            },
        ],
    )
    tms_ur_cv_odom_node4 = Node(
        name="tms_ur_cv_odom4",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/machine4/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "machine4",
            },
        ],
    )
    tms_ur_cv_odom_node5 = Node(
        name="tms_ur_cv_odom5",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/machine5/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "machine5",
            },
        ],
    )

    return LaunchDescription(
        [
            latest,
            tms_ur_cv_odom_node1,
            tms_ur_cv_odom_node2,
            tms_ur_cv_odom_node3,
            tms_ur_cv_odom_node4,
            tms_ur_cv_odom_node5,
        ]
    )
