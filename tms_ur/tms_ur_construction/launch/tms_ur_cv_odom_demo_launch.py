from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    latest = DeclareLaunchArgument("latest", default_value="True")

    # Nodes
    tms_ur_cv_odom_node1 = Node(
        # name="tms_ur_cv_odom1",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/backhow1/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "backhow1",
            },
        ],
    )
    tms_ur_cv_odom_node2 = Node(
        # name="tms_ur_cv_odom2",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/wheel_loader1/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "wheel_loader1",
            },
        ],
    )
    tms_ur_cv_odom_node3 = Node(
        # name="tms_ur_cv_odom3",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump1/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump1",
            },
        ],
    )
    tms_ur_cv_odom_node4 = Node(
        # name="tms_ur_cv_odom4",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump2/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump2",
            },
        ],
    )
    tms_ur_cv_odom_node5 = Node(
        # name="tms_ur_cv_odom5",
        package="tms_ur_construction",
        executable="tms_ur_cv_odom",
        output="screen",
        remappings=[
            ("~/output/odom", "/output/crawler_dump3/odom"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "crawler_dump3",
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
