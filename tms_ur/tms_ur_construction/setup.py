from glob import glob
import os
from setuptools import setup

package_name = "tms_ur_construction"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Ryuichi Maeda",
    author_email="maeda@irvs.ait.kyushu-u.ac.jp",
    maintainer="maeda",
    maintainer_email="maeda@irvs.ait.kyushu-u.ac.jp",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=("Package containing examples of how to use the rclpy API."),
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tms_ur_construction_ground = tms_ur_construction.tms_ur_construction_ground:main",
            "tms_ur_cv_odom = tms_ur_construction.tms_ur_cv_odom:main",
            "tms_ur_cv_posest = tms_ur_construction.tms_ur_cv_posest:main",
            "tms_ur_cv_joints = tms_ur_construction.tms_ur_cv_joints:main",
            "tms_ur_cv_odom_to_posest = tms_ur_construction.tms_ur_cv_odom_to_posest:main",
            "tms_ur_machine_write_posest = tms_ur_construction.tms_ur_machine_write_posest:main",
            "tms_ur_db_writer = tms_ur_construction.tms_ur_db_writer:main",
            "tms_ur_construction_terrain_static = tms_ur_construction.tms_ur_construction_terrain_static:main",
            "tms_ur_construction_terrain_dynamic = tms_ur_construction.tms_ur_construction_terrain_dynamic:main",
            "tms_ur_construction_terrain_mesh = tms_ur_construction.tms_ur_construction_terrain_mesh:main",
            "tms_ur_construction_terrain_heightmap = tms_ur_construction.tms_ur_construction_terrain_heightmap:main",
            "tms_ur_construction_terrain = tms_ur_construction.tms_ur_construction_terrain:main",
            "tms_ur_construction_terrain_dem = tms_ur_construction.tms_ur_construction_terrain_dem:main",
            "tms_ur_ground_mesh = tms_ur_construction.tms_ur_ground_mesh:main",
            "tms_ur_construction_theta_compressed = tms_ur_construction.tms_ur_construction_theta_compressed:main",
        ],
    },
)
