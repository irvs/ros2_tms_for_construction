from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tms_sd_theta"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
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
            "tms_sd_theta_compressed = tms_sd_theta.tms_sd_theta_compressed:main"
        ],
    },
)
