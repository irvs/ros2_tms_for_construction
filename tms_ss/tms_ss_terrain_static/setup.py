from glob import glob
import os
from setuptools import setup

package_name = 'tms_ss_terrain_static'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ryuichi Maeda',
    author_email='maeda@irvs.ait.kyushu-u.ac.jp',
    maintainer='kawamura',
    maintainer_email='Kawamura@irvs.ait.kyushu-u.ac,jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=('Package containing examples of how to use the rclpy API.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tms_ss_terrain_static = tms_ss_terrain_static.tms_ss_terrain_static:main',
            'tms_ss_terrain_static_mesh = tms_ss_terrain_static.tms_ss_terrain_static_mesh:main',
            'tms_ss_terrain_static_dem = tms_ss_terrain_static.tms_ss_terrain_static_dem:main',
        ],
    },
)
