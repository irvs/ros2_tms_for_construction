from setuptools import setup
from setuptools import find_packages

package_name = 'tms_ts_subtask'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','subtask.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Itsuka Tomoya',
    author_email='itsuka@irvs.ait.kyushu-u.ac.jp',
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
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'subtask_nodes = tms_ts_subtask.subtask_nodes:main',
            'subtask_nodes_bed = tms_ts_subtask.subtask_nodes_bed:main',
            'subtask_nodes_roomlight = tms_ts_subtask.subtask_nodes_roomlight:main',
        ],
    },
)


