import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'tms_ts_test'
setup(
    name=package_name,
    version='0.1.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'test.launch.py']),
        # (os.path.join('share', package_name, 'launch'), glob('*.launch.py')),
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
            'task_manager_test_client = tms_ts_test.task_manager_test_client:main',
            'task_manager_recursion = tms_ts_test.task_manager_recursion:main',
            'task_viewer = tms_ts_test.task_viewer:main',
            'subtask_nodes = tms_ts_test.subtask_nodes:main',
            'subtask_nodes_bed = tms_ts_test.subtask_nodes_bed:main',
            'subtask_nodes_roomlight = tms_ts_test.subtask_nodes_roomlight:main',
            'task_text_recognizer = tms_ts_test.task_text_recognizer:main',
            'task_text_recognizer_test_client = tms_ts_test.task_text_recognizer_test_client:main',
            'test_stop_service = tms_ts_test.test_stop_service:main',
            'test_stop_client = tms_ts_test.test_stop_client:main',
            'test_stop_publisher = tms_ts_test.test_stop_publisher:main',
        ],
    },
)


