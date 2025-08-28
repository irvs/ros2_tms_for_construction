import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'tms_db_manager'
setup(
    name=package_name,
    version='0.1.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='kasahara',
    maintainer_email='kashara.yuichiro.res@gmail.com',
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
            'tms_db_reader = tms_db_manager.tms_db_reader:main',
            'tms_db_reader_gridfs = tms_db_manager.tms_db_reader_gridfs:main',
            'tms_db_reader_task = tms_db_manager.tms_db_reader_task:main',
            'tms_db_reader_parameter = tms_db_manager.tms_db_reader_parameter:main',
            'tms_db_writer = tms_db_manager.tms_db_writer:main',
            'tms_db_writer_gridfs = tms_db_manager.tms_db_writer_gridfs:main',
            'tms_db_writer_collision = tms_db_manager.tms_db_writer_collision:main',
            'tms_db_writer_task = tms_db_manager.tms_db_writer_task:main',
            'tms_db_writer_param = tms_db_manager.tms_db_writer_param:main',
            'tms_db_reader_param = tms_db_manager.tms_db_reader_param:main',
            "tms_db_reader_heightmap = tms_db_manager.tms_db_reader_heightmap:main",
        ],
    },
)


