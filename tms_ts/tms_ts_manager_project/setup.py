from setuptools import setup
from setuptools import find_packages

package_name = 'tms_ts_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'tms_ts_manager = tms_ts_manager.tms_ts_manager:main',
            'tms_ts_manager_action = tms_ts_manager.tms_ts_manager_action:main',
            'tms_ts_manager_new = tms_ts_manager.tms_ts_manager_new:main',
            'tms_ts_manager_new2 = tms_ts_manager.tms_ts_manager_new2:main',
        ],
    },
)


