<<<<<<< HEAD
=======
import os
from glob import glob
>>>>>>> feature_user_request
from setuptools import setup
from setuptools import find_packages

package_name = 'tms_ss_pozyx'
setup(
    name=package_name,
    version='0.1.0',
<<<<<<< HEAD
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
=======
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # ('share/' + package_name, ['package.xml', 'test.launch.py']),
        # (os.path.join('share', package_name, 'launch'), glob('*.launch.py')),
>>>>>>> feature_user_request
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
<<<<<<< HEAD
            'vicon_stream = tms_ss_pozyx.vicon_stream:main',
=======
            'tms_ss_pozyx = tms_ss_pozyx.tms_ss_pozyx:main',
>>>>>>> feature_user_request
        ],
    },
)


