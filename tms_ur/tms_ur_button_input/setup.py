from glob import glob
import os
from setuptools import setup

package_name = 'tms_ur_button_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Yuchiro Kasahara',
    author_email='kasahara@irvs.ait.kyushu-u.ac.jp',
    maintainer='Yuichiro Kasahara',
    maintainer_email='kasahara@irvs.ait.kyushu-u.ac.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tms_ur_button = tms_ur_button_input.tms_ur_button:main',
            'tms_ur_button_ = tms_ur_button_input.tms_ur_button_:main',
        ],
    },
)
