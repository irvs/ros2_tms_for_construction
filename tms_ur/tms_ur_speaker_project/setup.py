from setuptools import setup

package_name = 'tms_ur_speaker'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
            'test_client = tms_ur_speaker.test_client:main',
            'tms_ur_speaker_node = tms_ur_speaker.tms_ur_speaker_node:main',
        ],
    },
)


