import os
from glob import glob
from setuptools import setup

import launch

package_name = 'tms_ss_gnss'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools',
                      'pyserial',
                     ],
    zip_safe=True,
    maintainer='Kohei Matsumoto',
    maintainer_email='matsumoto@irvs.ait.kyushu-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmea_navsat_driver = tms_ss_gnss.nmea_navsat_driver:main',
            'nmea_serial_reader = tms_ss_gnss.nmea_serial_reader:main',
            'dummy_pub_node = tms_ss_gnss.dummy_pub:main',
            'ntrip_hub_node = tms_ss_gnss.ntrip_hub_node:main',
            'rtcm_server_node = tms_ss_gnss.rtcm_server_node:main'
        ],
    },
)
