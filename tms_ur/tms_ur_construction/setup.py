from glob import glob
import os
from setuptools import setup

package_name = 'tms_ur_construction'

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
    maintainer='common',
    maintainer_email='uryu44213@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tms_ur_construction_ground = tms_ur_construction.tms_ur_construction_ground:main',
            'tms_ur_cv_odom = tms_ur_construction.tms_ur_cv_odom:main',
            'tms_ur_construction_terrain = tms_ur_construction.tms_ur_construction_terrain:main',
            'tms_ur_construction_terrain_mesh = tms_ur_construction.tms_ur_construction_terrain_mesh:main',
        ],
    },
)
