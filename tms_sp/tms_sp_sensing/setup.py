from setuptools import setup

package_name = 'tms_sp_sensing'

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
    maintainer='common',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tms_sp_zx200_end_effector = tms_sp_sensing.tms_sp_zx200_end_effector:main",
            "tms_sp_zx200_collision_objects = tms_sp_sensing.tms_sp_zx200_collision_objects_ic120:main",
            "sample = tms_sp_sensing.sample:main",
            "tms_sp_flgs = tms_sp_sensing.tms_sp_flgs:main",
            "tms_sp_flgs_202506 = tms_sp_sensing.tms_sp_flgs_202506:main",

            "tms_sp_navigate_anywhere = tms_sp_sensing.tms_sp_navigate_anywhere:main",
            "tms_sp_navigate_anywhere_deg = tms_sp_sensing.tms_sp_navigate_anywhere_deg:main",
            "tms_sp_follow_waypoints = tms_sp_sensing.tms_sp_follow_waypoints:main",
            "tms_sp_follow_waypoints_deg = tms_sp_sensing.tms_sp_follow_waypoints_deg:main",
            "tms_sp_navigate_through_poses = tms_sp_sensing.tms_sp_navigate_through_poses:main",
            "tms_sp_navigate_through_poses_deg = tms_sp_sensing.tms_sp_navigate_through_poses_deg:main",
            "tms_sp_release_soil = tms_sp_sensing.tms_sp_release_soil:main",
            "tms_sp_change_pose = tms_sp_sensing.tms_sp_change_pose:main",
            "tms_sp_excavate_simple = tms_sp_sensing.tms_sp_excavate_simple:main",
            "tms_sp_excavate_simple_plan = tms_sp_sensing.tms_sp_excavate_simple_plan:main",
            "tms_sp_release_simple = tms_sp_sensing.tms_sp_release_simple:main",
            
            "tms_sp_dig_cycle = tms_sp_sensing.tms_sp_dig_cycle:main",
            "tms_sp_dig_locations = tms_sp_sensing.tms_sp_dig_locations:main",
            "tms_sp_release_angle = tms_sp_sensing.tms_sp_release_angle:main",
            "tms_sp_release_locations = tms_sp_sensing.tms_sp_release_locations:main"
        ],
    },
)
