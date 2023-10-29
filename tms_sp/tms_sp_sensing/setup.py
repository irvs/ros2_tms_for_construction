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
            "tms_sp_zx120_end_effector = tms_sp_sensing.tms_sp_zx120_end_effector:main",
            "sample = tms_sp_sensing.sample:main"
        ],
    },
)
