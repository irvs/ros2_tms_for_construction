# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD


from setuptools import setup

setup(
    name='tms_db_manager',
    version='0.1.0',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='SONG',
    maintainer_email='song@irvs.ait.kyushu-u.ac,jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing examples of how to use the rclpy API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'tms_db_reader = scripts.tms_db_reader:main'
        ],
    },
)


