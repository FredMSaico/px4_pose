import os
from glob import glob
from setuptools import setup

package_name = 'px4_pose'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfredo',
    maintainer_email='amamanisai@unsa.edu.pe',
    description='Publish PX4 drone position and add teleoperation',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         	'px4_twist_teleop = px4_pose.drone_teleop:main',
        	'px4_drone_control = px4_pose.drone_pos_control:main',
        	'px4_tf_pub = px4_pose.drone_pose:main',
        ],
    },
)
