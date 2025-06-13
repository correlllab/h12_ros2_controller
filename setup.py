import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'h12_ros2_controller'

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
]

setup(
    name='h12_ros2_controller',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'] + requirements,
    zip_safe=True,
    maintainer='tonyzyt2000',
    maintainer_email='zhangyt2000@gmail.com',
    description='ROS2 package of h12 robot controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = h12_ros2_controller.joint_state_publisher:main',
            'dual_arm_server = h12_ros2_controller.dual_arm_server:main',
            'dual_arm_client = h12_ros2_controller.dual_arm_client:main',
        ],
    },
)
