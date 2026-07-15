import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'h12_ros2_controller'

# with open('requirements.txt') as f:
#     requirements = f.read().splitlines()

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
    ('share/' + package_name + '/data/collision', glob('data/collision/*.npy')),
]

setup(
    name='h12_ros2_controller',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        'h12_ros2_controller.tests': ['configs/*.yaml'],
    },
    data_files=data_files,
    install_requires=['setuptools'],# + requirements,
    zip_safe=True,
    maintainer='tonyzyt2000',
    maintainer_email='zhangyt2000@gmail.com',
    description='ROS2 package of h12 robot controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = h12_ros2_controller.ros2.joint_state_publisher:main',
            'dual_arm_server = h12_ros2_controller.ros2.dual_arm_server:main',
            'dual_arm_client = h12_ros2_controller.ros2.dual_arm_client:main',
            'frame_task_server = h12_ros2_controller.ros2.frame_task_server:main',
            'frame_task_client = h12_ros2_controller.ros2.frame_task_client:main',
            'hand_controller_node = h12_ros2_controller.ros2.hand_controller_node:main',
            'hand_cmd_gui = h12_ros2_controller.ros2.hand_cmd_gui:main',
            'planner_example = h12_ros2_controller.example.planner_example:main',
        ],
    },
)
