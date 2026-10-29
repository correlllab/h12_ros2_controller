import sys
import subprocess
import configparser
from pathlib import Path


def test_installed_console_scripts_are_preserved(tmp_path):
    root = Path(__file__).resolve().parents[1]
    subprocess.run(
        [sys.executable, 'setup.py', 'egg_info', '--egg-base', str(tmp_path)],
        cwd=root, check=True, capture_output=True, text=True,
    )
    metadata = next(tmp_path.glob('*.egg-info'))
    entry_points = configparser.ConfigParser()
    entry_points.read(metadata / 'entry_points.txt')
    assert dict(entry_points['console_scripts']) == {
        'joint_state_publisher':
            'h12_ros2_controller.ros2.joint_state_publisher:main',
        'dual_arm_server': 'h12_ros2_controller.ros2.dual_arm_server:main',
        'dual_arm_client': 'h12_ros2_controller.ros2.dual_arm_client:main',
        'frame_task_server': 'h12_ros2_controller.ros2.frame_task_server:main',
        'frame_task_client': 'h12_ros2_controller.ros2.frame_task_client:main',
        'hand_controller_node': 'h12_ros2_controller.ros2.hand_controller_node:main',
        'hand_cmd_gui': 'h12_ros2_controller.ros2.hand_cmd_gui:main',
        'planner_example': 'h12_ros2_controller.example.planner_example:main',
    }
