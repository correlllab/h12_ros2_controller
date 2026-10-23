'''Test fixed-base frame FK and the ROS frame action adapter'''

import ast
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pinocchio as pin
import pytest

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, ENABLED_JOINTS
from h12_ros2_controller.utility.path_definition import URDF_MAGPIE_PATH


def test_measured_frame_uses_pelvis_not_imu_or_frozen_reference():
    if not Path(URDF_MAGPIE_PATH).exists():
        pytest.skip('robot assets unavailable')
    robot = RobotModel(URDF_MAGPIE_PATH)
    robot.init_reduced_model(ENABLED_JOINTS)
    q = np.zeros(robot.model_body.nq)
    q[BODY_JOINTS.index('torso_joint')] = 0.24
    q[BODY_JOINTS.index('left_shoulder_pitch_joint')] = -0.3
    tilt = pin.rpy.rpyToMatrix(np.array([0.2, -0.3, 0.1]))
    quat = pin.Quaternion(tilt)
    imu = SimpleNamespace(quaternion=np.array([quat.w, quat.x, quat.y, quat.z]))
    robot.state_subscriber = SimpleNamespace(state={
        'q': q, 'dq': np.zeros_like(q), 'imu_state': imu,
    })
    frame = 'left_grasp_frame'

    expected_data = robot.model_body.createData()
    pin.forwardKinematics(robot.model_body, expected_data, q)
    pin.updateFramePlacements(robot.model_body, expected_data)
    expected = expected_data.oMf[robot.model_body.getFrameId(frame)].np
    world = robot.get_frame_transformation(frame)
    measured = robot.get_frame_transformation_body(frame)
    frozen = robot.get_frame_transformation_reduced(frame, q[robot.reduced_mask])
    controller = object.__new__(FrameController)
    controller.robot_model = robot

    assert not np.allclose(world, expected)
    assert not np.allclose(frozen, expected)
    np.testing.assert_allclose(measured, expected)
    np.testing.assert_allclose(controller.get_frame_transformation(frame), expected)
    np.testing.assert_allclose(controller.get_frame_pose(frame)[:3], expected[:3, 3])
    np.testing.assert_allclose(
        pin.rpy.rpyToMatrix(controller.get_frame_pose(frame)[3:]),
        expected[:3, :3],
    )
    with pytest.raises(ValueError, match='Unknown robot frame'):
        robot.get_frame_transformation_body('not_a_frame', q)


def load_server_methods():
    source = (Path(__file__).resolve().parents[1] / 'h12_ros2_controller' /
              'ros2' / 'frame_task_server.py')
    tree = ast.parse(source.read_text())
    server = next(node for node in tree.body if isinstance(node, ast.ClassDef))

    class Result:
        success = None

    class Message:
        def __init__(self):
            self.header = SimpleNamespace()

    namespace = {
        'FrameTask': SimpleNamespace(Result=Result),
        'StaleStateError': type('StaleStateError', (RuntimeError,), {}),
        'StringArray': Message, 'PoseArray': Message,
        'matrix_to_pose': lambda value: value.copy(),
        'pose_to_matrix': lambda value: value,
        'np': np,
    }
    for node in server.body:
        if isinstance(node, ast.FunctionDef) and node.name in {
            'frame_task_callback', '_frame_task_callback', '_resolve_target',
            'publisher_callback', '_stamp_pose',
        }:
            module = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
            exec(compile(module, str(source), 'exec'), namespace)
    return namespace


@pytest.mark.parametrize(('names', 'targets'), [
    ([], []), (['left_grasp_frame'], []),
    (['left_grasp_frame', 'left_grasp_frame'], [object(), object()]),
])
def test_invalid_frame_goal_aborts_before_mutating_tasks(names, targets):
    methods = load_server_methods()
    controller = SimpleNamespace(check_state_fresh=Mock(), clear_frame_tasks=Mock())
    server = SimpleNamespace(
        controller=controller, _controller_lock=threading.Lock(),
        get_logger=lambda: SimpleNamespace(info=Mock(), error=Mock()),
        frame_names=['previous'], frame_targets=['previous target'],
    )
    server._frame_task_callback = lambda goal: methods['_frame_task_callback'](server, goal)
    goal = SimpleNamespace(
        request=SimpleNamespace(frame_names=names, frame_targets=targets),
        abort=Mock(),
    )

    result = methods['frame_task_callback'](server, goal)

    assert result.success is False
    goal.abort.assert_called_once()
    controller.check_state_fresh.assert_not_called()
    controller.clear_frame_tasks.assert_not_called()
    assert server.frame_names == ['previous']
    assert server.frame_targets == ['previous target']


def test_nonfinite_frame_target_is_rejected_before_task_mutation():
    methods = load_server_methods()
    controller = SimpleNamespace(
        robot_model=SimpleNamespace(model_body=SimpleNamespace(
            frames=[SimpleNamespace(name='left_grasp_frame')],
        )),
        check_state_fresh=Mock(), clear_frame_tasks=Mock(),
    )
    server = SimpleNamespace(
        controller=controller, _controller_lock=threading.Lock(),
        get_logger=lambda: SimpleNamespace(info=Mock(), error=Mock()),
    )
    server._frame_task_callback = lambda goal: methods['_frame_task_callback'](server, goal)
    invalid = SimpleNamespace(
        position=SimpleNamespace(x=float('nan'), y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    goal = SimpleNamespace(
        request=SimpleNamespace(
            frame_names=['left_grasp_frame'], frame_targets=[invalid],
        ),
        abort=Mock(),
    )

    result = methods['frame_task_callback'](server, goal)

    assert result.success is False
    goal.abort.assert_called_once()
    controller.clear_frame_tasks.assert_not_called()


def test_ros_frame_goal_installs_two_tasks_before_one_planned_motion():
    methods = load_server_methods()

    class Target:
        position = SimpleNamespace(x=0.4, y=0.0, z=0.2)
        orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)

        def copy(self):
            return self

    names = ['left_grasp_frame', 'right_grasp_frame']
    controller = SimpleNamespace(
        robot_model=SimpleNamespace(
            model_body=SimpleNamespace(frames=[SimpleNamespace(name=name)
                                               for name in names]),
            state={'q': np.zeros(27)},
        ),
        check_state_fresh=Mock(), clear_frame_tasks=Mock(),
        add_frame_task=Mock(), update_ik_solver=Mock(), speed_scale=1.0,
    )
    server = SimpleNamespace(
        controller=controller, _controller_lock=threading.Lock(),
        get_logger=lambda: SimpleNamespace(info=Mock(), error=Mock()),
        timeout=1.0, frame_names=[], frame_targets=[],
        _resolve_target=lambda name, target, relative, measured_q: target,
        _plan_frame_task_callback=Mock(return_value='one plan'),
    )
    goal = SimpleNamespace(request=SimpleNamespace(
        frame_names=names, frame_targets=[Target(), Target()],
        relative=False, plan=True, slow_mode=False,
        duration=SimpleNamespace(sec=0, nanosec=0),
    ))

    result = methods['_frame_task_callback'](server, goal)

    assert result == 'one plan'
    assert controller.clear_frame_tasks.call_count == 1
    assert controller.add_frame_task.call_count == 2
    assert [call.args[1] for call in controller.add_frame_task.call_args_list] == names
    server._plan_frame_task_callback.assert_called_once_with(goal, 1.0)


def test_relative_target_and_published_poses_share_pelvis_frame():
    methods = load_server_methods()
    current = np.eye(4)
    current[:3, 3] = [0.3, 0.2, 0.5]
    delta = np.eye(4)
    delta[:3, 3] = [0.1, 0.0, 0.0]
    controller = SimpleNamespace(
        get_frame_transformation=Mock(return_value=current),
        robot_model=SimpleNamespace(state={'q': np.zeros(27)}),
        left_ee_name='left_grasp_frame', right_ee_name='right_grasp_frame',
        ik_solver=SimpleNamespace(frame_tasks={
            'left_ee_task': SimpleNamespace(transform_target_to_world=SimpleNamespace(np=delta)),
        }),
    )
    lock = threading.Lock()
    server = SimpleNamespace(
        controller=controller, _controller_lock=lock,
        frame_names=['left_grasp_frame'], frame_targets=[delta],
        frame_names_publisher=Mock(), frame_targets_publisher=Mock(),
        frame_poses_publisher=Mock(), left_ee_pose_publisher=Mock(),
        right_ee_pose_publisher=Mock(), left_ee_target_publisher=Mock(),
        right_ee_target_publisher=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'stamp')),
    )
    server._stamp_pose = lambda pose: SimpleNamespace(
        header=SimpleNamespace(frame_id='pelvis'), pose=pose,
    )
    absolute = methods['_resolve_target'](server, 'left_grasp_frame', delta, True)
    np.testing.assert_allclose(absolute[:3, 3], [0.4, 0.2, 0.5])

    lock.acquire()
    try:
        methods['publisher_callback'](server)
        server.frame_poses_publisher.publish.assert_not_called()
    finally:
        lock.release()
    methods['publisher_callback'](server)
    poses = server.frame_poses_publisher.publish.call_args.args[0]
    targets = server.frame_targets_publisher.publish.call_args.args[0]
    assert poses.header.frame_id == targets.header.frame_id == 'pelvis'
    np.testing.assert_allclose(poses.poses[0], current)
    np.testing.assert_allclose(
        server.left_ee_pose_publisher.publish.call_args.args[0].pose, current,
    )
    server.left_ee_target_publisher.publish.assert_called_once()
