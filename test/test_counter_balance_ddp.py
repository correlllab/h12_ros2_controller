from types import SimpleNamespace

import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.counter_balance_ddp import (
    CounterBalanceActionModel,
    CounterBalanceDDP,
)


def _chain_model(joint_count, free_flyer):
    model = pin.Model()
    parent = 0
    if free_flyer:
        parent = model.addJoint(
            0,
            pin.JointModelFreeFlyer(),
            pin.SE3.Identity(),
            'root_joint',
        )
        model.appendBodyToJoint(
            parent,
            pin.Inertia(2.0, np.zeros(3), 0.02 * np.eye(3)),
            pin.SE3.Identity(),
        )
    joint_models = (
        pin.JointModelRY,
        pin.JointModelRX,
        pin.JointModelRZ,
    )
    for index in range(joint_count):
        joint_model = joint_models[index % len(joint_models)]()
        parent = model.addJoint(
            parent,
            joint_model,
            pin.SE3(np.eye(3), np.array([0.08, 0.01, 0.03])),
            f'joint_{index}',
        )
        model.appendBodyToJoint(
            parent,
            pin.Inertia(
                0.5,
                np.array([0.04, 0.01, 0.0]),
                0.005 * np.eye(3),
            ),
            pin.SE3.Identity(),
        )
    model.lowerPositionLimit[:] = -2.0
    model.upperPositionLimit[:] = 2.0
    model.velocityLimit[:] = 2.0
    return model


def _action_models():
    model = _chain_model(4, free_flyer=True)
    full_q = pin.neutral(model)
    q = np.array([0.1, -0.2, 0.15, -0.1])
    full_q[7:] = q
    com = pin.centerOfMass(model, model.createData(), full_q)
    common = dict(
        model=model,
        full_q_template=full_q,
        motor_ids=[0, 1, 2, 3],
        q_ref=np.array([0.0, -0.1, 0.1, 0.0]),
        com_target=com[:2] + np.array([0.01, -0.005]),
        dt=0.02,
        position_lower=-np.ones(4),
        position_upper=np.ones(4),
        velocity_limit=np.array([0.8, 0.7, 0.6, 0.5]),
        w_com=1.0,
        w_control=0.01,
        w_posture=0.02,
        w_terminal_posture=0.03,
        w_soft_limit=0.5,
        soft_limit_margin=0.1,
    )
    running = CounterBalanceActionModel(
        **common,
        control_lower=-common['velocity_limit'],
        control_upper=common['velocity_limit'],
    )
    terminal = CounterBalanceActionModel(
        **common,
        is_terminal=True,
        control_lower=None,
        control_upper=None,
    )
    return q, running, terminal


def _finite_gradient(cost, value, epsilon=1e-7):
    gradient = np.zeros_like(value)
    for index in range(value.size):
        offset = np.zeros_like(value)
        offset[index] = epsilon
        gradient[index] = (
            cost(value + offset) - cost(value - offset)
        ) / (2.0 * epsilon)
    return gradient


def test_running_action_derivatives_match_finite_differences():
    q, running, _ = _action_models()
    velocity = np.array([0.2, -0.15, 0.1, -0.05])
    data = running.createData()
    running.calc(data, q, velocity)
    running.calcDiff(data, q, velocity)

    def x_cost(value):
        local_data = running.createData()
        running.calc(local_data, value, velocity)
        return local_data.cost

    def u_cost(value):
        local_data = running.createData()
        running.calc(local_data, q, value)
        return local_data.cost

    assert np.allclose(data.Lx, _finite_gradient(x_cost, q), atol=1e-7)
    assert np.allclose(
        data.Lu,
        _finite_gradient(u_cost, velocity),
        atol=1e-7,
    )
    assert np.allclose(data.Fx, np.eye(4))
    assert np.allclose(data.Fu, 0.02 * np.eye(4))


def test_terminal_com_gradient_matches_finite_differences():
    q, _, terminal = _action_models()
    data = terminal.createData()
    terminal.calc(data, q)
    terminal.calcDiff(data, q)

    def cost(value):
        local_data = terminal.createData()
        terminal.calc(local_data, value)
        return local_data.cost

    numerical = _finite_gradient(cost, q, epsilon=1e-6)

    assert np.allclose(data.Lx, numerical, atol=2e-4, rtol=2e-3)
    assert np.all(np.linalg.eigvalsh(data.Lxx) >= -1e-10)


def test_action_data_owns_private_pinocchio_data():
    _, _, terminal = _action_models()

    first = terminal.createData()
    second = terminal.createData()

    assert first.pin_data is not second.pin_data


def test_solver_returns_four_bounded_controls_and_diagnostics():
    full_model = _chain_model(27, free_flyer=True)
    body_model = _chain_model(27, free_flyer=False)
    robot = SimpleNamespace(
        model=full_model,
        model_body=body_model,
    )
    config = {
        'controller': {'dq_lim': 0.8},
        'limits': {
            'q_clip_limits': np.tile([-1.5, 1.5], (27, 1)),
            'dq_clip_limits': np.full(27, 0.5),
        },
        'counter_balance': {'maxiter': 10},
    }
    ddp = CounterBalanceDDP(robot, dt=0.02, config=config, arm='right')
    full_q = pin.neutral(full_model)
    com = pin.centerOfMass(full_model, full_model.createData(), full_q)

    plan = ddp.solve(full_q, com[:2] + [0.01, 0.005])

    assert plan.status in ('solved', 'best_effort')
    assert plan.command.shape == (4,)
    assert np.all(np.isfinite(plan.command))
    assert np.all(np.abs(plan.command) <= 0.5 + 1e-9)
    assert np.isfinite(plan.zero_com_error)
    assert np.isfinite(plan.optimized_com_error)


def test_solver_rejects_nonfinite_and_out_of_bounds_inputs():
    full_model = _chain_model(27, free_flyer=True)
    body_model = _chain_model(27, free_flyer=False)
    robot = SimpleNamespace(model=full_model, model_body=body_model)
    ddp = CounterBalanceDDP(robot, dt=0.02, config={}, arm='left')
    full_q = pin.neutral(full_model)

    full_q[7 + ddp.motor_ids[0]] = np.nan
    assert ddp.solve(full_q, [0.0, 0.0]).status == 'invalid_input'

    full_q = pin.neutral(full_model)
    full_q[7 + ddp.motor_ids[0]] = 3.0
    assert ddp.solve(full_q, [0.0, 0.0]).status == 'state_out_of_bounds'
