import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_ocp import (
    CounterBalanceActionModel,
    CounterDDPOCP,
    CounterTerminalActionModel,
    FrozenBalanceKnot,
)


def _settings():
    return (
        {
            'com': 1.0,
            'momentum': 2.0,
            'posture': 0.02,
            'acceleration': 0.01,
            'velocity': 0.02,
            'limit': 10.0,
            'terminal_posture': 0.0,
            'terminal_velocity': 0.0,
        },
        {'com': 2.0, 'gyro': 0.2},
        {
            'com_velocity': 0.1,
            'momentum': 1.0,
            'posture': 1.0,
            'acceleration': 25.0,
            'velocity': 1.0,
        },
    )


def _knot():
    return FrozenBalanceKnot(
        q_bar=np.array([0.1, -0.2, 0.05, 0.2]),
        q_ref=np.zeros(4),
        com_bar=np.array([0.01, -0.02]),
        com_target=np.zeros(2),
        com_counter=np.array([
            [0.1, 0.0, 0.02, -0.01],
            [0.0, 0.12, -0.03, 0.02],
        ]),
        com_moving=np.arange(14, dtype=np.float64).reshape(2, 7) * 0.001,
        momentum_counter=np.array([
            [0.0, 0.1, 0.5, 0.0],
            [0.2, 0.0, 0.0, 0.4],
        ]),
        momentum_moving=np.arange(14, dtype=np.float64).reshape(2, 7) * 0.02,
        moving_dq=np.linspace(-0.4, 0.5, 7),
        gyro_xy=np.array([0.05, -0.03]),
        com_gate=np.array([1.0, 0.5]),
        momentum_gate=np.array([0.8, 1.0]),
        q_lower=-np.ones(4),
        q_upper=np.ones(4),
        velocity_limit=np.ones(4),
    )


def test_action_dynamics_use_constant_acceleration():
    weights, gains, scales = _settings()
    model = CounterBalanceActionModel(0.02, weights, gains, scales)
    model.update(_knot(), -25.0 * np.ones(4), 25.0 * np.ones(4))
    data = model.createData()
    x = np.array([0.1, -0.2, 0.05, 0.2, 0.3, -0.4, 0.2, -0.1])
    u = np.array([2.0, -3.0, 1.0, 4.0])

    model.calc(data, x, u)
    model.calcDiff(data, x, u)

    expected_q = x[:4] + 0.02 * x[4:] + 0.5 * 0.02 ** 2 * u
    expected_dq = x[4:] + 0.02 * u
    assert np.allclose(data.xnext, np.concatenate([expected_q, expected_dq]))
    assert np.allclose(data.Fx[:4, 4:], 0.02 * np.eye(4))
    assert np.allclose(data.Fu[:4], 0.5 * 0.02 ** 2 * np.eye(4))
    assert np.allclose(data.Fu[4:], 0.02 * np.eye(4))


def test_action_derivatives_match_finite_differences():
    weights, gains, scales = _settings()
    model = CounterBalanceActionModel(0.02, weights, gains, scales)
    model.update(_knot(), -25.0 * np.ones(4), 25.0 * np.ones(4))
    data = model.createData()
    x = np.array([0.1, -0.2, 0.05, 0.2, 0.3, -0.4, 0.2, -0.1])
    u = np.array([2.0, -3.0, 1.0, 4.0])
    model.calc(data, x, u)
    model.calcDiff(data, x, u)
    eps = 1e-6

    def cost(x_value, u_value):
        value_data = model.createData()
        model.calc(value_data, x_value, u_value)
        return value_data.cost

    lx = np.array([
        (cost(x + eps * np.eye(8)[i], u)
         - cost(x - eps * np.eye(8)[i], u)) / (2.0 * eps)
        for i in range(8)
    ])
    lu = np.array([
        (cost(x, u + eps * np.eye(4)[i])
         - cost(x, u - eps * np.eye(4)[i])) / (2.0 * eps)
        for i in range(4)
    ])
    assert np.allclose(data.Lx, lx, atol=1e-6, rtol=1e-5)
    assert np.allclose(data.Lu, lu, atol=1e-6, rtol=1e-5)


def test_terminal_model_has_no_control():
    weights, _, scales = _settings()
    model = CounterTerminalActionModel(scales, weights)
    model.update(np.zeros(4))
    data = model.createData()
    model.calc(data, np.zeros(8))
    model.calcDiff(data, np.zeros(8))

    assert model.nu == 0
    assert data.Fu.shape == (8, 0)


def test_box_fddp_returns_bounded_finite_trajectory_and_warm_starts():
    ocp = CounterDDPOCP(horizon_steps=5, max_iterations=2)
    knots = [_knot() for _ in range(5)]
    lower = -2.0 * np.ones((5, 4))
    upper = 2.0 * np.ones((5, 4))

    first = ocp.solve(np.zeros(8), knots, lower, upper, np.zeros(4))
    second = ocp.solve(np.zeros(8), knots, lower, upper, np.zeros(4))

    assert first.accepted
    assert first.xs.shape == (6, 8)
    assert first.us.shape == (5, 4)
    assert np.all(first.us >= lower - 1e-8)
    assert np.all(first.us <= upper + 1e-8)
    assert np.isfinite(first.optimized_cost)
    assert first.optimized_cost <= first.seed_cost + 1e-10
    assert not first.warm_started
    assert second.warm_started
