import numpy as np
from scipy.optimize import lsq_linear

from h12_ros2_controller.core.controller.counter_balance.counter_velocity_ocp import (
    CounterVelocityOCP,
    VelocityActionModel,
)


def test_velocity_action_derivatives_match_finite_difference():
    rng = np.random.default_rng(3)
    model = VelocityActionModel(0.02)
    matrix = rng.normal(size=(12, 4))
    target = rng.normal(size=12)
    model.update(matrix, target, -np.ones(4), np.ones(4))
    data = model.createData()
    x = rng.normal(size=4)
    u = rng.normal(size=4) * 0.2
    model.calc(data, x, u)
    model.calcDiff(data, x, u)
    epsilon = 1e-6
    gradient = np.zeros(4)
    for index in range(4):
        perturbation = np.zeros(4)
        perturbation[index] = epsilon
        plus = model.createData()
        minus = model.createData()
        model.calc(plus, x, u + perturbation)
        model.calc(minus, x, u - perturbation)
        gradient[index] = (plus.cost - minus.cost) / (2.0 * epsilon)

    assert np.allclose(data.Lu, gradient, atol=1e-6, rtol=1e-5)
    assert np.allclose(data.Luu, matrix.T @ matrix)


def test_box_fddp_matches_scipy_bounded_least_squares():
    rng = np.random.default_rng(7)
    ocp = CounterVelocityOCP(max_iterations=100)
    for _ in range(50):
        matrix = rng.normal(size=(12, 4))
        target = rng.normal(size=12)
        lower = rng.uniform(-2.0, -0.1, size=4)
        upper = rng.uniform(0.1, 2.0, size=4)
        expected = lsq_linear(
            matrix, target, bounds=(lower, upper),
        ).x

        result = ocp.solve(
            np.zeros(4), matrix, target, lower, upper,
        )

        assert result.accepted
        assert np.allclose(result.velocity, expected, atol=1e-5, rtol=1e-5)


def test_velocity_ocp_enforces_active_bounds():
    ocp = CounterVelocityOCP(max_iterations=100)
    matrix = np.eye(4)
    target = np.full(4, 10.0)

    result = ocp.solve(
        np.zeros(4),
        matrix,
        target,
        -np.ones(4),
        np.array([0.2, 0.3, 0.4, 0.5]),
    )

    assert np.allclose(
        result.velocity, [0.2, 0.3, 0.4, 0.5], atol=2e-5,
    )
