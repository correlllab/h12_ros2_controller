import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    NX,
    ResidualH2Context,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h3_ocp import (
    ResidualH3ActionModel,
    ResidualH3OCP,
)


def _context():
    return ResidualH2Context(
        nominal_tilt=np.array([[0.03, -0.02], [0.025, -0.015]]),
        nominal_rate=np.array([[0.1, -0.08], [0.06, -0.04]]),
        u5_gain=np.array([0.7, 0.6, 0.0, 0.8]),
        r5_gain=np.array([[-0.1, 0.04], [-0.05, -0.3]]),
        momentum_map=np.array([
            [0.5, -0.2, 0.1, 0.3],
            [0.1, 0.4, -0.2, -0.3],
        ]),
        confidence=np.array([True, True]),
    )


@pytest.mark.parametrize('stage', [0, 1, 2])
def test_h3_action_derivatives_match_finite_differences(stage):
    model = ResidualH3ActionModel(0.02, stage, 0.01, 0.005)
    model.update(
        _context(),
        np.array([0.4, 0.5, 0.0, 0.3]),
        -0.01 * np.ones(4),
        0.01 * np.ones(4),
    )
    x = np.linspace(-0.03, 0.04, NX)
    u = np.array([0.003, -0.002, 0.0, 0.001])
    data = model.createData()
    model.calc(data, x, u)
    model.calcDiff(data, x, u)
    epsilon = 1e-7
    fx = np.empty_like(data.Fx)
    fu = np.empty_like(data.Fu)
    for index in range(NX):
        delta = np.zeros(NX)
        delta[index] = epsilon
        plus = model.createData()
        minus = model.createData()
        model.calc(plus, x + delta, u)
        model.calc(minus, x - delta, u)
        fx[:, index] = (plus.xnext - minus.xnext) / (2.0 * epsilon)
    for index in range(4):
        delta = np.zeros(4)
        delta[index] = epsilon
        plus = model.createData()
        minus = model.createData()
        model.calc(plus, x, u + delta)
        model.calc(minus, x, u - delta)
        fu[:, index] = (plus.xnext - minus.xnext) / (2.0 * epsilon)

    assert np.allclose(data.Fx, fx, atol=1e-8)
    assert np.allclose(data.Fu, fu, atol=1e-8)


def test_h3_solver_reuses_fixed_problem_and_masks_joint_two():
    ocp = ResidualH3OCP(max_iterations=1)
    lower = np.array([-0.01, -0.01, 0.0, -0.01])
    upper = np.array([0.01, 0.01, 0.0, 0.01])
    solver = ocp.solver

    result = ocp.solve(
        _context(),
        np.array([0.4, 0.5, 0.0, 0.3]),
        lower,
        upper,
    )

    assert result.accepted
    assert result.xs.shape == (4, NX)
    assert result.us.shape == (3, 4)
    assert np.all(np.abs(result.us[:, 2]) <= 1e-8)
    assert ocp.solver is solver
