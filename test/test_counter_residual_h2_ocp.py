import crocoddyl
import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    COUNTER_Q,
    NX,
    PENDING,
    ResidualH2ActionModel,
    ResidualH2Context,
    ResidualH2OCP,
    ResidualH2TerminalModel,
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


def _finite_differences(model, x, u, epsilon=1e-7):
    fx = np.empty((NX, NX))
    fu = np.empty((NX, len(u)))
    lx = np.empty(NX)
    lu = np.empty(len(u))
    for index in range(NX):
        plus = model.createData()
        minus = model.createData()
        delta = np.zeros(NX)
        delta[index] = epsilon
        model.calc(plus, x + delta, u)
        model.calc(minus, x - delta, u)
        fx[:, index] = (plus.xnext - minus.xnext) / (2.0 * epsilon)
        lx[index] = (plus.cost - minus.cost) / (2.0 * epsilon)
    for index in range(len(u)):
        plus = model.createData()
        minus = model.createData()
        delta = np.zeros(len(u))
        delta[index] = epsilon
        model.calc(plus, x, u + delta)
        model.calc(minus, x, u - delta)
        fu[:, index] = (plus.xnext - minus.xnext) / (2.0 * epsilon)
        lu[index] = (plus.cost - minus.cost) / (2.0 * epsilon)
    return fx, fu, lx, lu


@pytest.mark.parametrize('stage', [0, 1])
def test_h2_action_calc_diff_matches_finite_differences(stage):
    model = ResidualH2ActionModel(0.02, stage, 1.0, 0.5)
    model.update(_context(), -0.1 * np.ones(4), 0.1 * np.ones(4))
    x = np.linspace(-0.03, 0.04, NX)
    u = np.array([0.03, -0.02, 0.0, 0.01])
    data = model.createData()

    model.calc(data, x, u)
    model.calcDiff(data, x, u)
    fx, fu, lx, lu = _finite_differences(model, x, u)

    assert np.allclose(data.Fx, fx, atol=1e-8)
    assert np.allclose(data.Fu, fu, atol=1e-8)
    assert np.allclose(data.Lx, lx, atol=1e-7)
    assert np.allclose(data.Lu, lu, atol=1e-7)


def test_h2_terminal_calc_diff_matches_finite_difference_gradient():
    model = ResidualH2TerminalModel({
        'tilt': 2.0,
        'rate': 1.0,
        'divergence': 2.0,
        'reserve': 0.1,
    })
    model.update(_context())
    x = np.zeros(NX)
    x[COUNTER_Q] = [0.01, -0.02, 0.0, 0.01]
    data = model.createData()
    model.calc(data, x)
    model.calcDiff(data, x)
    numeric = np.empty(NX)
    epsilon = 1e-7
    for index in range(NX):
        delta = np.zeros(NX)
        delta[index] = epsilon
        plus = model.createData()
        minus = model.createData()
        model.calc(plus, x + delta)
        model.calc(minus, x - delta)
        numeric[index] = (plus.cost - minus.cost) / (2.0 * epsilon)

    assert np.allclose(data.Lx, numeric, atol=1e-5)


def test_h2_solver_keeps_weak_joint_fixed_and_reuses_solver():
    ocp = ResidualH2OCP(max_iterations=1)
    lower = np.array([-0.1, -0.1, 0.0, -0.1])
    upper = np.array([0.1, 0.1, 0.0, 0.1])
    solver = ocp.solver

    first = ocp.solve(_context(), lower, upper)
    second = ocp.solve(_context(), lower, upper)

    assert first.accepted
    assert second.accepted
    assert ocp.solver is solver
    assert np.all(np.abs(first.us[:, 2]) <= 1e-8)
    assert second.warm_started


def test_h2_zero_bounds_produce_exact_zero_residual():
    ocp = ResidualH2OCP(max_iterations=1)

    result = ocp.solve(_context(), np.zeros(4), np.zeros(4))

    assert result.accepted
    assert np.allclose(result.residual, 0.0, atol=1e-8)
