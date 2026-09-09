from types import SimpleNamespace
from dataclasses import FrozenInstanceError, replace

import pytest
import numpy as np

from h12_ros2_controller.core.controller.counter_balance import (
    scipy_nominal_planner as planner,
)
from h12_ros2_controller.core.controller.counter_balance.objective import (
    reaction_targets,
    bounded_velocity_problem,
    CounterVelocityBoundsError,
)


def _config():
    return planner.ScipyNominalConfig(2.0, 3.0, 0.4, 0.2, 0.3, 0.7, 1.2)


def _nominal(balance=0.6, config=None):
    com = np.array([[1., 2., -1., 0.], [0., 1., 2., 1.]])
    momentum = np.array([[2., -1., 0., 1.], [1., 0., -1., 2.]])
    rhs = reaction_targets(
        np.eye(2), np.eye(2), np.array([0.2, -0.3]),
        np.array([0.1, 0.2]), np.array([0.3, -0.4, 0.]), balance, 2., 0.1,
    )
    return (com, momentum, *rhs, np.array([0.2, -0.1, 0.3, -0.4]),
            -np.ones(4), np.ones(4), balance, config or _config())


@pytest.mark.parametrize('balance', [0.0, 0.3, 1.0])
@pytest.mark.parametrize('posture,damping', [(0.4, 0.2), (0., 0.2), (0.4, 0.)])
def test_exact_affine_builder_and_solution(balance, posture, damping):
    config = replace(_config(), posture_weight=posture, damping=damping)
    args = _nominal(balance, config)
    copies = [value.copy() for value in args[:7]]
    solved = planner.solve_scipy_nominal(*args)
    matrix, target = bounded_velocity_problem(
        *args[:5], balance, *vars(config).values(),
    )
    np.testing.assert_array_equal(solved.objective_matrix, matrix)
    np.testing.assert_array_equal(solved.objective_target, target)
    expected = planner.lsq_linear(matrix, target, bounds=args[5:7],
                                  method='bvls', tol=1e-10)
    assert solved.accepted
    np.testing.assert_allclose(solved.requested_counter_dq, expected.x, atol=1e-9)
    assert solved.diagnostics.solve_time >= sum(
        attempt.solve_time for attempt in solved.diagnostics.attempts
    )
    for original, copy in zip(args[:7], copies):
        np.testing.assert_array_equal(original, copy)
    for array in (solved.requested_counter_dq, solved.objective_matrix,
                  solved.objective_target):
        with pytest.raises(ValueError):
            array.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        solved.diagnostics.cost = 0.


def test_saturated_fixed_and_shifted_problem(monkeypatch):
    matrix = np.array([[1., 2., 0., 1.], [0., 1., 1., 2.],
                       [2., 0., 1., 0.], [0., 0., 1., 1.]])
    target = np.array([8., -3., 4., 2.])
    lower = np.array([0.25, -1., 0.1, -1.])
    upper = np.array([0.25, 1., 0.1, 1.])
    original = planner.lsq_linear
    calls = []

    def solve(a, b, **kwargs):
        calls.append(kwargs)
        np.testing.assert_array_equal(a, matrix[:, [1, 3]])
        np.testing.assert_allclose(b, target - matrix[:, [0, 2]] @ lower[[0, 2]])
        return original(a, b, **kwargs)

    monkeypatch.setattr(planner, 'lsq_linear', solve)
    result = planner.solve_bounded_least_squares(matrix, target, lower, upper)
    assert result.accepted
    assert len(calls) == 1
    np.testing.assert_array_equal(result.velocity[[0, 2]], lower[[0, 2]])
    assert np.any(np.abs(result.velocity[[1, 3]]) == 1.)
    assert result.regularization == 0.
    assert not result.boxqp_polished


def test_all_fixed_without_solver(monkeypatch):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: pytest.fail('called'))
    bounds = np.array([0.2, -0.3, 0.4, 0.1])
    result = planner.solve_bounded_least_squares(np.eye(4), np.ones(4), bounds, bounds)
    assert result.accepted and result.backend == 'fixed'
    assert result.kkt_violation == 0.
    np.testing.assert_array_equal(result.velocity, bounds)


def test_narrow_nonzero_box_stays_free(monkeypatch):
    lower = np.ones(4)
    upper = lower + 1e-12

    def solve(a, b, **kwargs):
        assert a.shape == (4, 4)
        np.testing.assert_array_equal(kwargs['bounds'], (lower, upper))
        return SimpleNamespace(x=upper.copy(), success=False, nit=0)

    monkeypatch.setattr(planner, 'lsq_linear', solve)
    result = planner.solve_bounded_least_squares(np.eye(4), 2 * upper, lower, upper)
    assert result.accepted and not result.converged
    np.testing.assert_array_equal(result.velocity, upper)


@pytest.mark.parametrize('bad', [np.full(4, np.nan), np.full(4, np.inf),
                                     np.full(4, 2.), np.zeros(4), np.zeros((4, 1)),
                                     'exception'])
def test_bvls_rejection_gets_exactly_one_trf(monkeypatch, bad):
    calls = []

    def solve(a, b, **kwargs):
        calls.append(kwargs)
        if len(calls) == 1 and isinstance(bad, str):
            raise RuntimeError('backend failed')
        return SimpleNamespace(
            x=bad if len(calls) == 1 else np.full(4, 0.5),
            success=True, cost=-100., optimality=0., active_mask=np.ones(4), nit=2,
        )

    monkeypatch.setattr(planner, 'lsq_linear', solve)
    result = planner.solve_bounded_least_squares(
        np.eye(4), np.full(4, 0.5), -np.ones(4), np.ones(4),
    )
    assert result.accepted and result.retry_used and not result.primary_accepted
    assert [call['method'] for call in calls] == ['bvls', 'trf']
    for call in calls:
        assert call['lsq_solver'] == 'exact'
        assert call['tol'] == 1e-10 and call['max_iter'] == 100
    assert not result.attempts[0].accepted
    assert result.cost == 0. and result.kkt_violation == 0.
    assert result.iterations == sum(a.iterations for a in result.attempts)


@pytest.mark.parametrize('candidate,status', [
    (np.full(4, 0.2), 'poor_kkt'),
    (np.full(4, -0.2), 'objective_regression'),
    (np.full(4, 1. + 1e-12), 'out_of_bounds'),
])
def test_dual_reject_returns_placeholder(monkeypatch, candidate, status):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: SimpleNamespace(
        x=candidate, success=True, cost=0., optimality=0., nit=1,
    ))
    result = planner.solve_bounded_least_squares(
        np.eye(4), np.full(4, 0.5), -np.ones(4), np.ones(4),
    )
    assert not result.accepted and result.retry_used
    assert len(result.attempts) == 2
    assert result.status == status
    np.testing.assert_array_equal(result.velocity, np.zeros(4))


def test_narrow_interior_not_mistaken_for_fixed(monkeypatch):
    lower, upper = np.ones(4), np.ones(4) + 1e-6
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: SimpleNamespace(
        x=lower + 5e-7, success=True, nit=1,
    ))
    result = planner.solve_bounded_least_squares(np.eye(4), 2 * upper, lower, upper)
    assert not result.accepted
    assert result.kkt_violation > planner.KKT_TOLERANCE


@pytest.mark.parametrize('index,value', [
    (0, np.ones((2, 3))), (0, np.empty((0, 4))), (0, [[np.nan] * 4]),
    (0, 'bad'), (1, np.zeros((4, 1))), (1, np.full(4, np.inf)),
    (2, np.zeros(3)), (3, np.full(4, np.nan)),
])
def test_invalid_inputs_no_retry(monkeypatch, index, value):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: pytest.fail('called'))
    args = [np.eye(4), np.zeros(4), -np.ones(4), np.ones(4)]
    args[index] = value
    result = planner.solve_bounded_least_squares(*args)
    assert not result.accepted and not result.retry_used and not result.attempts
    assert np.all(np.isfinite(result.velocity))


def test_empty_box_preserves_bounds_error(monkeypatch):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: pytest.fail('called'))
    with pytest.raises(CounterVelocityBoundsError):
        planner.solve_bounded_least_squares(
            np.eye(4), np.zeros(4), np.ones(4), -np.ones(4),
        )
    args = list(_nominal())
    args[5], args[6] = args[6], args[5]
    with pytest.raises(CounterVelocityBoundsError):
        planner.solve_scipy_nominal(*args)


@pytest.mark.parametrize('index,value', [
    (0, np.zeros((3, 4))), (1, np.full((2, 4), np.nan)),
    (2, np.zeros((2, 1))), (3, np.zeros(3)), (4, np.full(4, np.inf)),
    (7, np.nan), (7, -1.), (7, [1.]),
    (8, replace(_config(), damping=-1.)),
    (8, replace(_config(), com_velocity_scale=0.)),
    (8, replace(_config(), momentum_scale=np.inf)),
    (8, replace(_config(), posture_velocity_scale=-1.)),
    (8, replace(_config(), com_weight=np.nan)),
    (8, planner.ScipyNominalConfig(0., 0., 0., 0., 1., 1., 1.)),
])
def test_invalid_nominal_no_solve(monkeypatch, index, value):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: pytest.fail('called'))
    args = list(_nominal())
    args[index] = value
    result = planner.solve_scipy_nominal(*args)
    assert not result.accepted and not result.diagnostics.attempts


def test_objective_regression_rejected_even_when_kkt_passes(monkeypatch):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: SimpleNamespace(
        x=np.full(4, 1e-4), success=True, nit=1, cost=0., optimality=0.,
    ))
    result = planner.solve_bounded_least_squares(
        np.eye(4), np.zeros(4), -np.ones(4), np.ones(4),
    )
    assert result.kkt_violation < planner.KKT_TOLERANCE
    assert not result.accepted and result.status == 'objective_regression'


def test_backend_cannot_mutate_inputs_or_retry_problem(monkeypatch):
    matrix, target = np.eye(4), np.full(4, 0.5)
    lower, upper = -np.ones(4), np.ones(4)
    calls = []

    def solve(a, b, **kwargs):
        np.testing.assert_array_equal(a, matrix)
        np.testing.assert_array_equal(b, target)
        np.testing.assert_array_equal(kwargs['bounds'], (lower, upper))
        calls.append(kwargs['method'])
        a[:] = 0.
        b[:] = 0.
        kwargs['bounds'][0][:] = -100.
        return SimpleNamespace(x=np.zeros(4), success=True, nit=1)

    monkeypatch.setattr(planner, 'lsq_linear', solve)
    result = planner.solve_bounded_least_squares(matrix, target, lower, upper)
    assert not result.accepted
    assert calls == ['bvls', 'trf']
    np.testing.assert_array_equal(matrix, np.eye(4))
    np.testing.assert_array_equal(target, np.full(4, 0.5))
    np.testing.assert_array_equal(lower, -np.ones(4))
    np.testing.assert_array_equal(upper, np.ones(4))


def test_real_narrow_solve_without_widening():
    lower = np.array([1., -0.2, 0., -1.])
    upper = np.array([1. + 1e-12, 0.4, 0., 1.])
    result = planner.solve_bounded_least_squares(
        np.eye(4), np.array([2., 0.1, 3., -2.]), lower, upper,
    )
    assert result.accepted
    np.testing.assert_array_equal(result.velocity[[0, 2, 3]],
                                  [upper[0], 0., -1.])
    assert result.velocity[1] == pytest.approx(0.1)


def test_nonfinite_derived_objective_never_accepted(monkeypatch):
    monkeypatch.setattr(planner, 'lsq_linear', lambda *a, **k: pytest.fail('called'))
    result = planner.solve_bounded_least_squares(
        np.eye(4) * 1e308, np.zeros(4), np.full(4, 2.), np.full(4, 2.),
    )
    assert not result.accepted and not result.attempts
